#!/usr/bin/env python3
"""Procedural textures for exhaust / smoke particle effects (deterministic, seed 7).

    python3 make_effect_textures.py <out_dir> [<out_dir> ...]

puff.png         256x256 RGBA soft billowy puff (white, alpha from radial falloff x fractal noise)
flame_ramp.png   colour over particle life: white-hot -> yellow -> RP-1 orange -> sooty red -> dark smoke
smoke_ramp.png   light grey -> grey -> darker grey (soot trail)
steam_ramp.png   bright white -> light grey (pad deluge steam / smoke)
"""
import sys
from pathlib import Path

import numpy as np
from PIL import Image


def fractal_noise(n, octaves=5, seed=7):
    rng = np.random.default_rng(seed)
    out = np.zeros((n, n))
    for o in range(octaves):
        k = 2 ** (o + 2)
        g = rng.random((k + 1, k + 1))
        xs = np.linspace(0, k, n)
        x0 = np.floor(xs).astype(int).clip(0, k - 1)
        fx = xs - x0
        fx = fx * fx * (3 - 2 * fx)
        a = g[np.ix_(x0, x0)]
        b = g[np.ix_(x0, x0 + 1)]
        c = g[np.ix_(x0 + 1, x0)]
        d = g[np.ix_(x0 + 1, x0 + 1)]
        top = a + (b - a) * fx[None, :]
        bot = c + (d - c) * fx[None, :]
        out += (top + (bot - top) * fx[:, None]) / 2 ** o
    return (out - out.min()) / (out.max() - out.min())


def puff(n=256):
    y, x = np.mgrid[-1:1:n * 1j, -1:1:n * 1j]
    r = np.sqrt(x * x + y * y)
    fall = np.clip(1 - r, 0, 1) ** 1.6
    noise = fractal_noise(n)
    a = np.clip(fall * (0.55 + 0.9 * noise) - 0.08, 0, 1)
    shade = 0.82 + 0.18 * noise
    rgb = (np.stack([shade] * 3, -1) * 255).astype(np.uint8)
    return Image.fromarray(np.dstack([rgb, (a * 255).astype(np.uint8)]), "RGBA")


def tinted_puff(core, mid, edge, n=256, hot=0.35, seed=11):
    """Puff whose colour runs core -> mid -> edge with radius (ogre2 ignores colour ranges)."""
    y, x = np.mgrid[-1:1:n * 1j, -1:1:n * 1j]
    r = np.sqrt(x * x + y * y)
    noise = fractal_noise(n, seed=seed)
    a = np.clip(np.clip(1 - r, 0, 1) ** 1.3 * (0.6 + 0.8 * noise) - 0.05, 0, 1)
    u = np.clip(r / 0.85 + 0.25 * (noise - 0.5), 0, 1)
    cols = np.array([core, mid, edge], float)
    rgb = np.empty((n, n, 3))
    for k in range(3):
        rgb[..., k] = np.where(u < hot, np.interp(u, [0, hot], [cols[0, k], cols[1, k]]),
                               np.interp(u, [hot, 1], [cols[1, k], cols[2, k]]))
    return Image.fromarray(np.dstack([rgb.clip(0, 255).astype(np.uint8), (a * 255).astype(np.uint8)]), "RGBA")


def ramp(stops, w=256):
    """stops: [(pos 0..1, (r,g,b,a))] -> 1-pixel-high RGBA gradient"""
    pos = np.array([p for p, _ in stops])
    cols = np.array([c for _, c in stops], float)
    xs = np.linspace(0, 1, w)
    img = np.stack([np.interp(xs, pos, cols[:, k]) for k in range(4)], -1)
    return Image.fromarray(np.repeat(img[None], 4, 0).astype(np.uint8), "RGBA")


def main():
    textures = {
        "puff.png": puff(),
        "flame_puff.png": tinted_puff((255, 252, 235), (255, 190, 70), (225, 95, 25)),
        "smoke_puff.png": tinted_puff((205, 200, 192), (170, 165, 158), (140, 136, 130), hot=0.5, seed=5),
        "flame_ramp.png": ramp([(0.0, (255, 255, 240, 255)), (0.12, (255, 236, 150, 255)),
                                (0.35, (255, 160, 50, 230)), (0.6, (210, 80, 25, 170)),
                                (0.85, (90, 60, 50, 90)), (1.0, (60, 55, 52, 0))]),
        "smoke_ramp.png": ramp([(0.0, (235, 232, 228, 200)), (0.5, (175, 172, 168, 140)),
                                (1.0, (140, 138, 135, 0))]),
        "steam_ramp.png": ramp([(0.0, (255, 255, 255, 230)), (0.6, (235, 235, 235, 170)),
                                (1.0, (210, 210, 210, 0))]),
    }
    for d in sys.argv[1:] or ["."]:
        Path(d).mkdir(parents=True, exist_ok=True)
        for name, im in textures.items():
            im.save(Path(d) / name)
        print("wrote", d, sorted(textures))


if __name__ == "__main__":
    main()
