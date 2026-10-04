"""Vehicle definition (YAML) -> kinematic tree with physically derived mass properties.

Shared by gen_urdf.py and the tests. Pure Python + numpy, no ROS / Blender imports.
Every link's mass/CoM/inertia is assembled from simple solids (thin shells, solid
propellant columns, boxes) with the parallel-axis theorem, so the numbers follow
from the geometry in the YAML instead of being typed in by hand.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

GIMBAL_STUB_MASS = 1.0   # kg, intermediate link between two revolute gimbal axes


# ---------------------------------------------------------------- mass primitives
@dataclass
class Body:
    """Rigid mass element: mass, CoM (link frame), inertia about its CoM (link axes)."""
    m: float
    c: np.ndarray
    I: np.ndarray

    @staticmethod
    def make(m, c, Idiag, R=None):
        I = np.diag(Idiag).astype(float)
        if R is not None:
            I = R @ I @ R.T
        return Body(float(m), np.asarray(c, float), I)


def combine(bodies: list[Body]) -> Body:
    m = sum(b.m for b in bodies)
    c = sum(b.m * b.c for b in bodies) / m
    I = np.zeros((3, 3))
    for b in bodies:
        d = b.c - c
        I += b.I + b.m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return Body(m, c, I)


def shell_cylinder(m, r, z0, z1, xy=(0.0, 0.0)):
    L = z1 - z0
    return Body.make(m, (xy[0], xy[1], (z0 + z1) / 2), (m * (r * r / 2 + L * L / 12),) * 2 + (m * r * r,))


def solid_cylinder(m, r, z0, z1, xy=(0.0, 0.0)):
    L = z1 - z0
    return Body.make(m, (xy[0], xy[1], (z0 + z1) / 2), (m * (3 * r * r + L * L) / 12,) * 2 + (m * r * r / 2,))


def box(m, size, c, R=None):
    x, y, z = size
    return Body.make(m, c, (m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12), R)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


# ---------------------------------------------------------------- kinematic tree
@dataclass
class Collision:
    kind: str                 # 'cylinder' | 'box'
    xyz: tuple
    rpy: tuple = (0.0, 0.0, 0.0)
    radius: float = 0.0
    length: float = 0.0
    size: tuple = (0.0, 0.0, 0.0)


@dataclass
class Link:
    name: str
    inertial: Body | None = None
    mesh: str | None = None                   # mesh file name (in meshes/<vehicle>/)
    mesh_rpy: tuple = (0.0, 0.0, 0.0)
    collisions: list[Collision] = field(default_factory=list)


@dataclass
class Joint:
    name: str
    type: str                 # fixed | revolute
    parent: str
    child: str
    xyz: tuple
    rpy: tuple = (0.0, 0.0, 0.0)
    axis: tuple = (0.0, 0.0, 1.0)
    lower: float = 0.0
    upper: float = 0.0
    effort: float = 1.0e7
    velocity: float = 1.0
    role: str = ""            # gimbal | fin_deploy | fin_steer | leg_deploy | separation
    preserve: bool = False    # keep fixed joint as its own link in SDF (needed for staging)


@dataclass
class Vehicle:
    name: str
    spec: dict
    links: list[Link] = field(default_factory=list)
    joints: list[Joint] = field(default_factory=list)

    def link(self, name):
        return next(l for l in self.links if l.name == name)

    def total_mass(self):
        return sum(l.inertial.m for l in self.links if l.inertial)


# ---------------------------------------------------------------- builders
def _propellant_columns(st, dens, stage_r):
    """Split stage propellant into LOX / RP-1 and spread each uniformly over its tank."""
    mp, mr = st["propellant_mass"], st["mixture_ratio"]
    m_lox, m_rp1 = mp * mr / (1 + mr), mp / (1 + mr)
    out, report = [], {}
    for tank, m, rho in (("lox_tank", m_lox, dens["lox"]), ("rp1_tank", m_rp1, dens["rp1"])):
        z0, z1 = st["sections"][tank]
        cap = math.pi * stage_r ** 2 * (z1 - z0) * rho
        report[tank] = dict(mass=m, capacity=cap, fill=m / cap)
        # fill from the bottom of the tank (settled propellant)
        h = min((z1 - z0), (z1 - z0) * m / cap)
        out.append(solid_cylinder(m, stage_r, z0, z0 + h))
    return out, report


def _engine_body(model):
    # Powerhead/chamber heavy near the pivot: CoM ~40 % of the length below the pivot;
    # inertia of a solid cylinder at ~75 % of the exit radius (approximation, documented).
    m, L, r = model["mass"] - GIMBAL_STUB_MASS, model["length"], 0.75 * model["exit_radius"]
    return Body.make(m, (0, 0, -0.4 * L), (m * (3 * r * r + L * L) / 12,) * 2 + (m * r * r / 2,))


def _stub():
    return Body.make(GIMBAL_STUB_MASS, (0, 0, 0), (1e-3, 1e-3, 1e-3))


def _add_engines(v, st, parent, prefix):
    em = v.spec["engine_models"][st["engines"]["model"]]
    lim = math.radians(em["gimbal_limit_deg"])
    E = st["engines"]
    positions = [(0.0, 0.0)]
    if E["layout"] == "octaweb":
        positions += [(E["ring_radius"] * math.cos(k * math.pi / 4), E["ring_radius"] * math.sin(k * math.pi / 4))
                      for k in range(8)]
    positions = positions[: E["count"]]
    mesh = f"{st['engines']['model']}.glb"
    for i, (x, y) in enumerate(positions, 1):
        n = f"{prefix}_engine_{i}" if E["count"] > 1 else f"{prefix}_engine"
        L = em["length"]
        v.links.append(Link(f"{n}_gimbal", _stub()))
        v.links.append(Link(n, _engine_body(em), mesh=mesh,
                            collisions=[Collision("cylinder", (0, 0, -0.55 * L), radius=em["exit_radius"] * 0.9,
                                                  length=0.9 * L)]))
        v.joints.append(Joint(f"{n}_gimbal_pitch", "revolute", parent, f"{n}_gimbal", (x, y, E["gimbal_z"]),
                              axis=(1, 0, 0), lower=-lim, upper=lim, effort=2.0e5, velocity=0.5, role="gimbal"))
        v.joints.append(Joint(f"{n}_gimbal_yaw", "revolute", f"{n}_gimbal", n, (0, 0, 0),
                              axis=(0, 1, 0), lower=-lim, upper=lim, effort=2.0e5, velocity=0.5, role="gimbal"))
    return len(positions) * em["mass"]


def _add_grid_fins(v, st, parent):
    G = st["grid_fins"]
    r0 = st["radius"] + G["radial_offset"]
    for k in range(G["count"]):
        a = math.radians(G["azimuth0_deg"]) + 2 * math.pi * k / G["count"]
        n = f"grid_fin_{k + 1}"
        # fin frame: x radial (lattice depth), y tangential (hinge axis), z along the stowed fin
        body = box(G["mass_each"] - GIMBAL_STUB_MASS, (G["depth"], G["width"], G["length"]), (0, 0, G["length"] / 2))
        v.links.append(Link(f"{n}_hinge", _stub()))
        v.links.append(Link(n, body, mesh="grid_fin.glb",
                            collisions=[Collision("box", (0, 0, G["length"] / 2),
                                                  size=(G["depth"], G["width"], G["length"]))]))
        v.joints.append(Joint(f"{n}_deploy", "revolute", parent, f"{n}_hinge",
                              (r0 * math.cos(a), r0 * math.sin(a), G["hinge_z"]), rpy=(0, 0, a),
                              axis=(0, 1, 0), lower=0.0, upper=math.radians(G["deploy_deg"]),
                              effort=5.0e5, velocity=0.6, role="fin_deploy"))
        lim = math.radians(G["steer_limit_deg"])
        v.joints.append(Joint(f"{n}_steer", "revolute", f"{n}_hinge", n, (0, 0, 0), axis=(0, 0, 1),
                              lower=-lim, upper=lim, effort=5.0e5, velocity=1.0, role="fin_steer"))
    return G["count"] * G["mass_each"]


def _add_legs(v, st, parent):
    Lg = st["legs"]
    r0 = st["radius"] + Lg["radial_offset"]
    for k in range(Lg["count"]):
        a = math.radians(Lg["azimuth0_deg"]) + 2 * math.pi * k / Lg["count"]
        n = f"leg_{k + 1}"
        L = Lg["length"]
        body = combine([box(0.85 * Lg["mass_each"], (0.4, 0.75, L), (0, 0, L * 0.45)),
                        solid_cylinder(0.15 * Lg["mass_each"], 0.55, L - 0.1, L)])
        v.links.append(Link(n, body, mesh="landing_leg.glb",
                            collisions=[Collision("box", (0, 0, L / 2), size=(0.4, 0.75, L)),
                                        Collision("cylinder", (0, 0, L - 0.05), radius=0.55, length=0.1)]))
        v.joints.append(Joint(f"{n}_deploy", "revolute", parent, n,
                              (r0 * math.cos(a), r0 * math.sin(a), Lg["hinge_z"]), rpy=(0, 0, a),
                              axis=(0, 1, 0), lower=0.0, upper=math.radians(Lg["deploy_deg"]),
                              effort=5.0e6, velocity=0.5, role="leg_deploy"))
    return Lg["count"] * Lg["mass_each"]


def build(spec_path: str | Path) -> Vehicle:
    spec = yaml.safe_load(Path(spec_path).read_text())
    v = Vehicle(spec["name"], spec)
    v.report = {}
    dens = spec["propellant_densities"]
    v.links.append(Link("base_link"))

    for st in spec["stages"]:
        sn, r = st["name"], st["radius"]
        body = f"{sn}_body"
        if sn == "s1":
            v.joints.append(Joint("base_to_s1", "fixed", "base_link", body, (0, 0, 0)))
            attached = _add_engines(v, st, body, sn) + _add_grid_fins(v, st, body) + _add_legs(v, st, body)
        else:
            attached = _add_engines(v, st, body, sn)
        # stage structure = dry mass minus separately modelled parts and gimbal stubs
        m_struct = st["dry_mass"] - attached
        props, rep = _propellant_columns(st, dens, r - 0.02)
        v.report[sn] = dict(structure=m_struct, attached=attached, **rep)
        inert = combine([shell_cylinder(m_struct, r, 0.0, st["length"])] + props)
        v.links.append(Link(body, inert, mesh=f"{body}.glb",
                            collisions=[Collision("cylinder", (0, 0, st["length"] / 2), radius=r,
                                                  length=st["length"])]))
    # stage 2 attaches on top of stage 1 (future separation interface)
    s1, s2 = spec["stages"][0], spec["stages"][1]
    v.joints.append(Joint("s1_s2_separation", "fixed", "s1_body", "s2_body", (0, 0, s2["base_z"] - s1["base_z"]),
                          role="separation", preserve=True))

    F = spec["fairing"]
    zf = F["base_z"] - s2["base_z"]
    for side, yaw in (("a", 0.0), ("b", math.pi)):
        # half shell, CoM offset 2r/pi toward its side; inertia of half a thin shell
        mh, ra, H = F["mass"] / 2, F["radius"] * 0.95, F["height"]
        d = 2 * ra / math.pi
        Izz = mh * ra * ra - mh * d * d
        Ixx = mh * (ra * ra / 2 + H * H / 12)
        Iyy = Ixx - mh * d * d
        R = rot_z(yaw)
        b = Body(mh, R @ np.array([d, 0, H * 0.42]), R @ np.diag([Iyy, Ixx, Izz]) @ R.T)
        n = f"fairing_{side}"
        v.links.append(Link(n, b, mesh="fairing_half.glb", mesh_rpy=(0, 0, yaw),
                            collisions=[Collision("box", tuple(R @ np.array([F["radius"] / 2, 0, H * 0.4])),
                                                  rpy=(0, 0, yaw), size=(F["radius"], 2 * F["radius"], 0.8 * H))]))
        v.joints.append(Joint(f"fairing_{side}_separation", "fixed", "s2_body", n, (0, 0, zf),
                              role="separation", preserve=True))
    P = spec["payload"]
    v.links.append(Link("payload", box(P["mass"], P["size"], (0, 0, P["size"][2] / 2)), mesh="payload.glb",
                        collisions=[Collision("box", (0, 0, P["size"][2] / 2), size=tuple(P["size"]))]))
    v.joints.append(Joint("payload_separation", "fixed", "s2_body", "payload", (0, 0, P["base_z"] - s2["base_z"]),
                          role="separation", preserve=True))
    return v


def check_inertia(b: Body, tol=1e-9) -> list[str]:
    """Physical validity: symmetric, positive definite, triangle inequality on principal moments."""
    errs = []
    if not np.allclose(b.I, b.I.T):
        errs.append("not symmetric")
    p = np.linalg.eigvalsh(b.I)
    if (p <= 0).any():
        errs.append(f"non-positive principal moments {p}")
    a, bb, c = sorted(p)
    if a + bb < c * (1 - 1e-6):
        errs.append(f"triangle inequality violated {p}")
    return errs


def vehicle_frame_com(v: Vehicle, q: dict | None = None, links=None) -> Body:
    """Whole-vehicle (or a subset of links) mass properties in the vehicle frame (joints at q, default 0)."""
    q = q or {}
    T = {"base_link": np.eye(4)}
    pending = list(v.joints)
    while pending:
        for j in list(pending):
            if j.parent in T:
                Tj = np.eye(4)
                r, p, y = j.rpy
                Tj[:3, :3] = rot_z(y) @ _rot_y(p) @ _rot_x(r)
                Tj[:3, 3] = j.xyz
                if j.type == "revolute":
                    Tj = Tj @ _axis_rot(j.axis, q.get(j.name, 0.0))
                T[j.child] = T[j.parent] @ Tj
                pending.remove(j)
    bodies = []
    for l in v.links:
        if l.inertial and (links is None or l.name in links):
            R, t = T[l.name][:3, :3], T[l.name][:3, 3]
            bodies.append(Body(l.inertial.m, R @ l.inertial.c + t, R @ l.inertial.I @ R.T))
    return combine(bodies)


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _axis_rot(axis, a):
    M = np.eye(4)
    M[:3, :3] = {(1, 0, 0): _rot_x, (0, 1, 0): _rot_y, (0, 0, 1): rot_z}[tuple(int(x) for x in axis)](a)
    return M


def mass_properties_at(v: Vehicle, propellant: dict[str, float], links=None) -> Body:
    """Whole-vehicle mass properties (vehicle frame, joints at 0) with stage propellant masses
    given by `propellant` {stage_name: kg}. Propellant settles at the bottom of each tank and is
    split by the stage mixture ratio, exactly as rocket_propulsion::RocketPropulsion does."""
    full = vehicle_frame_com(v, links=links)
    dens = v.spec["propellant_densities"]
    bodies = [full]
    for st in v.spec["stages"]:
        if st["name"] not in propellant:
            continue
        base = st["base_z"]
        r = st["radius"] - 0.02
        mr = st["mixture_ratio"]
        for tank, frac, rho in (("lox_tank", mr / (1 + mr), dens["lox"]), ("rp1_tank", 1 / (1 + mr), dens["rp1"])):
            z0, z1 = st["sections"][tank]
            for m, sign in ((st["propellant_mass"] * frac, -1.0), (propellant[st["name"]] * frac, 1.0)):
                if m <= 0:
                    continue
                h = min(m / (rho * math.pi * r * r), z1 - z0)
                col = solid_cylinder(m, r, base + z0, base + z0 + h)
                bodies.append(Body(sign * col.m, col.c, sign * col.I))
    # combine() works with signed masses (removal of the initial columns)
    return combine(bodies)
