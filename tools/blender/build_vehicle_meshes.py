"""Headless Blender: build glTF meshes for a vehicle from its YAML definition.

    blender --background --factory-startup --python build_vehicle_meshes.py -- \
        <vehicles/NAME.yaml> <output meshes dir>

Each mesh is authored in its URDF link frame (see tools/vehicle_model.py),
so no offsets are needed in the URDF visuals.
"""
import math
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bl_common as B  # noqa: E402

TAU = 2 * math.pi


def bell_profile(r_throat, z_throat, r_exit, z_exit, n=24, k=1.8, t=0.02):
    """Closed outline of a bell nozzle wall (outer skin down, lip, inner skin up)."""
    outer = []
    for i in range(n + 1):
        u = i / n
        r = r_throat + (r_exit - r_throat) * (1 - (1 - u) ** k)
        outer.append((r + t, z_throat + (z_exit - z_throat) * u))
    inner = [(r - t, z) for r, z in reversed(outer)]
    return outer + inner


def merlin(eng, name):
    L, re = eng["length"], eng["exit_radius"]
    vac = eng["thrust_sl"] == 0.0
    # gimbal block and powerhead
    B.box(f"{name}_gimbal_block", (0.32, 0.32, 0.22), (0, 0, -0.08), "dark_steel")
    B.cylinder(f"{name}_turbopump", 0.17, -0.85, -0.25, "dark_steel", loc=(0.30, 0, 0))
    B.cylinder(f"{name}_tp_housing", 0.21, -0.55, -0.45, "steel", loc=(0.30, 0, 0))
    B.cylinder(f"{name}_gg_exhaust", 0.06, -1.6 if not vac else -1.3, -0.8, "nozzle_dark", loc=(0.36, 0, 0))
    B.cylinder(f"{name}_lox_line", 0.07, -0.6, -0.2, "steel", loc=(-0.26, 0.12, 0))
    B.cylinder(f"{name}_fuel_line", 0.07, -0.6, -0.2, "steel", loc=(-0.26, -0.12, 0))
    # injector dome + combustion chamber (copper regen jacket) + throat
    zt = -1.12 if not vac else -1.0
    B.lathe(f"{name}_chamber", [(0.0, -0.18), (0.22, -0.2), (0.26, -0.32), (0.26, -0.85),
                                (0.20, zt + 0.12), (0.155, zt), (0.0, zt)], "copper", segs=64)
    B.lathe(f"{name}_manifold", [(0.27, -0.42), (0.30, -0.45), (0.30, -0.55), (0.27, -0.58)], "steel",
            segs=64, closed_loop=True)
    if not vac:
        B.lathe(f"{name}_bell", bell_profile(0.15, zt, re, -L, t=0.015), "nozzle_dark", segs=72,
                closed_loop=True)
        B.lathe(f"{name}_bell_ring", [(re + 0.01, -L + 0.06), (re + 0.035, -L + 0.06), (re + 0.035, -L),
                                      (re + 0.01, -L)], "steel", segs=72, closed_loop=True)
    else:
        # regen-cooled section, then the radiatively cooled niobium extension
        zj = -1.5
        B.lathe(f"{name}_regen", bell_profile(0.15, zt, 0.42, zj, n=8, k=1.6, t=0.02), "nozzle_dark", segs=72,
                closed_loop=True)
        B.lathe(f"{name}_joint_ring", [(0.43, zj + 0.05), (0.48, zj + 0.05), (0.48, zj - 0.05), (0.43, zj - 0.05)],
                "steel", segs=72, closed_loop=True)
        B.lathe(f"{name}_extension", bell_profile(0.42, zj, re, -L, n=30, k=1.5, t=0.012), "niobium", segs=128,
                closed_loop=True)


def stage1_body(st):
    r, s = st["radius"], st["sections"]
    z_es, z_tank_top, z_top = s["engine_section"][1], s["lox_tank"][1], st["length"]
    # engine section (dark) with the octaweb heat shield as its bottom face
    B.lathe("s1_engine_section", [(0.0, 0.0), (r, 0.0), (r, z_es)], "octaweb", segs=96)
    B.lathe("s1_heat_shield", [(0.0, 0.01), (r - 0.04, 0.01)], "heat_shield", segs=96)
    # white tank barrel with circumferential weld/panel lines
    B.lathe("s1_tanks", [(r, z_es), (r, z_tank_top)], "white_paint", segs=128)
    n_lines = 10
    for k in range(n_lines + 1):
        z = z_es + (z_tank_top - z_es) * k / n_lines
        B.lathe(f"s1_line_{k}", [(r - 0.01, z - 0.02), (r + 0.006, z - 0.02), (r + 0.006, z + 0.02),
                                 (r - 0.01, z + 0.02)], "panel_line", segs=128, closed_loop=True)
    # black carbon-composite interstage (hollow tube, open top)
    B.lathe("s1_interstage", [(r - 0.05, z_top), (r - 0.05, z_tank_top), (r, z_tank_top), (r, z_top)],
            "carbon_black", segs=128, closed_loop=True)
    B.lathe("s1_interstage_rim", [(r - 0.06, z_top - 0.08), (r + 0.01, z_top - 0.08), (r + 0.01, z_top),
                                  (r - 0.06, z_top)], "dark_steel", segs=128, closed_loop=True)
    # cable raceway running up the side (between the legs), and leg/fin root fairings
    a = math.radians(45)
    B.box("s1_raceway", (0.16, 0.30, z_top - z_es - 0.6), ((r + 0.07) * math.cos(a), (r + 0.07) * math.sin(a),
                                                           (z_top + z_es) / 2), "panel_line", rot=(0, 0, a))
    Lg, G = st["legs"], st["grid_fins"]
    for k in range(Lg["count"]):
        a = math.radians(Lg["azimuth0_deg"]) + TAU * k / Lg["count"]
        c, sn = math.cos(a), math.sin(a)
        B.box(f"s1_leg_root_{k}", (0.32, 1.0, 1.4), ((r + 0.12) * c, (r + 0.12) * sn, Lg["hinge_z"] - 0.4),
              "carbon_black", rot=(0, 0, a))
        B.box(f"s1_leg_latch_{k}", (0.12, 0.5, 0.3), ((r + 0.05) * c, (r + 0.05) * sn,
                                                      Lg["hinge_z"] + Lg["length"] - 0.2), "dark_steel", rot=(0, 0, a))
    for k in range(G["count"]):
        a = math.radians(G["azimuth0_deg"]) + TAU * k / G["count"]
        c, sn = math.cos(a), math.sin(a)
        B.box(f"s1_fin_actuator_{k}", (0.3, 0.55, 0.5), ((r + 0.08) * c, (r + 0.08) * sn, G["hinge_z"] - 0.25),
              "dark_steel", rot=(0, 0, a))
    # cold-gas thruster pods near the top of the interstage
    for k in range(2):
        a = math.radians(90 + 45) + math.pi * k
        B.box(f"s1_rcs_pod_{k}", (0.25, 0.6, 0.6), ((r + 0.1) * math.cos(a), (r + 0.1) * math.sin(a), z_top - 1.2),
              "dark_steel", rot=(0, 0, a))


def stage2_body(st):
    r, top = st["radius"], st["length"]
    B.lathe("s2_aft_skirt", [(0.0, 0.0), (r, 0.0), (r, 0.35)], "dark_steel", segs=128)
    B.lathe("s2_body", [(r, 0.35), (r, top)], "white_paint", segs=128)
    for z in (0.35, st["sections"]["rp1_tank"][1], st["sections"]["lox_tank"][1], top - 0.05):
        B.lathe(f"s2_line_{z:.2f}", [(r - 0.01, z - 0.02), (r + 0.006, z - 0.02), (r + 0.006, z + 0.02),
                                     (r - 0.01, z + 0.02)], "panel_line", segs=128, closed_loop=True)
    B.lathe("s2_top", [(r, top), (0.0, top)], "dark_steel", segs=96)
    a = math.radians(45)
    B.box("s2_raceway", (0.14, 0.26, top - 0.9), ((r + 0.06) * math.cos(a), (r + 0.06) * math.sin(a), top / 2 + 0.2),
          "panel_line", rot=(0, 0, a))


def grid_fin(G):
    d, w, L = G["depth"], G["width"], G["length"]
    slab = B.box("fin_slab", (d, w, L), (0, 0, L / 2), "titanium")
    # diamond lattice: cut a checkerboard of 45-degree square holes through the slab (x = flow direction)
    pitch, bar, margin = 0.15, 0.022, 0.07
    h = pitch / 2 - bar * 0.71
    side = h * math.sqrt(2)
    cutters = []
    ny, nz = int(w / pitch) + 2, int(L / pitch) + 2
    for i in range(-ny, ny + 1):
        for j in range(0, 2 * nz + 1):
            for off in (0.0, 0.5):
                y, z = (i + off) * pitch, (j + off) * pitch
                if abs(y) + h > w / 2 - margin + h * 0.6 or z - h < margin - h * 0.6 or z + h > L - margin + h * 0.6:
                    continue
                cutters.append(B.box(f"c{i}_{j}_{off}", (d * 3, side, side), (0, y, z), "titanium",
                                     rot=(math.radians(45), 0, 0)))
    cutter = B.join(cutters, "cutters")
    B.boolean(slab, cutter)
    # solid outer frame over the cut edges, root hinge and actuator lug
    fw = margin
    B.box("fin_frame_l", (d + 0.01, fw, L), (0, -w / 2 + fw / 2, L / 2), "titanium")
    B.box("fin_frame_r", (d + 0.01, fw, L), (0, w / 2 - fw / 2, L / 2), "titanium")
    B.box("fin_frame_b", (d + 0.01, w, fw), (0, 0, fw / 2), "titanium")
    B.box("fin_frame_t", (d + 0.01, w, fw), (0, 0, L - fw / 2), "titanium")
    B.cylinder("fin_hinge", 0.07, -0.25, 0.25, "dark_steel", rot=(math.radians(90), 0, 0))
    B.box("fin_lug", (0.18, 0.3, 0.22), (-0.05, 0, 0.0), "dark_steel")


def landing_leg(Lg):
    L = Lg["length"]
    th = math.radians(Lg["deploy_deg"])
    B.frustum("leg_body", 0.25, L - 0.15, 0.42, 0.85, 0.24, 0.50, "carbon_black", xoff0=0.0, xoff1=0.02)
    B.box("leg_spine", (0.04, 0.16, L - 0.8), (0.22, 0, L / 2), "dark_steel")
    B.cylinder("leg_hinge", 0.13, -0.5, 0.5, "dark_steel", rot=(math.radians(90), 0, 0))
    B.cylinder("leg_strut_fitting", 0.09, -0.35, 0.35, "steel", loc=(-0.1, 0, L * 0.55), rot=(math.radians(90), 0, 0))
    # foot pad: flat to the ground when deployed (normal = world z expressed in the deployed leg frame)
    n = (-math.sin(th), 0.0, math.cos(th))
    tilt = math.atan2(n[0], n[2])
    B.cylinder("leg_foot", 0.55, -0.06, 0.06, "dark_steel", loc=(0, 0, L - 0.05), rot=(0, tilt, 0))
    B.cylinder("leg_foot_crush", 0.2, -0.3, 0.0, "steel", loc=(0, 0, L - 0.05), rot=(0, tilt, 0))


def fairing_half(F):
    R, H, hb, hc = F["radius"], F["height"], F["boattail_height"], F["cylinder_height"]
    r0 = 1.83
    Lo = H - hb - hc
    Ro = (R * R + Lo * Lo) / (2 * R)          # tangent ogive radius
    outer = [(r0, 0.0), (R, hb), (R, hb + hc)]
    n = 28
    for i in range(1, n + 1):
        x = Lo * i / n
        r = math.sqrt(max(Ro * Ro - x * x, 0.0)) + R - Ro   # x from ogive base: r(0)=R, r(Lo)=0
        outer.append((max(r, 0.0), hb + hc + x))
    outer[-1] = (0.0, H)
    t = 0.035
    inner = [(max(r - t, 0.0), z - (t if z >= H - 0.01 else 0.0)) for r, z in reversed(outer)]
    B.lathe("fairing_shell", outer + inner, ["white_paint", "carbon_black"], segs=72, a0=-math.pi / 2,
            a1=math.pi / 2, closed_loop=True, cap_ends=True, end_material_index=1, angle=40)
    B.lathe("fairing_base_band", [(r0 - 0.02, 0.0), (r0 + 0.015, 0.0), (r0 + 0.13, 0.18), (r0 + 0.09, 0.18)],
            "panel_line", segs=48, a0=-math.pi / 2, a1=math.pi / 2, closed_loop=True)
    # separation-rail fairing strip along both seams
    for sgn in (1, -1):
        B.box(f"seam_{sgn}", (0.08, 0.1, hc), (0.0, sgn * (R + 0.02), hb + hc / 2), "panel_line")


def mars2020(P):
    """Approximate Mars 2020 launch stack: adapter, 4.5 m aeroshell (70-deg heat shield + backshell),
    cruise stage on top. Visual only; mass properties come from the YAML."""
    B.lathe("m20_adapter", [(0.0, 0.0), (0.95, 0.0), (1.35, 0.8), (0.0, 0.8)], "dark_steel", segs=64)
    B.lathe("m20_heatshield", [(0.0, 0.8), (0.35, 0.82), (2.25, 1.45), (2.25, 1.5), (0.0, 1.5)], "heat_shield",
            segs=96)
    B.lathe("m20_backshell", [(2.25, 1.5), (2.2, 1.55), (1.25, 3.05), (1.05, 3.3), (0.0, 3.3)], "white_paint",
            segs=96)
    B.lathe("m20_cruise", [(0.0, 3.3), (2.0, 3.3), (2.0, 3.75), (0.0, 3.75)], "silver", segs=96)
    for k in range(8):   # solar-cell segments on the cruise-stage disk
        a = 2 * math.pi * k / 8 + math.pi / 8
        B.box(f"m20_cells_{k}", (1.0, 0.7, 0.02), (1.3 * math.cos(a), 1.3 * math.sin(a), 3.76), "solar_cell",
              rot=(0, 0, a))


def payload(P):
    if P.get("type") == "mars2020":
        return mars2020(P)
    sx, sy, sz = P["size"]
    B.lathe("pl_adapter", [(0.0, 0.0), (0.95, 0.0), (0.62, 0.55), (0.0, 0.55)], "silver", segs=64)
    B.box("pl_bus", (sx * 0.8, sy * 0.8, sz * 0.62), (0, 0, 0.55 + sz * 0.31), "gold_foil")
    for sgn in (1, -1):
        B.box(f"pl_array_{sgn}", (sx * 0.72, 0.08, sz * 0.55), (0, sgn * (sy * 0.4 + 0.06), 0.55 + sz * 0.33),
              "solar_cell")
    B.lathe("pl_dish", [(0.0, sz * 0.62 + 0.55), (0.75, sz * 0.62 + 0.75), (0.72, sz * 0.62 + 0.78),
                        (0.0, sz * 0.62 + 0.6)], "white_paint", segs=48, closed_loop=True)
    B.cylinder("pl_feed", 0.05, sz * 0.62 + 0.55, sz * 0.62 + 1.1, "silver")


def main():
    argv = sys.argv[sys.argv.index("--") + 1:]
    spec = yaml.safe_load(Path(argv[0]).read_text())
    out = Path(argv[1])
    out.mkdir(parents=True, exist_ok=True)
    B.clear_scene()
    s1, s2 = spec["stages"][0], spec["stages"][1]
    for model_name in {st["engines"]["model"] for st in spec["stages"]}:
        merlin(spec["engine_models"][model_name], model_name)
        B.export_glb(out / f"{model_name}.glb", model_name)
    stage1_body(s1); B.export_glb(out / "s1_body.glb", "s1_body")
    stage2_body(s2); B.export_glb(out / "s2_body.glb", "s2_body")
    grid_fin(s1["grid_fins"]); B.export_glb(out / "grid_fin.glb", "grid_fin")
    landing_leg(s1["legs"]); B.export_glb(out / "landing_leg.glb", "landing_leg")
    fairing_half(spec["fairing"]); B.export_glb(out / "fairing_half.glb", "fairing_half")
    payload(spec["payload"]); B.export_glb(out / "payload.glb", "payload")


main()
