"""Small helper layer over bpy for scripted (headless) mesh generation.

Every builder creates plain mesh objects in the current scene; export_glb() joins
them into one object (multi-material) and writes a .glb, then clears the scene.
"""
import math

import bmesh
import bpy
from mathutils import Euler, Matrix, Vector

_MATS = {}

# name: (base colour RGB, metallic, roughness)
# Metallic values are deliberately moderate: Gazebo's default ogre2 scene has no
# environment map, so fully metallic PBR surfaces have nothing to reflect and render black.
PALETTE = {
    "white_paint":   ((0.90, 0.91, 0.92), 0.0, 0.32),
    "panel_line":    ((0.62, 0.63, 0.65), 0.0, 0.45),
    "carbon_black":  ((0.025, 0.025, 0.028), 0.05, 0.30),
    "octaweb":       ((0.18, 0.18, 0.19), 0.3, 0.55),
    "heat_shield":   ((0.07, 0.065, 0.06), 0.0, 0.85),
    "nozzle_dark":   ((0.20, 0.18, 0.17), 0.4, 0.38),
    "copper":        ((0.85, 0.50, 0.30), 0.5, 0.30),
    "steel":         ((0.70, 0.71, 0.73), 0.4, 0.32),
    "dark_steel":    ((0.30, 0.30, 0.32), 0.35, 0.45),
    "titanium":      ((0.60, 0.58, 0.55), 0.4, 0.40),
    "niobium":       ((0.30, 0.27, 0.24), 0.45, 0.30),
    "gold_foil":     ((1.00, 0.72, 0.25), 0.5, 0.28),
    "solar_cell":    ((0.04, 0.08, 0.25), 0.2, 0.18),
    "silver":        ((0.85, 0.85, 0.87), 0.45, 0.22),
    "concrete":      ((0.58, 0.57, 0.54), 0.0, 0.92),
    "concrete_dark": ((0.30, 0.29, 0.28), 0.0, 0.95),
    "tower_steel":   ((0.33, 0.34, 0.36), 0.3, 0.50),
    "aviation_red":  ((0.75, 0.06, 0.04), 0.0, 0.45),
    "tank_white":    ((0.86, 0.87, 0.88), 0.05, 0.40),
    "asphalt":       ((0.08, 0.08, 0.085), 0.0, 0.9),
    "yellow_paint":  ((0.95, 0.70, 0.05), 0.0, 0.5),
    "ocean":         ((0.05, 0.18, 0.30), 0.0, 0.15),
}


def mat(name):
    if name in _MATS and _MATS[name].name in bpy.data.materials:
        return _MATS[name]
    col, metal, rough = PALETTE[name]
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*col, 1.0)
    b.inputs["Metallic"].default_value = metal
    b.inputs["Roughness"].default_value = rough
    m.diffuse_color = (*col, 1.0)
    _MATS[name] = m
    return m


def _finish(name, bm, material, smooth=True, angle=35.0):
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-6)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    if isinstance(material, (list, tuple)):
        for m in material:
            me.materials.append(mat(m))
    else:
        me.materials.append(mat(material))
    for p in me.polygons:
        p.use_smooth = smooth
    if smooth:
        me.use_auto_smooth = True
        me.auto_smooth_angle = math.radians(angle)
    return ob


def lathe(name, profile, material, segs=96, a0=0.0, a1=2 * math.pi, closed_loop=False,
          cap_ends=False, end_material_index=0, smooth=True, angle=35.0):
    """Revolve an (r, z) polyline about z. closed_loop: profile is a closed outline (solid ring).
    For partial revolutions, cap_ends fills the two cut planes (needs closed_loop)."""
    prof = list(profile) + ([profile[0]] if closed_loop else [])
    full = abs((a1 - a0) - 2 * math.pi) < 1e-9
    n_ang = segs if full else segs + 1
    bm = bmesh.new()
    rings = []
    for i in range(n_ang):
        a = a0 + (a1 - a0) * i / segs
        c, s = math.cos(a), math.sin(a)
        rings.append([bm.verts.new((r * c, r * s, z)) for r, z in prof])
    P = len(prof)
    for i in range(segs):
        i2 = (i + 1) % n_ang if full else i + 1
        for j in range(P - 1):
            vs = [rings[i][j], rings[i2][j], rings[i2][j + 1], rings[i][j + 1]]
            if len({v.co.to_tuple(7) for v in vs}) >= 3:
                try:
                    bm.faces.new(vs)
                except ValueError:
                    pass
    if cap_ends and not full:
        for ring, flip in ((rings[0], True), (rings[-1], False)):
            vs = ring[:-1]
            f = bm.faces.new(list(reversed(vs)) if flip else vs)
            f.material_index = end_material_index
    return _finish(name, bm, material, smooth, angle)


def box(name, size, loc, material, rot=(0.0, 0.0, 0.0), smooth=False):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    M = Matrix.Translation(Vector(loc)) @ Euler(rot).to_matrix().to_4x4() @ Matrix.Diagonal((*size, 1.0))
    bmesh.ops.transform(bm, matrix=M, verts=bm.verts)
    return _finish(name, bm, material, smooth)


def frustum(name, z0, z1, sx0, sy0, sx1, sy1, material, xoff0=0.0, xoff1=0.0):
    """Rectangular frustum along z (tapered box), cross-section centred at x offset."""
    bm = bmesh.new()
    lo = [bm.verts.new((xoff0 + x * sx0 / 2, y * sy0 / 2, z0)) for x, y in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    hi = [bm.verts.new((xoff1 + x * sx1 / 2, y * sy1 / 2, z1)) for x, y in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    bm.faces.new(list(reversed(lo)))
    bm.faces.new(hi)
    for k in range(4):
        bm.faces.new([lo[k], lo[(k + 1) % 4], hi[(k + 1) % 4], hi[k]])
    return _finish(name, bm, material, smooth=False)


def cylinder(name, r, z0, z1, material, segs=48, loc=(0, 0, 0), rot=(0, 0, 0), r_top=None):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=segs, radius1=r,
                          radius2=r if r_top is None else r_top, depth=z1 - z0)
    M = Matrix.Translation(Vector(loc)) @ Euler(rot).to_matrix().to_4x4() @ Matrix.Translation((0, 0, (z0 + z1) / 2))
    bmesh.ops.transform(bm, matrix=M, verts=bm.verts)
    return _finish(name, bm, material, smooth=True, angle=30)


def strut(name, p0, p1, w, material):
    """Square-section member from p0 to p1 (lattice towers)."""
    p0, p1 = Vector(p0), Vector(p1)
    d = p1 - p0
    q = Vector((0, 0, 1)).rotation_difference(d.normalized())
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    M = Matrix.Translation((p0 + p1) / 2) @ q.to_matrix().to_4x4() @ Matrix.Diagonal((w, w, d.length, 1.0))
    bmesh.ops.transform(bm, matrix=M, verts=bm.verts)
    return _finish(name, bm, material, smooth=False)


def sphere(name, r, loc, material, segs=48):
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=segs, v_segments=segs // 2, radius=r)
    bmesh.ops.translate(bm, vec=Vector(loc), verts=bm.verts)
    return _finish(name, bm, material, smooth=True, angle=60)


def boolean(target, cutter, op="DIFFERENCE"):
    mod = target.modifiers.new("bool", "BOOLEAN")
    mod.operation = op
    mod.solver = "EXACT"
    mod.object = cutter
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.modifier_apply(modifier=mod.name)
    bpy.data.objects.remove(cutter, do_unlink=True)


def join(objs, name):
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    if len(objs) > 1:
        bpy.ops.object.join()
    ob = bpy.context.view_layer.objects.active
    ob.name = name
    return ob


def clear_scene():
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    for me in list(bpy.data.meshes):
        if me.users == 0:
            bpy.data.meshes.remove(me)


def export_glb(path, name):
    objs = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    ob = join(objs, name)
    bpy.ops.object.select_all(action="DESELECT")
    ob.select_set(True)
    # export_yup=False: keep Blender's Z-up coordinates. Neither Gazebo (gz-common assimp)
    # nor RViz converts glTF's Y-up back to ROS Z-up, so converted meshes lie on their side.
    bpy.ops.export_scene.gltf(filepath=str(path), export_format="GLB", use_selection=True,
                              export_apply=True, export_yup=False, export_materials="EXPORT")
    n_tris = sum(len(p.vertices) - 2 for p in ob.data.polygons)
    print(f"[export] {path}  tris={n_tris}  materials={[m.name for m in ob.data.materials]}")
    clear_scene()
