#!/usr/bin/env python3
"""
Re-import an exported FBX/GLB into a clean Blender scene, print object sizes/materials and
render a glossy three-quarter view (light grey material) to expose shading artifacts.

    python3 scripts/verify_fbx.py models/iphone_11_pro_max/iphone_11_pro_max.fbx [--out /tmp/verify.png]
"""
import argparse
import math
import os
import sys

import bpy
from mathutils import Vector, Matrix


def look_at(cam, target, distance, direction, up=(0, 1, 0)):
    d = Vector(direction).normalized()
    cam.location = Vector(target) + d * distance
    z = d.copy()
    x = Vector(up).cross(z).normalized()
    y = z.cross(x).normalized()
    cam.rotation_euler = Matrix((x, y, z)).transposed().to_euler()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--out", default="/tmp/verify.png")
    ap.add_argument("--samples", type=int, default=32)
    args = ap.parse_args()

    bpy.ops.wm.read_factory_settings(use_empty=True)
    if args.path.lower().endswith(".fbx"):
        bpy.ops.import_scene.fbx(filepath=args.path)
    else:
        bpy.ops.import_scene.gltf(filepath=args.path)

    meshes = [o for o in bpy.data.objects if o.type == "MESH"]
    lo = Vector((1e9,) * 3)
    hi = Vector((-1e9,) * 3)
    for ob in meshes:
        d = ob.dimensions
        print("%-18s %6.1f x %6.1f x %5.2f mm  mats=%s  custom_normals=%s" % (
            ob.name, d.x * 1000, d.y * 1000, d.z * 1000, [m.name for m in ob.data.materials],
            "custom_normal" in ob.data.attributes or ob.data.has_custom_normals))
        for c in ob.bound_box:
            w = ob.matrix_world @ Vector(c)
            lo = Vector(map(min, lo, w))
            hi = Vector(map(max, hi, w))
    center = (lo + hi) / 2
    size = max(hi - lo)

    gloss = bpy.data.materials.new("VerifyGloss")
    gloss.use_nodes = True
    b = gloss.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (0.5, 0.5, 0.5, 1)
    b.inputs["Roughness"].default_value = 0.2
    for ob in meshes:
        if ob.name.startswith("Cell"):
            for i in range(len(ob.data.materials)):
                ob.data.materials[i] = gloss

    sc = bpy.context.scene
    sc.world = bpy.data.worlds.new("W")
    sc.world.use_nodes = True
    sc.world.node_tree.nodes["Background"].inputs[1].default_value = 0.3
    for name, loc, rot, sz, e in (
        ("Key", (-0.35, 0.25, 0.65), (28, -28, 0), 1.0, 10.0),
        ("Fill", (0.45, -0.15, 0.5), (20, 40, 0), 1.2, 4.0),
    ):
        ld = bpy.data.lights.new(name, "AREA")
        ld.energy, ld.size = e, sz
        lo_ = bpy.data.objects.new(name, ld)
        lo_.location = center + Vector(loc)
        lo_.rotation_euler = [math.radians(a) for a in rot]
        sc.collection.objects.link(lo_)

    cd = bpy.data.cameras.new("Cam")
    cd.lens = 70
    cd.clip_start = 0.001
    cam = bpy.data.objects.new("Cam", cd)
    sc.collection.objects.link(cam)
    sc.camera = cam
    look_at(cam, center, size * 2.6, (0.9, -0.9, 1.1))

    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = args.samples
    sc.cycles.use_denoising = True
    sc.render.resolution_x, sc.render.resolution_y = 900, 700
    sc.render.filepath = os.path.abspath(args.out)
    bpy.ops.render.render(write_still=True)
    print("rendered ->", sc.render.filepath)


if __name__ == "__main__":
    if "--" in sys.argv:
        sys.argv = [sys.argv[0]] + sys.argv[sys.argv.index("--") + 1:]
    main()
