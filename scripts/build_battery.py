#!/usr/bin/env python3
"""
Parametric iPhone battery generator (Blender / bpy).

    python3 scripts/build_battery.py specs/iphone_11_pro_max.json [--out models] [--render] [--samples 64]

Reads a JSON spec (all sizes in mm, front view, origin = bottom-left of the
bounding box, +Z = towards the viewer of the label side) and produces:

    models/<id>/<id>.blend      Blender scene (mm units, 1 BU = 1 m)
    models/<id>/<id>.fbx        FBX (cm units, Y-up, as Blender exports by default)
    models/<id>/<id>.glb        glTF binary (metres)
    models/<id>/uv_template_*.png   UV layout templates for texturing
    models/<id>/preview_*.png   Cycles preview renders (only with --render)

Objects / materials
    Cell            Cell_Front | Cell_Back | Cell_Side
    Flex_*          Flex            planar UV = bounding box of the flex polygon
    Stiffener_*     Stiffener
    Connector_*     Connector_Plastic | Connector_Metal | Connector_Gold

Cell_Front UV  : u = x / width,       v = y / height   (texture aspect = width : height)
Cell_Back  UV  : u = 1 - x / width,   v = y / height   (mirrored, so the back texture is painted as seen from the back)
Cell_Side  UV  : u = perimeter position (0..1, CCW from the bottom-left corner), v = thickness (0..1)
"""
import argparse
import json
import math
import os
import sys

import bpy
import bmesh
from mathutils import Vector, Matrix

MM = 0.001  # 1 mm in Blender units (scene is in metres)


# --------------------------------------------------------------------------- 2D helpers
def fillet_polygon(pts, radii, seg_per_90=8):
    """Round the corners of a closed polygon. radii[i] = 0 keeps the corner sharp."""
    out = []
    n = len(pts)
    for i in range(n):
        p = Vector(pts[i])
        r = radii[i] if i < len(radii) else 0.0
        if r <= 0:
            out.append((p.x, p.y))
            continue
        a = Vector(pts[i - 1])
        b = Vector(pts[(i + 1) % n])
        d1 = (a - p).normalized()
        d2 = (b - p).normalized()
        ang = math.acos(max(-1.0, min(1.0, d1.dot(d2))))  # interior angle
        t = r / math.tan(ang / 2.0)
        t = min(t, (a - p).length * 0.499, (b - p).length * 0.499)
        r_eff = t * math.tan(ang / 2.0)
        p1 = p + d1 * t
        p2 = p + d2 * t
        bis = (d1 + d2).normalized()
        c = p + bis * (r_eff / math.sin(ang / 2.0))
        a1 = math.atan2(p1.y - c.y, p1.x - c.x)
        a2 = math.atan2(p2.y - c.y, p2.x - c.x)
        da = a2 - a1
        while da > math.pi:
            da -= 2 * math.pi
        while da < -math.pi:
            da += 2 * math.pi
        segs = max(2, int(round(abs(da) / (math.pi / 2) * seg_per_90)))
        for k in range(segs + 1):
            aa = a1 + da * k / segs
            out.append((c.x + r_eff * math.cos(aa), c.y + r_eff * math.sin(aa)))
    return out


def l_outline(o):
    """L-shaped cell outline (CCW), front view. Narrow upper part on the left."""
    W, H, uw, lh = o["width"], o["height"], o["upper_width"], o["lower_height"]
    ro, ri = o.get("outer_radius", 1.5), o.get("inner_radius", 3.0)
    pts = [(0, 0), (W, 0), (W, lh), (uw, lh), (uw, H), (0, H)]
    radii = [ro, ro, ro, ri, ro, ro]
    return fillet_polygon(pts, radii)


def rect_outline(o):
    W, H = o["width"], o["height"]
    ro = o.get("outer_radius", 1.5)
    pts = [(0, 0), (W, 0), (W, H), (0, H)]
    return fillet_polygon(pts, [ro] * 4)


def outline_from_spec(o):
    if o["type"] == "L":
        return l_outline(o)
    if o["type"] == "L_mirror":  # narrow upper part on the right
        pts = l_outline(o)
        W = o["width"]
        return [(W - x, y) for x, y in reversed(pts)]
    if o["type"] == "rect":
        return rect_outline(o)
    if o["type"] == "polygon":
        return fillet_polygon(o["points"], o.get("fillets", [0] * len(o["points"])))
    raise ValueError("unknown outline type %r" % o["type"])


def poly_bbox(poly):
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    return min(xs), min(ys), max(xs), max(ys)


def poly_area(poly):
    a = 0.0
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        a += x1 * y2 - x2 * y1
    return a / 2.0


class PerimeterParam:
    """Cumulative arc-length position (0..1) of a 2D point along a closed polygon."""

    def __init__(self, poly):
        self.poly = [Vector(p) for p in poly]
        self.cum = [0.0]
        for i in range(len(poly)):
            a, b = self.poly[i], self.poly[(i + 1) % len(poly)]
            self.cum.append(self.cum[-1] + (b - a).length)
        self.total = self.cum[-1]

    def __call__(self, x, y):
        p = Vector((x, y))
        best, best_s = 1e30, 0.0
        n = len(self.poly)
        for i in range(n):
            a, b = self.poly[i], self.poly[(i + 1) % n]
            ab = b - a
            L2 = ab.length_squared
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, (p - a).dot(ab) / L2))
            q = a + ab * t
            d = (p - q).length_squared
            if d < best:
                best, best_s = d, self.cum[i] + ab.length * t
        return best_s / self.total


# --------------------------------------------------------------------------- mesh helpers
def new_object(name, bm, materials, scale=MM):
    """Finish a bmesh (in mm) into a scene object (in metres)."""
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-4)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    for v in bm.verts:
        v.co *= scale
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    for m in materials:
        me.materials.append(m)
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    return ob


def set_area_weighted_normals(me):
    """Smooth shading with face-area-weighted vertex normals (same idea as the Weighted Normal
    modifier): the big flat faces dominate, so their boundary normals stay (0,0,+-1) and the
    n-gon interior shades evenly; the bevel still blends smoothly into the sides."""
    me.polygons.foreach_set("use_smooth", [True] * len(me.polygons))
    acc = [Vector((0.0, 0.0, 0.0)) for _ in me.vertices]
    for p in me.polygons:
        n = p.normal * p.area
        for vi in p.vertices:
            acc[vi] += n
    normals = [v.normalized() if v.length > 0 else Vector((0, 0, 1)) for v in acc]
    me.normals_split_custom_set_from_vertices(normals)
    me.update()


def mark_sharp_edges(me, angle_deg=35.0):
    """Smooth faces + sharp edges where the dihedral angle is large (works without auto-smooth)."""
    bm = bmesh.new()
    bm.from_mesh(me)
    thr = math.radians(angle_deg)
    for e in bm.edges:
        if len(e.link_faces) == 2:
            e.smooth = e.calc_face_angle(0.0) < thr
        else:
            e.smooth = False
    for f in bm.faces:
        f.smooth = True
    bm.to_mesh(me)
    bm.free()


def prism(bm, poly, z0, z1):
    """Extruded closed polygon, returns (bottom_face, top_face, side_faces)."""
    n = len(poly)
    vb = [bm.verts.new((x, y, z0)) for x, y in poly]
    vt = [bm.verts.new((x, y, z1)) for x, y in poly]
    fb = bm.faces.new(list(reversed(vb)))
    ft = bm.faces.new(vt)
    sides = [bm.faces.new([vb[i], vb[(i + 1) % n], vt[(i + 1) % n], vt[i]]) for i in range(n)]
    return fb, ft, sides


def box(bm, x0, y0, z0, x1, y1, z1):
    poly = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return prism(bm, poly, z0, z1)


def set_material(faces, idx):
    for f in faces:
        f.material_index = idx


def planar_uv(bm, uv_layer, faces, bbox, mirror_x=False):
    x0, y0, x1, y1 = bbox
    w, h = (x1 - x0) or 1.0, (y1 - y0) or 1.0
    for f in faces:
        for l in f.loops:
            u = (l.vert.co.x - x0) / w
            v = (l.vert.co.y - y0) / h
            if mirror_x:
                u = 1.0 - u
            l[uv_layer].uv = (u, v)


def box_uv(bm, uv_layer, faces, bbox3):
    """Simple cube projection normalised to a 3D bbox (good enough for small hardware)."""
    x0, y0, z0, x1, y1, z1 = bbox3
    sx, sy, sz = (x1 - x0) or 1, (y1 - y0) or 1, (z1 - z0) or 1
    for f in faces:
        n = f.normal
        ax, ay, az = abs(n.x), abs(n.y), abs(n.z)
        for l in f.loops:
            c = l.vert.co
            if az >= ax and az >= ay:
                uv = ((c.x - x0) / sx, (c.y - y0) / sy)
            elif ax >= ay:
                uv = ((c.y - y0) / sy, (c.z - z0) / sz)
            else:
                uv = ((c.x - x0) / sx, (c.z - z0) / sz)
            l[uv_layer].uv = uv


# --------------------------------------------------------------------------- materials
def make_material(name, color, roughness=0.5, metallic=0.0, specular=0.5, sheen=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["Metallic"].default_value = metallic
    if "Specular IOR Level" in bsdf.inputs:
        bsdf.inputs["Specular IOR Level"].default_value = specular
    if sheen and "Sheen Weight" in bsdf.inputs:
        bsdf.inputs["Sheen Weight"].default_value = sheen
    return m


def materials():
    return {
        "Cell_Front": make_material("Cell_Front", (0.012, 0.012, 0.013), 0.42, sheen=0.1),
        "Cell_Back": make_material("Cell_Back", (0.012, 0.012, 0.013), 0.42, sheen=0.1),
        "Cell_Side": make_material("Cell_Side", (0.010, 0.010, 0.011), 0.55),
        "Cell_Seal": make_material("Cell_Seal", (0.014, 0.014, 0.015), 0.6),
        "Flex": make_material("Flex", (0.020, 0.012, 0.008), 0.38),
        "Stiffener": make_material("Stiffener", (0.008, 0.008, 0.008), 0.6),
        "Connector_Plastic": make_material("Connector_Plastic", (0.015, 0.014, 0.013), 0.65),
        "Connector_Metal": make_material("Connector_Metal", (0.78, 0.78, 0.76), 0.32, metallic=1.0),
        "Connector_Gold": make_material("Connector_Gold", (1.0, 0.72, 0.30), 0.25, metallic=1.0),
    }


# --------------------------------------------------------------------------- builders
def build_cell(spec, mats):
    poly = outline_from_spec(spec["outline"])
    if poly_area(poly) < 0:
        poly.reverse()
    T = spec["thickness"]
    bb = poly_bbox(poly)

    bm = bmesh.new()
    prism(bm, poly, 0.0, T)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)

    bev = spec.get("edge_bevel")
    if bev and bev.get("offset", 0) > 0:
        edges = [e for e in bm.edges
                 if all(abs(v.co.z) < 1e-6 for v in e.verts) or all(abs(v.co.z - T) < 1e-6 for v in e.verts)]
        bmesh.ops.bevel(bm, geom=edges, offset=bev["offset"], segments=bev.get("segments", 4),
                        profile=bev.get("profile", 0.5), affect="EDGES", clamp_overlap=True)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)

    uv = bm.loops.layers.uv.new("UVMap")
    front, back, side = [], [], []
    for f in bm.faces:
        nz = f.normal.z
        # the whole bevel belongs to front/back (planar texture runs over the edge),
        # only the vertical band is Cell_Side -> no material seam on the slope
        if nz > 0.02:
            front.append(f)
        elif nz < -0.02:
            back.append(f)
        else:
            side.append(f)
    set_material(front, 0)
    set_material(back, 1)
    set_material(side, 2)
    planar_uv(bm, uv, front, bb)
    planar_uv(bm, uv, back, bb, mirror_x=True)

    pp = PerimeterParam(poly)
    for f in side:
        us = []
        for l in f.loops:
            us.append(pp(l.vert.co.x, l.vert.co.y))
        if max(us) - min(us) > 0.5:  # seam wrap
            us = [u + 1.0 if u < 0.5 else u for u in us]
        for l, u in zip(f.loops, us):
            l[uv].uv = (u, l.vert.co.z / T)

    ob = new_object("Cell", bm, [mats["Cell_Front"], mats["Cell_Back"], mats["Cell_Side"]])
    set_area_weighted_normals(ob.data)
    return ob, poly, bb


def build_seal_lips(spec, mats):
    """Flat laminate seal lips of the pouch (where the cell tabs exit); the flexes sit on top of them."""
    objs = []
    t = spec.get("seal_thickness", 0.3)
    zc = spec["thickness"] / 2.0
    for lip in spec.get("seal_lips", []):
        x0, y0, x1, y1 = lip["rect"]
        poly = fillet_polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], lip.get("fillets", [0.3] * 4))
        bm = bmesh.new()
        prism(bm, poly, zc - t / 2.0, zc + t / 2.0)
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        uv = bm.loops.layers.uv.new("UVMap")
        planar_uv(bm, uv, list(bm.faces), (x0, y0, x1, y1))
        ob = new_object(lip["name"], bm, [mats["Cell_Seal"]])
        mark_sharp_edges(ob.data, 35.0)
        objs.append(ob)
    return objs


def build_side_wraps(spec, mats):
    """Taped side-seal folds: rounded dark ribs hugging an outer edge, slightly thicker than the cell."""
    objs = []
    T = spec["thickness"]
    for w in spec.get("side_wraps", []):
        x0, y0, x1, y1 = w["rect"]
        ex = w.get("extra_thickness", 0.25)
        poly = fillet_polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], w.get("fillets", [1.0] * 4))
        bm = bmesh.new()
        prism(bm, poly, -ex / 2.0, T + ex / 2.0)
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        bev = w.get("bevel", {"offset": 1.2, "segments": 4})
        edges = [e for e in bm.edges
                 if all(abs(v.co.z + ex / 2.0) < 1e-6 for v in e.verts) or all(abs(v.co.z - T - ex / 2.0) < 1e-6 for v in e.verts)]
        bmesh.ops.bevel(bm, geom=edges, offset=bev["offset"], segments=bev.get("segments", 4),
                        profile=0.5, affect="EDGES", clamp_overlap=True)
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        uv = bm.loops.layers.uv.new("UVMap")
        planar_uv(bm, uv, list(bm.faces), (x0, y0, x1, y1))
        ob = new_object(w["name"], bm, [mats["Cell_Seal"]])
        set_area_weighted_normals(ob.data)
        objs.append(ob)
    return objs


def fillet_polyline(pts, radii, seg_per_90=8):
    """Round the interior corners of an OPEN polyline (first/last points are kept)."""
    if len(pts) < 3:
        return [tuple(p) for p in pts]
    closed = fillet_polygon(list(pts), [0] + list(radii[1:-1]) + [0], seg_per_90)
    # fillet_polygon treats the list as closed; the first and last points have radius 0 so the
    # only spurious effect is the closing edge, which does not add points. Return as is.
    return closed


def ribbon_polygon(path, radii, width, seg_per_90=8):
    """Closed outline (CCW) of a flat strip of `width` following an open centreline."""
    c = fillet_polyline(path, radii, seg_per_90)
    pts = [Vector(p) for p in c]
    n = len(pts)
    left, right = [], []
    for i in range(n):
        if i == 0:
            t = (pts[1] - pts[0]).normalized()
        elif i == n - 1:
            t = (pts[-1] - pts[-2]).normalized()
        else:
            t1 = (pts[i] - pts[i - 1]).normalized()
            t2 = (pts[i + 1] - pts[i]).normalized()
            t = (t1 + t2)
            t = t.normalized() if t.length > 1e-6 else t1
            # widen at the corner so the strip keeps its width (miter)
        nrm = Vector((-t.y, t.x))
        k = 1.0
        if 0 < i < n - 1:
            cos_half = max(0.2, abs(nrm.dot(Vector((-t1.y, t1.x)))))
            k = 1.0 / cos_half
        left.append(pts[i] + nrm * (width / 2.0) * k)
        right.append(pts[i] - nrm * (width / 2.0) * k)
    poly = [(p.x, p.y) for p in left] + [(p.x, p.y) for p in reversed(right)]
    if poly_area(poly) < 0:
        poly.reverse()
    return poly


def rounded_rect(cx, cy, w, h, r, rot_deg=0.0):
    pts = [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)]
    poly = fillet_polygon(pts, [min(r, w / 2 - 0.01, h / 2 - 0.01)] * 4)
    a = math.radians(rot_deg)
    ca, sa = math.cos(a), math.sin(a)
    return [(cx + x * ca - y * sa, cy + x * sa + y * ca) for x, y in poly]


def build_flex(spec, fx, mats):
    if "path" in fx:
        poly = ribbon_polygon(fx["path"], fx.get("radii", [0] * len(fx["path"])), fx.get("width", 1.2))
    else:
        poly = fillet_polygon(fx["polygon"], fx.get("fillets", [0] * len(fx["polygon"])))
    if poly_area(poly) < 0:
        poly.reverse()
    t = spec.get("flex_thickness", 0.2)
    zc = spec["thickness"] / 2.0 if spec.get("flex_z", "mid") == "mid" else float(spec["flex_z"])
    z0, z1 = zc - t / 2.0, zc + t / 2.0
    bb = poly_bbox(poly)

    bm = bmesh.new()
    prism(bm, poly, z0, z1)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    uv = bm.loops.layers.uv.new("UVMap")
    planar_uv(bm, uv, list(bm.faces), bb)
    ob = new_object(fx["name"], bm, [mats["Flex"]])
    mark_sharp_edges(ob.data, 35.0)

    objs = [ob]
    # connector pad: polyimide tab at the end of the strip, slightly larger than the plug
    pad = fx.get("pad")
    if pad:
        pcx, pcy = pad["center"]
        pw, ph = pad["size"]
        ppoly = rounded_rect(pcx, pcy, pw, ph, pad.get("radius", 0.5), pad.get("rotation_deg", 0))
        bm = bmesh.new()
        prism(bm, ppoly, z0, z1 + 0.05)
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        uv = bm.loops.layers.uv.new("UVMap")
        planar_uv(bm, uv, list(bm.faces), poly_bbox(ppoly))
        po = new_object(fx["name"].replace("Flex", "Pad"), bm, [mats["Stiffener"]])
        mark_sharp_edges(po.data, 35.0)
        objs.append(po)
        z1 = z1 + 0.05
    # stiffener plate on the side opposite to the connector (legacy, optional)
    st = fx.get("stiffener")
    if st:
        sx0, sy0, sx1, sy1 = st
        ts = spec.get("stiffener_thickness", 0.3)
        if spec.get("connector_side", "back") == "back":
            sz0, sz1 = z1, z1 + ts
        else:
            sz0, sz1 = z0 - ts, z0
        bm = bmesh.new()
        box(bm, sx0 + 0.15, sy0 + 0.15, sz0, sx1 - 0.15, sy1 - 0.15, sz1)
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        uv = bm.loops.layers.uv.new("UVMap")
        box_uv(bm, uv, list(bm.faces), (sx0, sy0, sz0, sx1, sy1, sz1))
        so = new_object(fx["name"].replace("Flex", "Stiffener"), bm, [mats["Stiffener"]])
        mark_sharp_edges(so.data, 35.0)
        objs.append(so)

    con = fx.get("connector")
    if con:
        objs.append(build_connector(spec, con, z0, z1, mats))
    return objs


def build_connector(spec, con, flex_z0, flex_z1, mats):
    """Board-to-board plug on the flex: plastic housing, metal shell, gold contacts."""
    L, W, H = con["size"]
    cx, cy = con["center"]
    side_back = spec.get("connector_side", "back") == "back"
    # local frame: connector centred at origin, mating face at +z, then flipped/moved into place
    bm = bmesh.new()
    plastic, metal, gold = [], [], []

    floor_h = 0.18
    wall = 0.45
    skirt = 0.06
    # floor (plastic)
    fb, ft, fs = box(bm, -L / 2, -W / 2, 0, L / 2, W / 2, floor_h)
    plastic += [fb, ft] + fs
    # rectangular ring of walls around the cavity; top rim = metal shield
    for sy in (-1, 1):
        y0, y1 = (W / 2 - wall, W / 2) if sy > 0 else (-W / 2, -(W / 2 - wall))
        fb, ft, fs = box(bm, -L / 2, y0, floor_h, L / 2, y1, H)
        plastic += [fb] + fs
        metal.append(ft)
    for sx in (-1, 1):
        x0, x1 = (L / 2 - wall, L / 2) if sx > 0 else (-L / 2, -(L / 2 - wall))
        fb, ft, fs = box(bm, x0, -W / 2 + wall, floor_h, x1, W / 2 - wall, H)
        plastic += [fb] + fs
        metal.append(ft)
    # metal shield skirt: four thin plates around the outside, upper part of the housing
    band_z0 = H - 0.26
    for sy in (-1, 1):
        y0, y1 = (W / 2, W / 2 + skirt) if sy > 0 else (-W / 2 - skirt, -W / 2)
        fb, ft, fs = box(bm, -L / 2 - skirt, y0, band_z0, L / 2 + skirt, y1, H)
        metal += [fb, ft] + fs
    for sx in (-1, 1):
        x0, x1 = (L / 2, L / 2 + skirt) if sx > 0 else (-L / 2 - skirt, -L / 2)
        fb, ft, fs = box(bm, x0, -W / 2, band_z0, x1, W / 2, H)
        metal += [fb, ft] + fs
    # contacts: two rows of L-shaped gold fingers on the inner faces of the long walls
    n = int(con.get("pins_per_row", 4))
    pitch = con.get("pitch", 0.5)
    pw = 0.22           # finger width
    depth = 0.10        # how far the finger stands proud of the wall
    foot = 0.30         # foot length on the floor
    top_z = H - 0.07    # fingers stay below the rim
    xs = [(i - (n - 1) / 2.0) * pitch for i in range(n)]
    for sy in (-1, 1):
        wall_in = sy * (W / 2 - wall)              # inner face of the wall
        for x in xs:
            ya, yb = sorted([wall_in - sy * depth, wall_in + sy * 0.05])
            fb, ft, fs = box(bm, x - pw / 2, ya, floor_h - 0.01, x + pw / 2, yb, top_z)
            gold += [fb, ft] + fs
            fa, fb2 = sorted([wall_in - sy * depth, wall_in - sy * (depth + foot)])
            fb, ft, fs = box(bm, x - pw / 2, fa, floor_h - 0.01, x + pw / 2, fb2, floor_h + 0.06)
            gold += [fb, ft] + fs
    if con.get("power_pads", True):
        # wide power contacts on the inner faces of the end walls
        pad_w = W - 2 * wall - 0.3
        for sx in (-1, 1):
            wall_in = sx * (L / 2 - wall)
            xa, xb = sorted([wall_in - sx * depth, wall_in + sx * 0.05])
            fb, ft, fs = box(bm, xa, -pad_w / 2, floor_h - 0.01, xb, pad_w / 2, top_z)
            gold += [fb, ft] + fs
            fa, fb2 = sorted([wall_in - sx * depth, wall_in - sx * (depth + foot)])
            fb, ft, fs = box(bm, fa, -pad_w / 2, floor_h - 0.01, fb2, pad_w / 2, floor_h + 0.06)
            gold += [fb, ft] + fs

    if con.get("cap", True):
        # metal shield plate on the far side of the flex pad (what you see from the label side
        # when the plug faces the board): sits on top of the pad, so offset by the pad thickness
        cap_t = 0.14
        off = flex_z1 - flex_z0
        cpoly = rounded_rect(0, 0, L + 2 * skirt, W + 2 * skirt, 0.35)
        fb, ft, fs = prism(bm, cpoly, -off - cap_t, -off)
        metal += [fb, ft] + fs
        # small notch mark on the plate (polarity key)
        fb, ft, fs = box(bm, L / 2 - 0.55, -0.18, -off - cap_t - 0.03, L / 2 - 0.25, 0.18, -off - cap_t)
        plastic += [fb, ft] + fs

    set_material(plastic, 0)
    set_material(metal, 1)
    set_material(gold, 2)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    uv = bm.loops.layers.uv.new("UVMap")
    box_uv(bm, uv, list(bm.faces), (-L / 2, -W / 2, 0, L / 2, W / 2, H))

    rot = Matrix.Rotation(math.radians(con.get("rotation_deg", 0)), 4, "Z")
    if side_back:
        # mating face towards -Z, sitting under the flex
        place = Matrix.Translation((cx, cy, flex_z0)) @ rot @ Matrix.Rotation(math.pi, 4, "X")
    else:
        place = Matrix.Translation((cx, cy, flex_z1)) @ rot
    bmesh.ops.transform(bm, matrix=place, verts=bm.verts)
    ob = new_object(con["name"], bm,
                    [mats["Connector_Plastic"], mats["Connector_Metal"], mats["Connector_Gold"]])
    mark_sharp_edges(ob.data, 35.0)
    return ob


# --------------------------------------------------------------------------- UV templates
def write_uv_templates(spec, cell_poly, bb, out_dir, px_long=2048):
    """PNG templates (black bg, white outline + grid) matching the Cell_Front/Cell_Back UV mapping."""
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        print("Pillow not available, skipping UV templates")
        return
    x0, y0, x1, y1 = bb
    w_mm, h_mm = x1 - x0, y1 - y0
    if h_mm >= w_mm:
        H = px_long
        W = int(round(px_long * w_mm / h_mm))
    else:
        W = px_long
        H = int(round(px_long * h_mm / w_mm))
    for side in ("front", "back"):
        im = Image.new("RGB", (W, H), (0, 0, 0))
        d = ImageDraw.Draw(im)

        def to_px(x, y):
            u = (x - x0) / w_mm
            v = (y - y0) / h_mm
            if side == "back":
                u = 1.0 - u
            return (u * (W - 1), (1.0 - v) * (H - 1))

        # 10 mm grid
        for gx in range(0, int(w_mm) + 1, 10):
            xs = to_px(x0 + gx, y0)[0]
            d.line([(xs, 0), (xs, H)], fill=(40, 40, 40), width=1)
        for gy in range(0, int(h_mm) + 1, 10):
            ys = to_px(x0, y0 + gy)[1]
            d.line([(0, ys), (W, ys)], fill=(40, 40, 40), width=1)
        pts = [to_px(x, y) for x, y in cell_poly]
        d.polygon(pts, outline=(255, 255, 255))
        d.line(pts + [pts[0]], fill=(255, 255, 255), width=3)
        # flex exits, so the texture author knows where the flexes are
        for fx in spec.get("flexes", []):
            fpoly = ribbon_polygon(fx["path"], fx.get("radii", [0] * len(fx["path"])), fx.get("width", 1.2)) if "path" in fx else fx["polygon"]
            fp = [to_px(x, y) for x, y in fpoly]
            d.line(fp + [fp[0]], fill=(120, 120, 120), width=2)
            if fx.get("pad"):
                pc, ps = fx["pad"]["center"], fx["pad"]["size"]
                pp = [to_px(x, y) for x, y in rounded_rect(pc[0], pc[1], ps[0], ps[1], 0.4, fx["pad"].get("rotation_deg", 0))]
                d.line(pp + [pp[0]], fill=(120, 120, 120), width=2)
        for lip in spec.get("seal_lips", []) + spec.get("side_wraps", []):
            lx0, ly0, lx1, ly1 = lip["rect"]
            lp = [to_px(lx0, ly0), to_px(lx1, ly0), to_px(lx1, ly1), to_px(lx0, ly1)]
            d.line(lp + [lp[0]], fill=(90, 90, 90), width=2)
        label = "%s  %s  %.1f x %.1f mm  (%d x %d px)" % (spec["name"], side.upper(), w_mm, h_mm, W, H)
        d.text((12, 12), label, fill=(255, 255, 255))
        d.text((12, 28), "UV: u = x/width, v = y/height" + ("  (mirrored for the back)" if side == "back" else ""),
               fill=(200, 200, 200))
        im.save(os.path.join(out_dir, "uv_template_%s.png" % side))
    # flexes: planar bbox UV
    for fx in spec.get("flexes", []):
        poly = ribbon_polygon(fx["path"], fx.get("radii", [0] * len(fx["path"])), fx.get("width", 1.2)) if "path" in fx else fillet_polygon(fx["polygon"], fx.get("fillets", [0] * len(fx["polygon"])))
        fx0, fy0, fx1, fy1 = poly_bbox(poly)
        fw, fh = fx1 - fx0, fy1 - fy0
        scale = 1024 / max(fw, fh)
        W2, H2 = int(fw * scale), int(fh * scale)
        im = Image.new("RGB", (W2, H2), (0, 0, 0))
        d = ImageDraw.Draw(im)
        pts = [((x - fx0) / fw * (W2 - 1), (1 - (y - fy0) / fh) * (H2 - 1)) for x, y in poly]
        d.line(pts + [pts[0]], fill=(255, 255, 255), width=2)
        d.text((8, 8), "%s  %.1f x %.1f mm" % (fx["name"], fw, fh), fill=(255, 255, 255))
        im.save(os.path.join(out_dir, "uv_template_%s.png" % fx["name"].lower()))


# --------------------------------------------------------------------------- scene / render
def setup_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.unit_settings.system = "METRIC"
    sc.unit_settings.length_unit = "MILLIMETERS"
    sc.unit_settings.scale_length = 1.0
    return sc


def add_studio(bb, thickness):
    x0, y0, x1, y1 = bb
    cx, cy = (x0 + x1) / 2 * MM, (y0 + y1) / 2 * MM
    sc = bpy.context.scene
    sc.world = bpy.data.worlds.new("World")
    sc.world.use_nodes = True
    bg = sc.world.node_tree.nodes["Background"]
    bg.inputs[0].default_value = (0.75, 0.75, 0.75, 1)
    bg.inputs[1].default_value = 0.25

    def area(name, loc, rot, size, energy):
        ld = bpy.data.lights.new(name, "AREA")
        ld.energy = energy
        ld.size = size
        ob = bpy.data.objects.new(name, ld)
        ob.location = loc
        ob.rotation_euler = rot
        sc.collection.objects.link(ob)
        return ob

    # big, dim softboxes: a glossy black cell mirrors the lights, so keep their radiance (W / (pi * area)) low
    area("Key", (cx - 0.35, cy + 0.25, 0.65), (math.radians(28), math.radians(-28), 0), 1.0, 10.0)
    area("Fill", (cx + 0.45, cy - 0.15, 0.5), (math.radians(20), math.radians(40), 0), 1.2, 4.0)
    area("Rim", (cx, cy + 0.5, 0.15), (math.radians(75), 0, 0), 0.8, 3.0)
    area("Under", (cx + 0.2, cy, -0.6), (math.radians(180), 0, 0), 1.2, 8.0)


def camera_look_at(cam, target, distance, direction, up=(0.0, 1.0, 0.0)):
    """Place the camera at target + direction*distance looking at target, image-up = world `up` (+Y by default)."""
    d = Vector(direction).normalized()
    cam.location = Vector(target) + d * distance
    z_cam = d.copy()                      # camera +Z points away from the target
    upv = Vector(up)
    if abs(upv.dot(z_cam)) > 0.999:
        upv = Vector((0.0, 0.0, 1.0))
    x_cam = upv.cross(z_cam).normalized()
    y_cam = z_cam.cross(x_cam).normalized()
    m = Matrix((x_cam, y_cam, z_cam)).transposed()  # columns = camera axes
    cam.rotation_euler = m.to_euler()


def parts_bbox(spec, bb):
    """2D bbox (mm) of cell + flexes, used to frame the camera."""
    x0, y0, x1, y1 = bb
    for fx in spec.get("flexes", []):
        pts = list(fx.get("path", fx.get("polygon", [])))
        if fx.get("pad"):
            pc, ps = fx["pad"]["center"], fx["pad"]["size"]
            m = max(ps) / 2
            pts += [(pc[0] - m, pc[1] - m), (pc[0] + m, pc[1] + m)]
        fx0, fy0, fx1, fy1 = poly_bbox(pts)
        x0, y0, x1, y1 = min(x0, fx0), min(y0, fy0), max(x1, fx1), max(y1, fy1)
    for lip in spec.get("seal_lips", []) + spec.get("side_wraps", []):
        lx0, ly0, lx1, ly1 = lip["rect"]
        x0, y0, x1, y1 = min(x0, lx0), min(y0, ly0), max(x1, lx1), max(y1, ly1)
    return x0, y0, x1, y1


def apply_uv_check_textures(out_dir, mats):
    """Plug the generated UV templates into the cell/flex materials to verify the mapping visually."""
    pairs = {"Cell_Front": "uv_template_front.png", "Cell_Back": "uv_template_back.png",
             "Flex": "uv_template_flex_main.png"}
    for mname, fname in pairs.items():
        path = os.path.join(out_dir, fname)
        if not os.path.exists(path):
            continue
        m = mats[mname]
        nt = m.node_tree
        bsdf = nt.nodes.get("Principled BSDF")
        tex = nt.nodes.new("ShaderNodeTexImage")
        tex.image = bpy.data.images.load(os.path.abspath(path))
        nt.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
        bsdf.inputs["Roughness"].default_value = 0.6


def render_views(spec, bb, out_dir, samples, quick=False, only=None, suffix=""):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = samples
    sc.cycles.use_denoising = True
    sc.render.film_transparent = False
    sc.view_settings.view_transform = "Filmic" if "Filmic" in [i.identifier for i in sc.view_settings.bl_rna.properties["view_transform"].enum_items] else "AgX"
    x0, y0, x1, y1 = parts_bbox(spec, bb)
    cx, cy = (x0 + x1) / 2 * MM, (y0 + y1) / 2 * MM
    cz = spec["thickness"] / 2 * MM
    h = (y1 - y0) * MM

    cam_data = bpy.data.cameras.new("Cam")
    cam_data.clip_start = 0.001
    cam = bpy.data.objects.new("Cam", cam_data)
    sc.collection.objects.link(cam)
    sc.camera = cam

    views = {
        "front": dict(dir=(0.0, -0.12, 1.0), lens=85, size=(1400, 1800), dist_k=2.75),
        "back": dict(dir=(0.0, -0.12, -1.0), lens=85, size=(1400, 1800), dist_k=2.75),
        "three_quarter": dict(dir=(0.9, -0.9, 1.1), lens=70, size=(1800, 1400), dist_k=2.6),
    }
    scale = 0.45 if quick else 1.0
    for name, v in views.items():
        if only and name not in only:
            continue
        cam_data.lens = v["lens"]
        cam_data.sensor_width = 36
        sc.render.resolution_x, sc.render.resolution_y = int(v["size"][0] * scale), int(v["size"][1] * scale)
        camera_look_at(cam, (cx, cy, cz), h * v["dist_k"], v["dir"])
        sc.render.filepath = os.path.join(out_dir, "preview_%s%s.png" % (name, suffix))
        bpy.ops.render.render(write_still=True)

    # macro views on each connector, both sides
    for fx in spec.get("flexes", []):
        con = fx.get("connector")
        if not con:
            continue
        tx, ty = con["center"]
        t = (tx * MM, ty * MM, cz)
        for side, d in (("back", (0.55, -0.6, -1.0)), ("front", (0.55, -0.6, 1.0))):
            vname = "%s_%s" % (con["name"].lower(), side)
            if only and vname not in only and "connectors" not in only:
                continue
            cam_data.lens = 100
            sc.render.resolution_x, sc.render.resolution_y = int(1400 * scale), int(1000 * scale)
            camera_look_at(cam, t, 0.022, d)
            sc.render.filepath = os.path.join(out_dir, "preview_%s_%s%s.png" % (con["name"].lower(), side, suffix))
            bpy.ops.render.render(write_still=True)


# --------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("spec")
    ap.add_argument("--out", default="models")
    ap.add_argument("--render", action="store_true")
    ap.add_argument("--samples", type=int, default=64)
    ap.add_argument("--no-export", action="store_true")
    ap.add_argument("--quick", action="store_true", help="small, noisy preview renders")
    ap.add_argument("--views", default="", help="comma list: front,back,three_quarter,connectors,connector_main_back,...")
    ap.add_argument("--uvcheck", action="store_true", help="also render front/back with the UV templates as textures")
    ap.add_argument("--gloss", action="store_true", help="render the cell with a light glossy material to inspect shading")
    args = ap.parse_args()

    with open(args.spec, "r", encoding="utf-8") as f:
        spec = json.load(f)
    out_dir = os.path.join(args.out, spec["id"])
    os.makedirs(out_dir, exist_ok=True)

    setup_scene()
    mats = materials()
    cell, cell_poly, bb = build_cell(spec, mats)
    parts = [cell]
    parts += build_seal_lips(spec, mats)
    parts += build_side_wraps(spec, mats)
    for fx in spec.get("flexes", []):
        parts += build_flex(spec, fx, mats)

    # group everything under an empty so the FBX has a single root
    root = bpy.data.objects.new(spec["id"], None)
    bpy.context.scene.collection.objects.link(root)
    for ob in parts:
        ob.parent = root

    write_uv_templates(spec, cell_poly, bb, out_dir)
    add_studio(bb, spec["thickness"])

    if not args.no_export:
        base = os.path.join(out_dir, spec["id"])
        bpy.ops.object.select_all(action="DESELECT")
        for ob in parts + [root]:
            ob.select_set(True)
        bpy.ops.export_scene.fbx(filepath=base + ".fbx", use_selection=True, apply_unit_scale=True,
                                 apply_scale_options="FBX_SCALE_NONE", mesh_smooth_type="EDGE",
                                 use_mesh_modifiers=True, add_leaf_bones=False, path_mode="AUTO")
        bpy.ops.export_scene.gltf(filepath=base + ".glb", export_format="GLB", use_selection=True,
                                  export_apply=True, export_yup=True)
        bpy.ops.wm.save_as_mainfile(filepath=base + ".blend")

    if args.render:
        only = [v for v in args.views.split(",") if v] or None
        if args.gloss:
            for mn in ("Cell_Front", "Cell_Back", "Cell_Side"):
                bsdf = mats[mn].node_tree.nodes.get("Principled BSDF")
                bsdf.inputs["Base Color"].default_value = (0.5, 0.5, 0.5, 1)
                bsdf.inputs["Roughness"].default_value = 0.2
        smp = args.samples if not args.quick else min(args.samples, 24)
        render_views(spec, bb, out_dir, smp, args.quick, only)
        if args.uvcheck:
            apply_uv_check_textures(out_dir, mats)
            render_views(spec, bb, out_dir, smp, args.quick, ["front", "back"], suffix="_uvcheck")

    # report
    for ob in parts:
        print("%-18s verts=%6d faces=%6d" % (ob.name, len(ob.data.vertices), len(ob.data.polygons)))
    print("done ->", out_dir)


if __name__ == "__main__":
    # strip blender-style args if run through `blender -b -P`
    if "--" in sys.argv:
        sys.argv = [sys.argv[0]] + sys.argv[sys.argv.index("--") + 1:]
    main()
