#!/usr/bin/env python3
"""
Detailed structural analysis of three DXF files:
  1. FINISHED MODEL.dxf  – target/reference output
  2. Test_3D_Output.dxf  – current pipeline output
  3. Test.dxf            – source input
"""

import ezdxf
from ezdxf import units
from collections import defaultdict
import math

FILES = {
    "FINISHED MODEL": "/home/user/test.dxf/FINISHED MODEL.dxf",
    "Test_3D_Output":  "/home/user/test.dxf/Test_3D_Output.dxf",
    "Test":            "/home/user/test.dxf/Test.dxf",
}

def fmt_z(z):
    return f"{z:.4f}" if z is not None else "None"

def analyse_lwpolyline(entity):
    """Return (elevation, has_nonzero_elevation, vertex_z_values, has_nonzero_vertex_z)"""
    elev = entity.dxf.get("elevation", 0.0)
    has_nonzero_elev = abs(elev) > 1e-9

    vertex_zs = []
    try:
        for pt in entity.get_points(format="xyzb"):
            vertex_zs.append(pt[2])
    except Exception:
        # fallback: LWPOLYLINE stores points as (x,y,start_width,end_width,bulge)
        try:
            for pt in entity.get_points(format="xyzsb"):
                vertex_zs.append(pt[2])
        except Exception:
            vertex_zs = []

    has_nonzero_vz = any(abs(z) > 1e-9 for z in vertex_zs)
    return elev, has_nonzero_elev, vertex_zs, has_nonzero_vz

def analyse_line(entity):
    start = entity.dxf.start
    end   = entity.dxf.end
    return start, end, abs(start[2]) > 1e-9 or abs(end[2]) > 1e-9

def analyse_3dface(entity):
    pts = [entity.dxf.vtx0, entity.dxf.vtx1, entity.dxf.vtx2, entity.dxf.vtx3]
    has_nonzero_z = any(abs(p[2]) > 1e-9 for p in pts)
    zs = [p[2] for p in pts]
    return pts, zs, has_nonzero_z

def analyse_point(entity):
    loc = entity.dxf.location
    return loc, abs(loc[2]) > 1e-9

def analyse_file(label, path):
    print("=" * 80)
    print(f"FILE: {label}")
    print(f"PATH: {path}")
    print("=" * 80)

    try:
        doc = ezdxf.readfile(path)
    except Exception as e:
        print(f"  ERROR reading file: {e}")
        return

    msp = doc.modelspace()

    # ── 1. Layer table ──────────────────────────────────────────────────────────
    layer_names = sorted(layer.dxf.name for layer in doc.layers)
    print(f"\n[LAYERS]  total={len(layer_names)}")
    for ln in layer_names:
        print(f"  {ln}")

    # ── 2. Entity census ────────────────────────────────────────────────────────
    # layer → entity_type → list of entities
    layer_entities = defaultdict(lambda: defaultdict(list))

    for entity in msp:
        layer = entity.dxf.get("layer", "0")
        etype = entity.dxftype()
        layer_entities[layer][etype].append(entity)

    # Also recurse into INSERT (blocks) – optional, flag if present
    insert_count = sum(len(v.get("INSERT", [])) for v in layer_entities.values())

    print(f"\n[ENTITY SUMMARY BY LAYER]")
    for layer in sorted(layer_entities.keys()):
        type_counts = {t: len(ents) for t, ents in layer_entities[layer].items()}
        total = sum(type_counts.values())
        type_str = ", ".join(f"{t}:{c}" for t, c in sorted(type_counts.items()))
        print(f"  {layer:40s}  total={total:4d}  [{type_str}]")

    # ── 3. LWPOLYLINE elevation/Z detail ───────────────────────────────────────
    print(f"\n[LWPOLYLINE Z/ELEVATION DETAIL]")
    lw_with_elev = 0
    lw_with_vz   = 0
    lw_total     = 0

    for layer in sorted(layer_entities.keys()):
        polys = layer_entities[layer].get("LWPOLYLINE", [])
        if not polys:
            continue
        layer_elev_set  = []
        layer_vz_set    = []
        sample_elevs    = []
        sample_vz       = []
        for ent in polys:
            elev, has_e, vz, has_vz = analyse_lwpolyline(ent)
            lw_total += 1
            if has_e:
                lw_with_elev += 1
                layer_elev_set.append(elev)
                sample_elevs.append(elev)
            if has_vz:
                lw_with_vz += 1
                layer_vz_set.extend(vz)
                sample_vz.extend(vz)

        unique_elevs = sorted(set(round(e, 4) for e in layer_elev_set))
        unique_vz    = sorted(set(round(z, 4) for z in layer_vz_set))
        non_zero_elev = len(layer_elev_set)
        non_zero_vz   = len(layer_vz_set)

        elev_info = f"non-zero elevations={non_zero_elev}/{len(polys)}"
        if unique_elevs:
            elev_info += f"  values={unique_elevs[:8]}"
        vz_info = f"non-zero vertex-Z={non_zero_vz}/{len(polys)}"
        if unique_vz:
            vz_info += f"  values={unique_vz[:8]}"

        print(f"  {layer:40s}  {elev_info}")
        print(f"  {'':40s}  {vz_info}")

    print(f"\n  LWPOLYLINE totals: {lw_total} total, "
          f"{lw_with_elev} with non-zero .elevation, "
          f"{lw_with_vz} with non-zero vertex-Z")

    # ── 4. LINE Z detail ────────────────────────────────────────────────────────
    print(f"\n[LINE Z DETAIL]")
    for layer in sorted(layer_entities.keys()):
        lines = layer_entities[layer].get("LINE", [])
        if not lines:
            continue
        nonzero = 0
        sample_zs = []
        for ent in lines:
            start, end, has_z = analyse_line(ent)
            if has_z:
                nonzero += 1
                sample_zs += [start[2], end[2]]
        unique_zs = sorted(set(round(z, 4) for z in sample_zs))
        print(f"  {layer:40s}  {nonzero}/{len(lines)} have non-zero Z  "
              f"values={unique_zs[:8]}")

    # ── 5. 3DFACE Z detail ──────────────────────────────────────────────────────
    print(f"\n[3DFACE Z DETAIL]")
    for layer in sorted(layer_entities.keys()):
        faces = layer_entities[layer].get("3DFACE", [])
        if not faces:
            continue
        nonzero = 0
        zs_all = []
        for ent in faces:
            pts, zs, has_z = analyse_3dface(ent)
            if has_z:
                nonzero += 1
            zs_all.extend(zs)
        unique_zs = sorted(set(round(z, 4) for z in zs_all))
        print(f"  {layer:40s}  {nonzero}/{len(faces)} have non-zero Z  "
              f"Z range=[{min(zs_all):.4f}, {max(zs_all):.4f}]  "
              f"unique count={len(unique_zs)}")

    # ── 6. POINT Z detail ───────────────────────────────────────────────────────
    print(f"\n[POINT Z DETAIL]")
    for layer in sorted(layer_entities.keys()):
        pts_list = layer_entities[layer].get("POINT", [])
        if not pts_list:
            continue
        nonzero = 0
        zs = []
        for ent in pts_list:
            loc, has_z = analyse_point(ent)
            if has_z:
                nonzero += 1
            zs.append(loc[2])
        unique_zs = sorted(set(round(z, 4) for z in zs))
        print(f"  {layer:40s}  {nonzero}/{len(pts_list)} have non-zero Z  "
              f"values={unique_zs[:10]}")

    # ── 7. Spot-check: first LWPOLYLINE per layer (elevation + first 3 vertices) ─
    print(f"\n[SPOT-CHECK: first LWPOLYLINE per layer – elevation + first 3 vertices]")
    for layer in sorted(layer_entities.keys()):
        polys = layer_entities[layer].get("LWPOLYLINE", [])
        if not polys:
            continue
        ent = polys[0]
        elev = ent.dxf.get("elevation", 0.0)
        try:
            verts = list(ent.get_points(format="xyzb"))
        except Exception:
            verts = []
        vert_str = "  ".join(f"({v[0]:.2f},{v[1]:.2f},z={v[2]:.4f})" for v in verts[:3])
        closed = ent.is_closed
        print(f"  {layer:40s}  elev={elev:.4f}  closed={closed}  verts[0:3]: {vert_str}")

    print()


# ── Run analysis ────────────────────────────────────────────────────────────────
for label, path in FILES.items():
    analyse_file(label, path)


# ── Cross-file comparison ───────────────────────────────────────────────────────
print("=" * 80)
print("CROSS-FILE COMPARISON: FINISHED MODEL vs Test_3D_Output")
print("=" * 80)

def get_layer_type_map(path):
    doc = ezdxf.readfile(path)
    msp = doc.modelspace()
    layer_types = defaultdict(set)
    layer_counts = defaultdict(lambda: defaultdict(int))
    for entity in msp:
        layer = entity.dxf.get("layer", "0")
        etype = entity.dxftype()
        layer_types[layer].add(etype)
        layer_counts[layer][etype] += 1
    return layer_types, layer_counts

fm_lt,  fm_lc  = get_layer_type_map("/home/user/test.dxf/FINISHED MODEL.dxf")
t3_lt,  t3_lc  = get_layer_type_map("/home/user/test.dxf/Test_3D_Output.dxf")
src_lt, src_lc = get_layer_type_map("/home/user/test.dxf/Test.dxf")

fm_layers  = set(fm_lt.keys())
t3_layers  = set(t3_lt.keys())
src_layers = set(src_lt.keys())

print(f"\nLayers in FINISHED MODEL only (not in Test_3D_Output):  {sorted(fm_layers - t3_layers)}")
print(f"Layers in Test_3D_Output only (not in FINISHED MODEL): {sorted(t3_layers - fm_layers)}")
print(f"Layers in both:                                         {sorted(fm_layers & t3_layers)}")
print(f"\nLayers in source Test.dxf:                              {sorted(src_layers)}")
print(f"Source layers carried through to FINISHED MODEL:        {sorted(src_layers & fm_layers)}")
print(f"Source layers carried through to Test_3D_Output:        {sorted(src_layers & t3_layers)}")

print("\n\n[ENTITY TYPE DIFFERENCES for shared layers]")
for layer in sorted(fm_layers & t3_layers):
    fm_types = fm_lt[layer]
    t3_types = t3_lt[layer]
    if fm_types != t3_types:
        print(f"  {layer}")
        print(f"    FINISHED MODEL  types: {sorted(fm_types)}")
        print(f"    Test_3D_Output  types: {sorted(t3_types)}")
        # count differences
        for etype in sorted(fm_types | t3_types):
            fc = fm_lc[layer].get(etype, 0)
            tc = t3_lc[layer].get(etype, 0)
            if fc != tc:
                print(f"      {etype:20s}  FM={fc:4d}  T3={tc:4d}  diff={fc-tc:+d}")
    else:
        # same types – check counts
        diffs = []
        for etype in sorted(fm_types):
            fc = fm_lc[layer].get(etype, 0)
            tc = t3_lc[layer].get(etype, 0)
            if fc != tc:
                diffs.append(f"{etype}: FM={fc} T3={tc}")
        if diffs:
            print(f"  {layer}  (same types, different counts)")
            for d in diffs:
                print(f"    {d}")
