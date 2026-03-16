#!/usr/bin/env python3
"""
Detailed structural comparison of three DXF files.
"""

import ezdxf
from ezdxf import units
from collections import defaultdict
import sys

FILES = {
    "FINISHED MODEL": "/home/user/test.dxf/FINISHED MODEL.dxf",
    "Test_3D_Output":  "/home/user/test.dxf/Test_3D_Output.dxf",
    "Test (source)":   "/home/user/test.dxf/Test.dxf",
}

# ── helpers ──────────────────────────────────────────────────────────────────

def lwpoly_z_info(e):
    """Return (elevation, has_nonzero_vertex_z, vertex_z_values) for a LWPOLYLINE."""
    elev = e.dxf.get("elevation", 0.0)
    zvals = [v[2] if len(v) > 2 else 0.0 for v in e.get_points("xyb")]
    # get_points returns (x,y,bulge) for LWPOLYLINE; Z is only in elevation
    # Use the raw vertices for proper Z check
    raw = list(e.vertices())          # each vertex is (x, y, [bulge, [sw, ew, w]])
    # elevation IS the z; vertex coords are 2‑D in LWPOLYLINE
    return elev, False, []            # LWPOLYLINE vertices have no per-vertex Z

def lwpoly_z_info_v2(e):
    """Better version: read dxf.elevation and also attempt vertex Z."""
    elev = 0.0
    try:
        elev = e.dxf.elevation
    except Exception:
        pass
    # LWPOLYLINE stores 2‑D points; Z is purely in the elevation attribute
    return elev

def poly3d_z_info(e):
    """Return list of (x,y,z) for a POLYLINE (3D) or LWPOLYLINE."""
    pts = []
    for v in e.vertices:
        pts.append((v.dxf.location.x, v.dxf.location.y, v.dxf.location.z))
    return pts

def face_z(e):
    """Return all Z values from a 3DFACE."""
    zs = []
    for attr in ("vtx0","vtx1","vtx2","vtx3"):
        try:
            pt = e.dxf.get(attr)
            if pt:
                zs.append(pt.z)
        except Exception:
            pass
    return zs

def line_z(e):
    """Return start/end Z for a LINE."""
    try:
        return e.dxf.start.z, e.dxf.end.z
    except Exception:
        return 0.0, 0.0

def point_z(e):
    try:
        return e.dxf.location.z
    except Exception:
        return 0.0


# ── per‑file analysis ─────────────────────────────────────────────────────────

def analyse(label, path):
    print("=" * 70)
    print(f"FILE: {label}")
    print(f"PATH: {path}")
    print("=" * 70)

    try:
        doc = ezdxf.readfile(path)
    except Exception as ex:
        print(f"  ERROR reading file: {ex}")
        return

    msp = doc.modelspace()

    # Collect all layers defined in the layer table
    defined_layers = sorted([l.dxf.name for l in doc.layers])
    print(f"\nDefined layers ({len(defined_layers)}):")
    for ln in defined_layers:
        print(f"  {ln}")

    # Walk all entities in modelspace
    layer_entities = defaultdict(list)   # layer -> list of entity type strings
    for e in msp:
        layer_entities[e.dxf.layer].append(e)

    used_layers = sorted(layer_entities.keys())
    print(f"\nLayers with entities ({len(used_layers)}):")

    for layer in used_layers:
        ents = layer_entities[layer]
        type_counts = defaultdict(int)
        for e in ents:
            type_counts[e.dxftype()] += 1
        tc_str = ", ".join(f"{t}×{n}" for t, n in sorted(type_counts.items()))
        print(f"\n  Layer: '{layer}'  (total: {len(ents)})")
        print(f"    Types: {tc_str}")

        # ---------- LWPOLYLINE details ----------
        lwpolys = [e for e in ents if e.dxftype() == "LWPOLYLINE"]
        if lwpolys:
            nonzero_elev = []
            zero_elev    = []
            for e in lwpolys:
                elev = lwpoly_z_info_v2(e)
                if abs(elev) > 1e-9:
                    nonzero_elev.append(elev)
                else:
                    zero_elev.append(elev)
            print(f"    LWPOLYLINE elevation stats:")
            print(f"      with non-zero elevation: {len(nonzero_elev)}")
            if nonzero_elev:
                unique_z = sorted(set(round(z,4) for z in nonzero_elev))
                print(f"      unique elevation values: {unique_z[:30]}")
            print(f"      with zero elevation:     {len(zero_elev)}")

        # ---------- LINE details ----------
        lines = [e for e in ents if e.dxftype() == "LINE"]
        if lines:
            nz = [(line_z(e)) for e in lines if any(abs(z) > 1e-9 for z in line_z(e))]
            print(f"    LINE elevation stats:")
            print(f"      with non-zero Z (start or end): {len(nz)}")
            if nz:
                all_z = sorted(set(round(z, 4) for pair in nz for z in pair if abs(z) > 1e-9))
                print(f"      unique Z values: {all_z[:30]}")

        # ---------- 3DFACE details ----------
        faces = [e for e in ents if e.dxftype() == "3DFACE"]
        if faces:
            all_z = []
            for e in faces:
                all_z.extend(face_z(e))
            nz = [z for z in all_z if abs(z) > 1e-9]
            unique_z = sorted(set(round(z,4) for z in nz))
            print(f"    3DFACE elevation stats:")
            print(f"      total vertex Z values: {len(all_z)}, non-zero: {len(nz)}")
            if unique_z:
                print(f"      unique Z values: {unique_z[:30]}")

        # ---------- POINT details ----------
        points = [e for e in ents if e.dxftype() == "POINT"]
        if points:
            nz = [point_z(e) for e in points if abs(point_z(e)) > 1e-9]
            print(f"    POINT elevation stats:")
            print(f"      with non-zero Z: {len(nz)}")
            if nz:
                print(f"      unique Z: {sorted(set(round(z,4) for z in nz))[:20]}")

    print()


# ── comparison ────────────────────────────────────────────────────────────────

def compare(label_a, path_a, label_b, path_b):
    print("=" * 70)
    print(f"COMPARISON: '{label_a}'  vs  '{label_b}'")
    print("=" * 70)

    try:
        doc_a = ezdxf.readfile(path_a)
        doc_b = ezdxf.readfile(path_b)
    except Exception as ex:
        print(f"  ERROR: {ex}")
        return

    msp_a = doc_a.modelspace()
    msp_b = doc_b.modelspace()

    def layer_type_map(msp):
        m = defaultdict(lambda: defaultdict(int))
        for e in msp:
            m[e.dxf.layer][e.dxftype()] += 1
        return m

    ma = layer_type_map(msp_a)
    mb = layer_type_map(msp_b)

    layers_a = set(ma.keys())
    layers_b = set(mb.keys())

    only_a = sorted(layers_a - layers_b)
    only_b = sorted(layers_b - layers_a)
    common = sorted(layers_a & layers_b)

    print(f"\nLayers only in '{label_a}' ({len(only_a)}):")
    for ln in only_a:
        tc = ", ".join(f"{t}×{n}" for t,n in sorted(ma[ln].items()))
        print(f"  '{ln}': {tc}")

    print(f"\nLayers only in '{label_b}' ({len(only_b)}):")
    for ln in only_b:
        tc = ", ".join(f"{t}×{n}" for t,n in sorted(mb[ln].items()))
        print(f"  '{ln}': {tc}")

    print(f"\nCommon layers ({len(common)}) — count differences:")
    for ln in common:
        total_a = sum(ma[ln].values())
        total_b = sum(mb[ln].values())
        if total_a != total_b:
            tc_a = ", ".join(f"{t}×{n}" for t,n in sorted(ma[ln].items()))
            tc_b = ", ".join(f"{t}×{n}" for t,n in sorted(mb[ln].items()))
            print(f"  '{ln}': A={total_a} ({tc_a})  B={total_b} ({tc_b})")
        # else identical — skip to keep output concise

    identical = [ln for ln in common
                 if sum(ma[ln].values()) == sum(mb[ln].values())]
    print(f"\n  ({len(identical)} common layers have identical entity counts)")
    print()


# ── deep dive: building pad / FFL / fence / drive layers ─────────────────────

KEYWORDS = [
    "ffl", "pad", "plot", "building", "fence", "drive", "road",
    "ground", "gl", "retaining", "wall", "level",
]

def keyword_layer_search(label, path):
    print("=" * 70)
    print(f"KEYWORD LAYER SEARCH in '{label}'")
    print("=" * 70)
    try:
        doc = ezdxf.readfile(path)
    except Exception as ex:
        print(f"  ERROR: {ex}")
        return
    msp = doc.modelspace()
    layer_entities = defaultdict(list)
    for e in msp:
        layer_entities[e.dxf.layer].append(e)

    for layer in sorted(layer_entities.keys()):
        lname_lower = layer.lower()
        if any(kw in lname_lower for kw in KEYWORDS):
            ents = layer_entities[layer]
            type_counts = defaultdict(int)
            for e in ents:
                type_counts[e.dxftype()] += 1
            tc_str = ", ".join(f"{t}×{n}" for t,n in sorted(type_counts.items()))

            # Elevation sample for LWPOLYLINES
            lwpolys = [e for e in ents if e.dxftype() == "LWPOLYLINE"]
            elev_sample = ""
            if lwpolys:
                elevs = [lwpoly_z_info_v2(e) for e in lwpolys]
                nz = sorted(set(round(z,4) for z in elevs if abs(z)>1e-9))
                elev_sample = f"  | LWPOLY elevations (non-zero): {nz[:20]}" if nz else "  | LWPOLY elevations: all zero"

            print(f"  '{layer}': {tc_str}{elev_sample}")
    print()


# ── main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    for label, path in FILES.items():
        analyse(label, path)

    compare(
        "FINISHED MODEL", FILES["FINISHED MODEL"],
        "Test_3D_Output",  FILES["Test_3D_Output"],
    )
    compare(
        "Test (source)", FILES["Test (source)"],
        "Test_3D_Output",  FILES["Test_3D_Output"],
    )

    for label, path in FILES.items():
        keyword_layer_search(label, path)
