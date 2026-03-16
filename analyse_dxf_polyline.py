#!/usr/bin/env python3
"""
Dig into the POLYLINE entities in FINISHED MODEL.dxf
to understand structure of PLOTS FFL, PLOTS EXTERNAL LEVEL, and 3D_LINES
"""
import ezdxf
from collections import defaultdict

def analyse_polyline(ent, label=""):
    """Return a summary string for a POLYLINE entity."""
    flags = ent.dxf.get("flags", 0)
    is_3d = bool(flags & ezdxf.lldxf.const.POLYLINE_3D_POLYLINE)
    is_mesh = bool(flags & ezdxf.lldxf.const.POLYLINE_3D_POLYMESH)
    is_closed = ent.is_closed
    try:
        vertices = list(ent.vertices)
    except Exception:
        vertices = []
    z_vals = []
    for v in vertices:
        try:
            z_vals.append(v.dxf.location[2])
        except Exception:
            pass
    has_nonzero_z = any(abs(z) > 1e-9 for z in z_vals)
    unique_z = sorted(set(round(z, 4) for z in z_vals))
    return (f"flags={flags:#06x} 3d={is_3d} mesh={is_mesh} closed={is_closed} "
            f"verts={len(vertices)} has_nonzero_z={has_nonzero_z} "
            f"z_range=[{min(z_vals) if z_vals else 'N/A':.4f},{max(z_vals) if z_vals else 'N/A':.4f}] "
            f"unique_z_count={len(unique_z)}")

path = "/home/user/test.dxf/FINISHED MODEL.dxf"
doc = ezdxf.readfile(path)
msp = doc.modelspace()

layer_ents = defaultdict(list)
for ent in msp:
    layer_ents[ent.dxf.get("layer", "0")].append(ent)

# Focus layers
focus = ["PLOTS FFL", "PLOTS EXTERNAL LEVEL", "3D_LINES"]

for layer in focus:
    ents = layer_ents.get(layer, [])
    print(f"\n{'='*70}")
    print(f"LAYER: {layer}   ({len(ents)} entities)")
    print(f"{'='*70}")
    for i, ent in enumerate(ents[:5]):   # first 5
        if ent.dxftype() == "POLYLINE":
            info = analyse_polyline(ent)
            print(f"  [{i}] POLYLINE  {info}")
            # show first 3 vertex z values
            try:
                verts = list(ent.vertices)
                zs = [v.dxf.location[2] for v in verts[:6]]
                print(f"       first-6 vertex Z: {[round(z,4) for z in zs]}")
            except Exception as e:
                print(f"       (vertex read error: {e})")
    if len(ents) > 5:
        print(f"  ... ({len(ents)-5} more)")

    # unique Z across all entities in layer
    all_z = []
    for ent in ents:
        if ent.dxftype() == "POLYLINE":
            try:
                for v in ent.vertices:
                    all_z.append(v.dxf.location[2])
            except Exception:
                pass
    if all_z:
        unique_z = sorted(set(round(z, 4) for z in all_z))
        print(f"\n  LAYER SUMMARY: {len(all_z)} total vertex Z values")
        print(f"  unique Z values ({len(unique_z)} total): {unique_z[:20]}")
        print(f"  Z range: [{min(all_z):.4f}, {max(all_z):.4f}]")
        nonzero = sum(1 for z in all_z if abs(z) > 1e-9)
        print(f"  Non-zero Z: {nonzero}/{len(all_z)}")

# Count polylines per layer
print("\n\n=== POLYLINE counts in 3D_LINES – vertex count distribution ===")
lines3d = layer_ents.get("3D_LINES", [])
vert_counts = defaultdict(int)
for ent in lines3d:
    if ent.dxftype() == "POLYLINE":
        try:
            n = len(list(ent.vertices))
        except Exception:
            n = 0
        vert_counts[n] += 1
for n in sorted(vert_counts):
    print(f"  {n} vertices: {vert_counts[n]} polylines")

# Compare 3D_LINES vs 3D_Lines (case)
print("\n\n=== Comparing FINISHED MODEL layer names with 3D prefix ===")
fm_layers = set(l.dxf.name for l in doc.layers)
print(f"  All layers with '3D' in name: {sorted(l for l in fm_layers if '3D' in l.upper())}")

# Check PLOTS FFL vs PLOTS EXTERNAL LEVEL – same polylines?
print("\n\n=== Are PLOTS FFL and PLOTS EXTERNAL LEVEL parallel (same plot boundaries)? ===")
ffl_ents   = [e for e in layer_ents.get("PLOTS FFL", []) if e.dxftype()=="POLYLINE"]
ext_ents   = [e for e in layer_ents.get("PLOTS EXTERNAL LEVEL", []) if e.dxftype()=="POLYLINE"]
print(f"  PLOTS FFL count:            {len(ffl_ents)}")
print(f"  PLOTS EXTERNAL LEVEL count: {len(ext_ents)}")

# For each FFL polyline – what is its constant Z (FFL value)?
print("\n  PLOTS FFL – unique Z per polyline (should be the FFL elevation):")
for i, ent in enumerate(ffl_ents):
    try:
        zs = sorted(set(round(v.dxf.location[2], 4) for v in ent.vertices))
    except Exception:
        zs = []
    try:
        vcount = len(list(ent.vertices))
    except Exception:
        vcount = 0
    closed = ent.is_closed
    print(f"    [{i:2d}] verts={vcount}  closed={closed}  Z values: {zs}")

print("\n  PLOTS EXTERNAL LEVEL – unique Z per polyline (ground level):")
for i, ent in enumerate(ext_ents):
    try:
        zs = sorted(set(round(v.dxf.location[2], 4) for v in ent.vertices))
    except Exception:
        zs = []
    try:
        vcount = len(list(ent.vertices))
    except Exception:
        vcount = 0
    closed = ent.is_closed
    print(f"    [{i:2d}] verts={vcount}  closed={closed}  Z values: {zs}")
