#!/usr/bin/env python3
"""
Investigate:
1. How fences/walls are represented (LINE vs POLYLINE, any Z data?)
2. How drives/roads are handled (A-STEN-ROAD, MVK layers)
3. What MVK layers in FINISHED MODEL contain (even if empty in layer table)
4. The 11 zero-Z 3D_Lines – what geometry are they?
"""
import ezdxf
from collections import defaultdict

def poly_z_summary(ent):
    try:
        verts = list(ent.vertices)
    except Exception:
        return "no-verts"
    z_vals = [v.dxf.location[2] for v in verts]
    unique_z = sorted(set(round(z, 4) for z in z_vals))
    nonzero = sum(1 for z in z_vals if abs(z) > 1e-9)
    return (f"verts={len(verts)} nonzero_z={nonzero} "
            f"z=[{min(z_vals):.4f},{max(z_vals):.4f}] uniq={unique_z[:6]}")

docfm = ezdxf.readfile("/home/user/test.dxf/FINISHED MODEL.dxf")
mspfm = docfm.modelspace()
doc3  = ezdxf.readfile("/home/user/test.dxf/Test_3D_Output.dxf")
msp3  = doc3.modelspace()

layerfm = defaultdict(list)
for ent in mspfm:
    layerfm[ent.dxf.get("layer", "0")].append(ent)

layer3 = defaultdict(list)
for ent in msp3:
    layer3[ent.dxf.get("layer", "0")].append(ent)

# ── MVK layers in FINISHED MODEL ──────────────────────────────────────────────
print("=" * 70)
print("FINISHED MODEL – MVK layers (layer table vs entity count)")
print("=" * 70)
for layer in sorted(l.dxf.name for l in docfm.layers):
    if 'MVK' in layer or 'Radius' in layer:
        cnt = len(layerfm.get(layer, []))
        print(f"  {layer:45s} -> {cnt} entities in modelspace")

# ── A-STEN-BOUNDARY layers in FINISHED MODEL ──────────────────────────────────
print("\n" + "=" * 70)
print("FINISHED MODEL – A-STEN-BOUNDARY layers")
print("=" * 70)
for layer in sorted(l.dxf.name for l in docfm.layers):
    if 'A-STEN' in layer:
        cnt = len(layerfm.get(layer, []))
        print(f"  {layer:45s} -> {cnt} entities")

# ── Fence / wall 3D representation in FINISHED MODEL ─────────────────────────
print("\n" + "=" * 70)
print("FINISHED MODEL – How fences are in 3D_LINES?")
print("  (Looking for polylines that might be fence representations)")
print("=" * 70)

lines_fm = layerfm.get("3D_LINES", [])
# Classify by Z range size (small range = flat feature, large = sloped)
flat_range = []  # Z max-min < 0.05
sloped_range = []  # Z max-min >= 0.05

for ent in lines_fm:
    if ent.dxftype() == "POLYLINE":
        try:
            verts = list(ent.vertices)
        except Exception:
            verts = []
        z_vals = [v.dxf.location[2] for v in verts]
        if z_vals:
            zrange = max(z_vals) - min(z_vals)
            if zrange < 0.05:
                flat_range.append((ent, min(z_vals), zrange))
            else:
                sloped_range.append((ent, min(z_vals), max(z_vals), zrange))

print(f"\n3D_LINES polylines with narrow Z range (<0.05m): {len(flat_range)}")
print(f"3D_LINES polylines with varying Z (>=0.05m):     {len(sloped_range)}")

print("\nSample of flat Z polylines (first 8):")
for ent, zmin, zrange in flat_range[:8]:
    try:
        verts = list(ent.vertices)
    except Exception:
        verts = []
    n = len(verts)
    print(f"  verts={n:3d}  Z={zmin:.4f}  dZ={zrange:.4f}  closed={ent.is_closed}")

print("\nSample of sloped Z polylines (first 10):")
for ent, zmin, zmax, zrange in sloped_range[:10]:
    try:
        verts = list(ent.vertices)
    except Exception:
        verts = []
    n = len(verts)
    print(f"  verts={n:3d}  Z=[{zmin:.4f},{zmax:.4f}]  dZ={zrange:.4f}  closed={ent.is_closed}")

# ── 3D_Lines zero-Z polylines in Test_3D_Output – what are they? ──────────────
print("\n" + "=" * 70)
print("Test_3D_Output  3D_Lines – zero-Z polylines (pass-through source?)")
print("=" * 70)
lines3d = layer3.get("3D_Lines", [])
zero_z_polys = []
for ent in lines3d:
    if ent.dxftype() == "POLYLINE":
        try:
            verts = list(ent.vertices)
        except Exception:
            verts = []
        z_vals = [v.dxf.location[2] for v in verts]
        if z_vals and not any(abs(z) > 1e-9 for z in z_vals):
            zero_z_polys.append(ent)

print(f"\nTotal zero-Z polylines: {len(zero_z_polys)}")
for i, ent in enumerate(zero_z_polys):
    try:
        verts = list(ent.vertices)
    except Exception:
        verts = []
    n = len(verts)
    if verts:
        xs = [v.dxf.location[0] for v in verts]
        ys = [v.dxf.location[1] for v in verts]
        print(f"  [{i:2d}] verts={n:4d}  closed={ent.is_closed}  "
              f"X=[{min(xs):.0f},{max(xs):.0f}] Y=[{min(ys):.0f},{max(ys):.0f}]")

# ── Road / drive in FINISHED MODEL 3D_LINES ───────────────────────────────────
print("\n" + "=" * 70)
print("FINISHED MODEL 3D_LINES vertex count distribution")
print("=" * 70)
from collections import Counter
vc = Counter()
for ent in lines_fm:
    if ent.dxftype() == "POLYLINE":
        try:
            n = len(list(ent.vertices))
        except Exception:
            n = 0
        vc[n] += 1
for n in sorted(vc):
    print(f"  {n:4d} vertices:  {vc[n]:3d} polylines")

# ── Spot check: fence LINEs in Test_3D_Output, do they have extrusion/Z? ────────
print("\n" + "=" * 70)
print("Test_3D_Output fence LINE Z inspection (first 15 fence lines)")
print("=" * 70)
fence_lines = [e for e in layer3.get("A-STEN-BOUNDARY SCREEN FENCE", [])
               if e.dxftype() == "LINE"]
for i, ent in enumerate(fence_lines[:15]):
    s = ent.dxf.start
    e2 = ent.dxf.end
    print(f"  [{i:2d}] start=({s[0]:.1f},{s[1]:.1f},{s[2]:.4f})  "
          f"end=({e2[0]:.1f},{e2[1]:.1f},{e2[2]:.4f})")
