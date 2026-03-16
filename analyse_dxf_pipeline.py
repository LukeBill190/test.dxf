#!/usr/bin/env python3
"""
Deep analysis of Test_3D_Output.dxf pipeline layers:
- 3D_Building_Pads
- 3D_Lines
- 3D_Points
- A-STEN-BOUNDARY SCREEN FENCE / SCREEN WALL
- A-STEN-ROAD / PATH layers
+ compare with what FINISHED MODEL expects
"""
import ezdxf
from collections import defaultdict

def analyse_polyline(ent, index=None):
    flags = ent.dxf.get("flags", 0)
    is_3d = bool(flags & ezdxf.lldxf.const.POLYLINE_3D_POLYLINE)
    try:
        verts = list(ent.vertices)
    except Exception:
        verts = []
    z_vals = []
    for v in verts:
        try:
            z_vals.append(v.dxf.location[2])
        except Exception:
            pass
    unique_z = sorted(set(round(z, 4) for z in z_vals))
    prefix = f"[{index}] " if index is not None else ""
    return (f"{prefix}POLYLINE 3d={is_3d} closed={ent.is_closed} "
            f"verts={len(verts)} "
            f"Z=[{min(z_vals) if z_vals else 'N/A':.4f},{max(z_vals) if z_vals else 'N/A':.4f}] "
            f"unique_z={unique_z[:6]}")

# ── Test_3D_Output deep dive ────────────────────────────────────────────────────
print("=" * 70)
print("Test_3D_Output.dxf – PIPELINE OUTPUT LAYERS")
print("=" * 70)

doc3 = ezdxf.readfile("/home/user/test.dxf/Test_3D_Output.dxf")
msp3 = doc3.modelspace()
layer3 = defaultdict(list)
for ent in msp3:
    layer3[ent.dxf.get("layer", "0")].append(ent)

# 3D_Building_Pads
print("\n--- 3D_Building_Pads ---")
pads = layer3.get("3D_Building_Pads", [])
print(f"Total: {len(pads)}")
for i, ent in enumerate(pads):
    if ent.dxftype() == "POLYLINE":
        print("  " + analyse_polyline(ent, i))
        if i >= 9:
            print(f"  ... ({len(pads)-10} more)")
            break

# 3D_Lines
print("\n--- 3D_Lines ---")
lines3d = layer3.get("3D_Lines", [])
print(f"Total: {len(lines3d)}")
type_counts = defaultdict(int)
for ent in lines3d:
    type_counts[ent.dxftype()] += 1
print(f"  Types: {dict(type_counts)}")
for i, ent in enumerate(lines3d[:5]):
    if ent.dxftype() == "POLYLINE":
        print("  " + analyse_polyline(ent, i))

# 3D_Points
print("\n--- 3D_Points ---")
pts = layer3.get("3D_Points", [])
print(f"Total: {len(pts)}")
type_counts = defaultdict(int)
for ent in pts:
    type_counts[ent.dxftype()] += 1
print(f"  Types: {dict(type_counts)}")
z_vals = []
for ent in pts:
    if ent.dxftype() == "POINT":
        z_vals.append(ent.dxf.location[2])
if z_vals:
    print(f"  Z range: [{min(z_vals):.4f}, {max(z_vals):.4f}]")
    unique_z = sorted(set(round(z, 4) for z in z_vals))
    print(f"  Unique Z count: {len(unique_z)}")
    print(f"  First 10 unique: {unique_z[:10]}")

# Fences and walls
for layer_name in ["A-STEN-BOUNDARY SCREEN FENCE", "A-STEN-BOUNDARY SCREEN WALL"]:
    ents = layer3.get(layer_name, [])
    print(f"\n--- {layer_name} ---")
    print(f"Total: {len(ents)}")
    type_counts = defaultdict(int)
    for ent in ents:
        type_counts[ent.dxftype()] += 1
    print(f"  Types: {dict(type_counts)}")
    # Sample Z values from LINEs
    line_ents = [e for e in ents if e.dxftype() == "LINE"]
    if line_ents:
        z_starts = [e.dxf.start[2] for e in line_ents]
        z_ends = [e.dxf.end[2] for e in line_ents]
        all_z = z_starts + z_ends
        unique_z = sorted(set(round(z, 4) for z in all_z))
        nonzero = sum(1 for z in all_z if abs(z) > 1e-9)
        print(f"  LINE Z: non-zero={nonzero}/{len(all_z)}, unique={unique_z[:10]}")

# Road layers
for layer_name in ["A-STEN-ROAD", "A-STEN-ROAD CENTRE LINE", "A-STEN-ROAD FOOTPATH", "A-STEN-PATH"]:
    ents = layer3.get(layer_name, [])
    print(f"\n--- {layer_name} ---")
    print(f"Total: {len(ents)}")
    type_counts = defaultdict(int)
    for ent in ents:
        type_counts[ent.dxftype()] += 1
    print(f"  Types: {dict(type_counts)}")

# ── FINISHED MODEL deep dive ─────────────────────────────────────────────────
print("\n\n" + "=" * 70)
print("FINISHED MODEL.dxf – REFERENCE LAYERS")
print("=" * 70)

docfm = ezdxf.readfile("/home/user/test.dxf/FINISHED MODEL.dxf")
mspfm = docfm.modelspace()
layerfm = defaultdict(list)
for ent in mspfm:
    layerfm[ent.dxf.get("layer", "0")].append(ent)

# All layers with entities
print("\nAll layers with entities:")
for layer_name in sorted(layerfm.keys()):
    ents = layerfm[layer_name]
    type_counts = defaultdict(int)
    for ent in ents:
        type_counts[ent.dxftype()] += 1
    print(f"  {layer_name:45s} {len(ents):4d}  {dict(type_counts)}")

# 3D_LINES
print("\n--- 3D_LINES (FINISHED MODEL) ---")
lines3d_fm = layerfm.get("3D_LINES", [])
print(f"Total: {len(lines3d_fm)}")
for i, ent in enumerate(lines3d_fm[:5]):
    if ent.dxftype() == "POLYLINE":
        print("  " + analyse_polyline(ent, i))

# All Z values in 3D_LINES
all_z = []
for ent in lines3d_fm:
    if ent.dxftype() == "POLYLINE":
        for v in ent.vertices:
            try:
                all_z.append(v.dxf.location[2])
            except Exception:
                pass
if all_z:
    print(f"  Z range: [{min(all_z):.4f}, {max(all_z):.4f}]")
    unique_z = sorted(set(round(z, 4) for z in all_z))
    print(f"  Unique Z count: {len(unique_z)}")

# ── Count comparison: 3D_LINES vs 3D_Lines ─────────────────────────────────────
print("\n\n" + "=" * 70)
print("COUNT COMPARISON")
print("=" * 70)
print(f"FINISHED MODEL 3D_LINES:      {len(lines3d_fm)} POLYLINEs")
print(f"Test_3D_Output 3D_Lines:       {len(lines3d)} entities")
print(f"FINISHED MODEL PLOTS FFL:     {len(layerfm.get('PLOTS FFL', []))} POLYLINEs")
print(f"FINISHED MODEL PLOTS EXT LVL: {len(layerfm.get('PLOTS EXTERNAL LEVEL', []))} POLYLINEs")
print(f"Test_3D_Output 3D_Building_Pads: {len(pads)} entities")
print(f"Test_3D_Output 3D_Points:      {len(pts)} entities")

# Check FINISHED MODEL layers missing from Test_3D_Output
print("\n\nKey observations:")
fm_layer_names = set(l.dxf.name for l in docfm.layers)
t3_layer_names = set(l.dxf.name for l in doc3.layers)

print(f"\nFINISHED MODEL has these layers NOT in Test_3D_Output:")
for ln in sorted(fm_layer_names - t3_layer_names):
    cnt = len(layerfm.get(ln, []))
    print(f"  {ln} ({cnt} entities)")

print(f"\nFINISHED MODEL has these specific 3D layers:")
for ln in sorted(fm_layer_names):
    if any(x in ln.upper() for x in ['3D', 'FFL', 'PLOT', 'MVK', 'PAD']):
        cnt = len(layerfm.get(ln, []))
        print(f"  {ln} ({cnt} entities)")

print(f"\nTest_3D_Output has these specific 3D/output layers:")
for ln in sorted(t3_layer_names):
    if any(x in ln.upper() for x in ['3D', 'FFL', 'PLOT', 'MVK', 'PAD', 'BUILDING']):
        cnt = len(layer3.get(ln, []))
        print(f"  {ln} ({cnt} entities)")

# ── Merged/adjacent plots check ────────────────────────────────────────────────
print("\n\n" + "=" * 70)
print("MERGED PLOT ANALYSIS: PLOTS FFL in FINISHED MODEL")
print("=" * 70)
ffl_ents = [e for e in layerfm.get("PLOTS FFL", []) if e.dxftype()=="POLYLINE"]
print("Checking vertex counts – fewer verts relative to a simple rectangle suggests merging:")
for i, ent in enumerate(ffl_ents):
    try:
        verts = list(ent.vertices)
    except Exception:
        verts = []
    n = len(verts)
    z = verts[0].dxf.location[2] if verts else None
    # 5 verts = closed rectangle (4 corners + repeated first), >5 = L-shape or merged
    shape = "rect" if n == 5 else f"complex({n}verts)"
    print(f"  [{i:2d}] {shape:20s}  Z={z:.4f}" if z else f"  [{i:2d}] {shape}")
