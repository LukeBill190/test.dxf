#!/usr/bin/env python3
"""
Detailed comparison of 3D_Lines (pipeline) vs 3D_LINES (reference).
- What do the zero-Z polylines in 3D_Lines represent?
- Are there truly 3D (non-zero Z) polylines in 3D_Lines?
- Why 500 vs 339?
"""
import ezdxf
from collections import defaultdict

def poly_summary(ent, idx=None):
    try:
        verts = list(ent.vertices)
    except Exception:
        verts = []
    z_vals = [v.dxf.location[2] for v in verts]
    has_nonzero = any(abs(z) > 1e-9 for z in z_vals)
    z_min = min(z_vals) if z_vals else 0
    z_max = max(z_vals) if z_vals else 0
    prefix = f"[{idx:3d}] " if idx is not None else ""
    return (f"{prefix}verts={len(verts):3d}  has_nonzero_Z={has_nonzero}  "
            f"Z=[{z_min:.4f},{z_max:.4f}]  closed={ent.is_closed}")

doc3 = ezdxf.readfile("/home/user/test.dxf/Test_3D_Output.dxf")
msp3 = doc3.modelspace()
layer3 = defaultdict(list)
for ent in msp3:
    layer3[ent.dxf.get("layer", "0")].append(ent)

docfm = ezdxf.readfile("/home/user/test.dxf/FINISHED MODEL.dxf")
mspfm = docfm.modelspace()
layerfm = defaultdict(list)
for ent in mspfm:
    layerfm[ent.dxf.get("layer", "0")].append(ent)

# ── 3D_Lines zero-Z analysis ────────────────────────────────────────────────────
print("=" * 70)
print("Test_3D_Output  3D_Lines – Z value classification")
print("=" * 70)

lines3d = layer3.get("3D_Lines", [])
zero_z = []
nonzero_z = []
for ent in lines3d:
    if ent.dxftype() == "POLYLINE":
        try:
            verts = list(ent.vertices)
        except Exception:
            verts = []
        z_vals = [v.dxf.location[2] for v in verts]
        if any(abs(z) > 1e-9 for z in z_vals):
            nonzero_z.append(ent)
        else:
            zero_z.append(ent)

print(f"\nTotal 3D_Lines: {len(lines3d)}")
print(f"  With ALL zero Z (flat/2D):  {len(zero_z)}")
print(f"  With non-zero Z (true 3D):  {len(nonzero_z)}")

print("\nSample of ZERO-Z polylines (first 10):")
for i, ent in enumerate(zero_z[:10]):
    try:
        verts = list(ent.vertices)
    except Exception:
        verts = []
    n = len(verts)
    # Get XY extent for clue about what it represents
    if verts:
        xs = [v.dxf.location[0] for v in verts]
        ys = [v.dxf.location[1] for v in verts]
        extent = f"X=[{min(xs):.1f},{max(xs):.1f}] Y=[{min(ys):.1f},{max(ys):.1f}]"
    else:
        extent = "no verts"
    print(f"  [{i:3d}] verts={n:3d}  closed={ent.is_closed}  {extent}")

print("\nSample of NON-ZERO-Z polylines (first 10):")
for i, ent in enumerate(nonzero_z[:10]):
    print("  " + poly_summary(ent, i))
    # show first 4 vertex Z
    try:
        verts = list(ent.vertices)
        zs = [round(v.dxf.location[2], 4) for v in verts[:6]]
        print(f"    first-6 vertex Z: {zs}")
    except Exception:
        pass

# ── FINISHED MODEL 3D_LINES analysis ───────────────────────────────────────────
print("\n\n" + "=" * 70)
print("FINISHED MODEL  3D_LINES – Z value classification")
print("=" * 70)

fm_lines = layerfm.get("3D_LINES", [])
fm_zero  = []
fm_nonzero = []
for ent in fm_lines:
    if ent.dxftype() == "POLYLINE":
        try:
            verts = list(ent.vertices)
        except Exception:
            verts = []
        z_vals = [v.dxf.location[2] for v in verts]
        if any(abs(z) > 1e-9 for z in z_vals):
            fm_nonzero.append(ent)
        else:
            fm_zero.append(ent)

print(f"\nTotal 3D_LINES: {len(fm_lines)}")
print(f"  With ALL zero Z (flat/2D):  {len(fm_zero)}")
print(f"  With non-zero Z (true 3D):  {len(fm_nonzero)}")

print("\nSample of NON-ZERO-Z polylines in FINISHED MODEL (first 10):")
for i, ent in enumerate(fm_nonzero[:10]):
    print("  " + poly_summary(ent, i))
    try:
        verts = list(ent.vertices)
        zs = [round(v.dxf.location[2], 4) for v in verts[:6]]
        print(f"    first-6 vertex Z: {zs}")
    except Exception:
        pass

# ── 3D_Building_Pads classification ─────────────────────────────────────────────
print("\n\n" + "=" * 70)
print("Test_3D_Output  3D_Building_Pads – full listing")
print("=" * 70)

pads = layer3.get("3D_Building_Pads", [])
print(f"Total: {len(pads)}")

# Group by Z value
by_z = defaultdict(list)
for ent in pads:
    if ent.dxftype() == "POLYLINE":
        try:
            verts = list(ent.vertices)
        except Exception:
            verts = []
        z_vals = [v.dxf.location[2] for v in verts]
        unique_z = tuple(sorted(set(round(z, 4) for z in z_vals)))
        by_z[unique_z].append((ent, len(verts)))

print("\nGrouped by Z profile (FFL value):")
for z_key in sorted(by_z.keys()):
    entries = by_z[z_key]
    vert_counts = [e[1] for e in entries]
    print(f"  Z={z_key}  count={len(entries)}  vert_counts={sorted(vert_counts)}")

print("\nAll 75 building pads:")
for i, ent in enumerate(pads):
    if ent.dxftype() == "POLYLINE":
        try:
            verts = list(ent.vertices)
        except Exception:
            verts = []
        z_vals = [v.dxf.location[2] for v in verts]
        unique_z = sorted(set(round(z, 4) for z in z_vals))
        n = len(verts)
        shape = "rect" if n == 5 else f"complex({n}v)"
        print(f"  [{i:2d}] {shape:15s}  Z={unique_z}")

# ── FFL count in source ─────────────────────────────────────────────────────────
print("\n\n" + "=" * 70)
print("Test.dxf (source)  LR LLFA FFL – FFL annotations")
print("=" * 70)
docsrc = ezdxf.readfile("/home/user/test.dxf/Test.dxf")
mspsrc = docsrc.modelspace()
layersrc = defaultdict(list)
for ent in mspsrc:
    layersrc[ent.dxf.get("layer", "0")].append(ent)

ffl_src = layersrc.get("LR LLFA FFL", [])
print(f"Total LR LLFA FFL entities: {len(ffl_src)}")
for i, ent in enumerate(ffl_src):
    etype = ent.dxftype()
    if etype == "MTEXT":
        text = ent.text[:40] if hasattr(ent, 'text') else "(no text attr)"
        ins = ent.dxf.insert
        print(f"  [{i:2d}] MTEXT  pos=({ins[0]:.1f},{ins[1]:.1f})  text={text!r}")
    else:
        print(f"  [{i:2d}] {etype}")

print("\n\nSame in Test_3D_Output:")
ffl_t3 = layer3.get("LR LLFA FFL", [])
print(f"Total LR LLFA FFL entities: {len(ffl_t3)}")
for i, ent in enumerate(ffl_t3[:10]):
    etype = ent.dxftype()
    if etype == "MTEXT":
        text = ent.text[:40] if hasattr(ent, 'text') else ""
        ins = ent.dxf.insert
        print(f"  [{i:2d}] MTEXT  pos=({ins[0]:.1f},{ins[1]:.1f})  text={text!r}")
