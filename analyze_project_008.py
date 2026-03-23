#!/usr/bin/env python3
"""
Standalone analysis script: counts "owned" vertices at different Z tolerances
for project_008 DXF pair.
"""

import math
import re

import ezdxf

INPUT_DXF  = "/home/user/test.dxf/training_data/project_008/input.dxf"
OUTPUT_DXF = "/home/user/test.dxf/training_data/project_008/output.dxf"

OUTPUT_LAYER    = "3D_LINES"
OWNED_XY_RADIUS = 2.0   # metres

# Layer substring patterns (case-insensitive matching)
SPOT_LAYERS = ["SPOT LEVEL", "PROP-LEVELS", "EXIST-LEVELS", "PROPOSED LEVEL"]
DPC_LAYERS  = ["DPC LEVEL", "DPC"]
L018_LAYERS = ["L018 HA_ANN_FEAT_TEXT"]
FFL_LAYERS  = ["LLFA FFL", "FINISHED FLOOR", "FFL LEVEL", "FFL"]


def _parse_z(text):
    nums = re.findall(r'[+-]?\d{2,3}\.\d{1,4}', str(text))
    return float(nums[0]) if nums else None


def collect_input_annotations(msp):
    """Return list of (x, y, z, ann_type) for all elevation text, excluding FFL."""
    annotations = []
    for ent in msp:
        if ent.dxftype() not in ("TEXT", "MTEXT"):
            continue
        try:
            xy = (ent.dxf.insert.x, ent.dxf.insert.y)
        except Exception:
            continue
        try:
            txt = ent.dxf.text if ent.dxftype() == "TEXT" else ent.text
        except Exception:
            continue
        z = _parse_z(txt)
        if z is None:
            continue
        layer = ent.dxf.layer
        layer_up = layer.upper()
        if   any(s.upper() in layer_up for s in SPOT_LAYERS):  ann_type = "SPOT"
        elif any(s.upper() in layer_up for s in DPC_LAYERS):   ann_type = "DPC"
        elif any(s.upper() in layer_up for s in L018_LAYERS):  ann_type = "L018"
        elif any(s.upper() in layer_up for s in FFL_LAYERS):   ann_type = "FFL"
        else:
            continue
        annotations.append((xy[0], xy[1], z, ann_type))
    return annotations


def collect_output_verts(msp):
    """Return list of (x, y, z) from 3D_LINES POLYLINE and LWPOLYLINE entities."""
    verts = []
    for ent in msp:
        if ent.dxf.layer != OUTPUT_LAYER:
            continue
        if ent.dxftype() == "POLYLINE":
            for v in ent.vertices:
                try:
                    loc = v.dxf.location
                    verts.append((loc.x, loc.y, loc.z))
                except Exception:
                    pass
        elif ent.dxftype() == "LWPOLYLINE":
            # LWPOLYLINE stores points as (x, y, [start_width, end_width, bulge])
            # elevation is stored in ent.dxf.elevation (Z for all points)
            try:
                elev = ent.dxf.elevation
            except Exception:
                elev = 0.0
            for pt in ent.get_points():
                verts.append((pt[0], pt[1], elev))
    return verts


# ---------------------------------------------------------------------------
# Load DXFs
# ---------------------------------------------------------------------------
print("Loading DXFs...")
in_doc  = ezdxf.readfile(INPUT_DXF)
out_doc = ezdxf.readfile(OUTPUT_DXF)

in_msp  = in_doc.modelspace()
out_msp = out_doc.modelspace()

# --- Collect terrain annotations from input ---
all_ann     = collect_input_annotations(in_msp)
non_ffl_ann = [a for a in all_ann if a[3] != "FFL"]

print(f"Total annotations:         {len(all_ann)}")
print(f"  SPOT:  {sum(1 for a in all_ann if a[3]=='SPOT')}")
print(f"  DPC:   {sum(1 for a in all_ann if a[3]=='DPC')}")
print(f"  L018:  {sum(1 for a in all_ann if a[3]=='L018')}")
print(f"  FFL:   {sum(1 for a in all_ann if a[3]=='FFL')}")
print(f"Non-FFL annotations used:  {len(non_ffl_ann)}")

# --- Collect output vertices ---
out_verts = collect_output_verts(out_msp)
print(f"Output 3D_LINES vertices:  {len(out_verts)}")
print()

# ---------------------------------------------------------------------------
# For each vertex: find minimum Z diff among annotations within XY radius
# ---------------------------------------------------------------------------
# min_z_diff_list[i] = minimum |az - vz| for annotations within OWNED_XY_RADIUS
# (or None if no annotation is within XY radius)

min_z_diff_list = []

for vx, vy, vz in out_verts:
    best_z_diff = None
    for ax, ay, az, _ in non_ffl_ann:
        xy_dist = math.sqrt((ax - vx) ** 2 + (ay - vy) ** 2)
        if xy_dist <= OWNED_XY_RADIUS:
            z_diff = abs(az - vz)
            if best_z_diff is None or z_diff < best_z_diff:
                best_z_diff = z_diff
    min_z_diff_list.append(best_z_diff)

verts_with_nearby_ann = sum(1 for d in min_z_diff_list if d is not None)
print(f"Vertices with annotation within {OWNED_XY_RADIUS}m XY: {verts_with_nearby_ann} / {len(out_verts)}")
print()

# ---------------------------------------------------------------------------
# Table: owned count at each Z tolerance
# ---------------------------------------------------------------------------
z_tols = [0.012, 0.020, 0.025, 0.030, 0.040, 0.050, 0.075, 0.100]
total  = len(out_verts)

print(f"{'Z_tol_mm':>10} | {'owned_count':>12} | {'% of total verts':>18}")
print("-" * 48)
for zt in z_tols:
    owned = sum(1 for d in min_z_diff_list if d is not None and d <= zt)
    pct   = 100.0 * owned / total if total > 0 else 0.0
    print(f"{zt*1000:>10.1f} | {owned:>12} | {pct:>17.1f}%")

print()

# ---------------------------------------------------------------------------
# Distribution of minimum Z differences
# ---------------------------------------------------------------------------
# Only for vertices that DO have a nearby annotation
diffs_mm = [d * 1000 for d in min_z_diff_list if d is not None]

bins = [
    ("0–12 mm",    0,    12),
    ("12–25 mm",  12,    25),
    ("25–50 mm",  25,    50),
    ("50–100 mm", 50,   100),
    ("100 mm+",  100, float('inf')),
]

print("Distribution of min Z-difference (for vertices within 2.0m XY of any non-FFL annotation):")
print(f"  Total vertices with nearby annotation: {len(diffs_mm)}")
print()
print(f"  {'Range':>12} | {'count':>8} | {'%':>8}")
print("  " + "-" * 36)
for label, lo, hi in bins:
    cnt = sum(1 for d in diffs_mm if lo <= d < hi)
    pct = 100.0 * cnt / len(diffs_mm) if diffs_mm else 0.0
    print(f"  {label:>12} | {cnt:>8} | {pct:>7.1f}%")

# Also show some percentile stats
if diffs_mm:
    diffs_sorted = sorted(diffs_mm)
    n = len(diffs_sorted)
    def pct_val(p):
        idx = int(p / 100.0 * n)
        return diffs_sorted[min(idx, n-1)]
    print()
    print("  Percentiles of min Z-diff (mm):")
    for p in [25, 50, 75, 90, 95, 99]:
        print(f"    P{p:>2}: {pct_val(p):>8.2f} mm")
    print(f"    Max: {diffs_sorted[-1]:>8.2f} mm")
