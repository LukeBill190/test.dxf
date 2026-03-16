#!/usr/bin/env python3
"""
Compare pipeline output against reference DXF.

Two assessments are performed:
  1. PLOTS FFL  — pad centroid XY accuracy (tighter tolerance than previous runs)
  2. 3D_LINES   — fence/driveway/road Z elevation accuracy using vertex-cloud
                  nearest-neighbour matching in XY

Usage:
    python compare_outputs.py [output.dxf [reference.dxf]]
"""
import math
import sys
import ezdxf
from shapely.geometry import Polygon

OUTPUT = "/home/user/test.dxf/Test_v3_Output.dxf"
REF    = "/home/user/test.dxf/FINISHED MODEL.dxf"

# ── Thresholds ────────────────────────────────────────────────────────────────
# PLOTS FFL centroid distance
PAD_OK_M   = 0.50   # ≤ 500 mm  → OK
PAD_WARN_M = 1.50   # ≤ 1500 mm → WARN  (> 1500 mm → FAIL)

# 3D_LINES vertex matching
LINES_XY_M     = 0.50   # XY snap radius for vertex pairing (m)
LINES_Z_OK_M   = 0.050  # |ΔZ| ≤ 50 mm  → OK
LINES_Z_WARN_M = 0.150  # |ΔZ| ≤ 150 mm → WARN  (> 150 mm → FAIL)


# ── DXF reading helpers ───────────────────────────────────────────────────────

def _iter_polyline_verts_xy(ent):
    """Yield (x, y) pairs from a POLYLINE or LWPOLYLINE entity."""
    t = ent.dxftype()
    if t == "POLYLINE":
        for v in ent.vertices:
            loc = v.dxf.location
            yield (loc.x, loc.y)
    elif t == "LWPOLYLINE":
        for x, y, *_ in ent.get_points():
            yield (x, y)


def _iter_polyline_verts_xyz(ent):
    """Yield (x, y, z) tuples from a POLYLINE or LWPOLYLINE entity."""
    t = ent.dxftype()
    if t == "POLYLINE":
        for v in ent.vertices:
            loc = v.dxf.location
            yield (loc.x, loc.y, loc.z)
    elif t == "LWPOLYLINE":
        elev = ent.dxf.get("elevation", 0.0)
        for x, y, *_ in ent.get_points():
            yield (x, y, elev)


def read_layer_polys_xy(dxf_path, layer_name):
    """Return list of [(x,y), ...] vertex lists for all polylines on layer."""
    doc = ezdxf.readfile(dxf_path)
    msp = doc.modelspace()
    result = []
    for ent in msp:
        if ent.dxf.layer.upper() != layer_name.upper():
            continue
        verts = list(_iter_polyline_verts_xy(ent))
        if len(verts) >= 2:
            result.append(verts)
    return result


def read_layer_polys_xyz(dxf_path, layer_name):
    """Return list of [(x,y,z), ...] vertex lists for all polylines on layer."""
    doc = ezdxf.readfile(dxf_path)
    msp = doc.modelspace()
    result = []
    for ent in msp:
        if ent.dxf.layer.upper() != layer_name.upper():
            continue
        verts = list(_iter_polyline_verts_xyz(ent))
        if len(verts) >= 2:
            result.append(verts)
    return result


def poly_area(verts_xy):
    try:
        p = Polygon(verts_xy)
        return p.area if p.is_valid else 0.0
    except Exception:
        return 0.0


def poly_centroid(verts_xy):
    try:
        p = Polygon(verts_xy)
        if p.is_valid and not p.is_empty:
            return (p.centroid.x, p.centroid.y)
    except Exception:
        pass
    n = len(verts_xy)
    return (sum(x for x, y in verts_xy) / n, sum(y for x, y in verts_xy) / n)


def dist2d(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


# ── Simple grid index for fast XY nearest-neighbour ──────────────────────────

def build_grid(verts_xyz, cell=1.0):
    """Build a dict grid for (x, y, z) vertex list indexed by grid cell."""
    grid = {}
    for i, (x, y, z) in enumerate(verts_xyz):
        key = (int(math.floor(x / cell)), int(math.floor(y / cell)))
        grid.setdefault(key, []).append(i)
    return grid, cell


def nearest_in_grid(qx, qy, verts_xyz, grid, cell, max_d):
    """Return (best_dist_xy, best_z) for the nearest vertex within max_d of (qx,qy)."""
    cx = int(math.floor(qx / cell))
    cy = int(math.floor(qy / cell))
    r  = int(math.ceil(max_d / cell)) + 1
    best_d, best_z = max_d, None
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            for i in grid.get((cx + dx, cy + dy), []):
                ox, oy, oz = verts_xyz[i]
                d = math.sqrt((qx - ox) ** 2 + (qy - oy) ** 2)
                if d < best_d:
                    best_d = d
                    best_z = oz
    return best_d, best_z


# ── Assessment 1: PLOTS FFL ───────────────────────────────────────────────────

def compare_plots_ffl(out_path, ref_path):
    print("=" * 64)
    print("  ASSESSMENT 1 — PLOTS FFL pad centroid accuracy")
    print(f"  OK ≤ {PAD_OK_M*1000:.0f} mm  |  WARN ≤ {PAD_WARN_M*1000:.0f} mm  |  FAIL > {PAD_WARN_M*1000:.0f} mm")
    print("=" * 64)

    out_pads = [v for v in read_layer_polys_xy(out_path, "PLOTS FFL") if len(v) >= 3]
    ref_pads = [v for v in read_layer_polys_xy(ref_path, "PLOTS FFL") if len(v) >= 3]

    print(f"  Output: {len(out_pads)} pads   Reference: {len(ref_pads)} pads\n")

    out_c = [poly_centroid(v) for v in out_pads]
    ref_c = [poly_centroid(v) for v in ref_pads]
    out_a = [poly_area(v) for v in out_pads]
    ref_a = [poly_area(v) for v in ref_pads]

    ok = warn = fail = 0
    errors = []

    for i, rc in enumerate(ref_c):
        best_j = min(range(len(out_c)), key=lambda j: dist2d(rc, out_c[j]))
        d = dist2d(rc, out_c[best_j])
        errors.append(d)
        if d <= PAD_OK_M:
            tag, ok = "OK  ", ok + 1
        elif d <= PAD_WARN_M:
            tag, warn = "WARN", warn + 1
        else:
            tag, fail = "FAIL", fail + 1
        print(f"  Ref[{i:2d}] ({rc[0]:.1f},{rc[1]:.1f}) {ref_a[i]:.1f} m²"
              f"  →  [{tag}] dist={d*1000:.0f} mm  out_area={out_a[best_j]:.1f} m²")

    print(f"\n  OK:{ok}  WARN:{warn}  FAIL:{fail}  total:{len(ref_c)}")
    if errors:
        print(f"  Mean error: {sum(errors)/len(errors)*1000:.0f} mm   Max: {max(errors)*1000:.0f} mm")

    # Spurious output pads
    spurious = [j for j, oc in enumerate(out_c)
                if min(dist2d(oc, rc) for rc in ref_c) > PAD_WARN_M]
    if spurious:
        print(f"\n  Spurious output pads (>{PAD_WARN_M*1000:.0f} mm from any ref):")
        for j in spurious:
            print(f"    Out[{j}] ({out_c[j][0]:.1f},{out_c[j][1]:.1f}) {out_a[j]:.1f} m²")
    print()


# ── Assessment 2: 3D_LINES ───────────────────────────────────────────────────

def compare_3d_lines(out_path, ref_path):
    print("=" * 64)
    print("  ASSESSMENT 2 — 3D_LINES (fences / driveways / roads) Z accuracy")
    print(f"  XY snap: {LINES_XY_M*1000:.0f} mm  |"
          f"  Z OK ≤ {LINES_Z_OK_M*1000:.0f} mm  |"
          f"  Z WARN ≤ {LINES_Z_WARN_M*1000:.0f} mm")
    print("=" * 64)

    out_polys = read_layer_polys_xyz(out_path, "3D_LINES")
    ref_polys = read_layer_polys_xyz(ref_path, "3D_LINES")

    out_verts = [(x, y, z) for poly in out_polys for x, y, z in poly]
    ref_verts = [(x, y, z) for poly in ref_polys for x, y, z in poly]

    print(f"  Output: {len(out_polys)} polylines, {len(out_verts)} vertices")
    print(f"  Ref:    {len(ref_polys)} polylines, {len(ref_verts)} vertices\n")

    if not out_verts or not ref_verts:
        print("  [WARN] One or both files have no 3D_LINES — skipping Z comparison.\n")
        return

    # Build spatial grid on output vertices for fast XY lookup
    grid, cell = build_grid(out_verts, cell=max(0.5, LINES_XY_M))

    z_errors  = []   # |ΔZ| for each matched reference vertex
    unmatched = 0    # ref vertices with no output vertex within XY snap

    for rx, ry, rz in ref_verts:
        _dxy, oz = nearest_in_grid(rx, ry, out_verts, grid, cell, LINES_XY_M)
        if oz is not None:
            z_errors.append(abs(rz - oz))
        else:
            unmatched += 1

    if not z_errors:
        print("  [WARN] No vertex pairs found within XY snap radius.\n")
        return

    n = len(z_errors)
    ok_n   = sum(1 for e in z_errors if e <= LINES_Z_OK_M)
    warn_n = sum(1 for e in z_errors if LINES_Z_OK_M < e <= LINES_Z_WARN_M)
    fail_n = sum(1 for e in z_errors if e > LINES_Z_WARN_M)
    mean_e = sum(z_errors) / n
    median_e = sorted(z_errors)[n // 2]

    print(f"  Matched vertices : {n}   Unmatched (>{LINES_XY_M*1000:.0f} mm XY): {unmatched}")
    print(f"  Mean  |ΔZ| : {mean_e*1000:6.1f} mm")
    print(f"  Median|ΔZ| : {median_e*1000:6.1f} mm")
    print(f"  Max   |ΔZ| : {max(z_errors)*1000:6.1f} mm")
    print(f"  OK  (≤{LINES_Z_OK_M*1000:.0f} mm)  : {ok_n:4d}  ({100*ok_n/n:5.1f}%)")
    print(f"  WARN (≤{LINES_Z_WARN_M*1000:.0f} mm) : {warn_n:4d}  ({100*warn_n/n:5.1f}%)")
    print(f"  FAIL (>{LINES_Z_WARN_M*1000:.0f} mm)  : {fail_n:4d}  ({100*fail_n/n:5.1f}%)")

    # Worst-20 Z errors for diagnosis
    paired = sorted(zip(z_errors, ref_verts), key=lambda x: x[0], reverse=True)
    print(f"\n  Worst 20 Z errors:")
    print(f"  {'XY position':>30}  ref_Z    |ΔZ|")
    for dz, (rx, ry, rz) in paired[:20]:
        print(f"  ({rx:10.2f}, {ry:10.2f})         {rz:7.3f}  {dz*1000:6.1f} mm")
    print()


# ── Assessment 3: PLOTS EXTERNAL LEVEL Z ─────────────────────────────────────

def compare_external_level(out_path, ref_path):
    print("=" * 64)
    print("  ASSESSMENT 3 — PLOTS EXTERNAL LEVEL Z accuracy (pad terrain)")
    print(f"  XY snap: 1000 mm  |  Z OK ≤ {LINES_Z_OK_M*1000:.0f} mm"
          f"  |  Z WARN ≤ {LINES_Z_WARN_M*1000:.0f} mm")
    print("=" * 64)

    out_polys = read_layer_polys_xyz(out_path, "PLOTS EXTERNAL LEVEL")
    ref_polys = read_layer_polys_xyz(ref_path, "PLOTS EXTERNAL LEVEL")

    print(f"  Output: {len(out_polys)} pads   Reference: {len(ref_polys)} pads\n")

    if not out_polys or not ref_polys:
        print("  [WARN] No EXTERNAL LEVEL polylines found in one or both files.\n")
        return

    # Build centroid index to match output → reference pads
    def centroid_xy(verts):
        xs = [x for x, y, z in verts]
        ys = [y for x, y, z in verts]
        return sum(xs) / len(xs), sum(ys) / len(ys)

    out_c = [centroid_xy(v) for v in out_polys]
    ref_c = [centroid_xy(v) for v in ref_polys]

    # Build spatial grid on output vertices for XY matching
    all_out_verts = [(x, y, z) for v in out_polys for x, y, z in v]
    grid, cell = build_grid(all_out_verts, cell=1.0)

    XY_SNAP = 1.0  # wider snap for pad boundary (shapes may differ slightly)
    z_errors = []
    unmatched = 0

    for ref_v in ref_polys:
        for rx, ry, rz in ref_v:
            _dxy, oz = nearest_in_grid(rx, ry, all_out_verts, grid, cell, XY_SNAP)
            if oz is not None:
                z_errors.append(abs(rz - oz))
            else:
                unmatched += 1

    if not z_errors:
        print("  [WARN] No vertex pairs found within XY snap.\n")
        return

    n = len(z_errors)
    ok_n   = sum(1 for e in z_errors if e <= LINES_Z_OK_M)
    warn_n = sum(1 for e in z_errors if LINES_Z_OK_M < e <= LINES_Z_WARN_M)
    fail_n = sum(1 for e in z_errors if e > LINES_Z_WARN_M)

    print(f"  Matched vertices : {n}   Unmatched (>1000 mm XY): {unmatched}")
    print(f"  Mean  |ΔZ| : {sum(z_errors)/n*1000:6.1f} mm")
    print(f"  Median|ΔZ| : {sorted(z_errors)[n//2]*1000:6.1f} mm")
    print(f"  Max   |ΔZ| : {max(z_errors)*1000:6.1f} mm")
    print(f"  OK  (≤{LINES_Z_OK_M*1000:.0f} mm)  : {ok_n:4d}  ({100*ok_n/n:5.1f}%)")
    print(f"  WARN (≤{LINES_Z_WARN_M*1000:.0f} mm) : {warn_n:4d}  ({100*warn_n/n:5.1f}%)")
    print(f"  FAIL (>{LINES_Z_WARN_M*1000:.0f} mm)  : {fail_n:4d}  ({100*fail_n/n:5.1f}%)")

    paired = sorted(zip(z_errors,
                        [(x, y, z) for v in ref_polys for x, y, z in v]),
                    key=lambda x: x[0], reverse=True)
    print(f"\n  Worst 10 EXTERNAL LEVEL Z errors:")
    print(f"  {'XY position':>30}  ref_Z    |ΔZ|")
    for dz, (rx, ry, rz) in paired[:10]:
        print(f"  ({rx:10.2f}, {ry:10.2f})         {rz:7.3f}  {dz*1000:6.1f} mm")
    print()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    out = sys.argv[1] if len(sys.argv) > 1 else OUTPUT
    ref = sys.argv[2] if len(sys.argv) > 2 else REF
    compare_plots_ffl(out, ref)
    compare_external_level(out, ref)
    compare_3d_lines(out, ref)


if __name__ == "__main__":
    main()
