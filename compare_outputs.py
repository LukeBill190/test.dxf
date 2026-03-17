#!/usr/bin/env python3
"""
Compare pipeline output against reference DXF.

Full vertex-coordinate comparison across all geometry layers:
  PLOTS FFL, PLOTS EXTERNAL LEVEL, 3D_LINES

For each layer, every (x,y,z) vertex from the reference is matched to the
nearest output vertex in XY space. XY displacement and |ΔZ| are reported.

Goal: XY displacement ≈ 0 mm, |ΔZ| ≈ 0 mm  (coordinate lists should be identical).

Usage:
    python compare_outputs.py [output.dxf [reference.dxf]]
"""
import math
import sys
import ezdxf

OUTPUT = "/home/user/test.dxf/Test_v3_Output.dxf"
REF    = "/home/user/test.dxf/FINISHED MODEL.dxf"

# ── Thresholds ────────────────────────────────────────────────────────────────
XY_OK_M    = 0.010   # ≤ 10 mm  → OK
XY_WARN_M  = 0.100   # ≤ 100 mm → WARN  (> 100 mm → FAIL)

Z_OK_M     = 0.010   # ≤ 10 mm  → OK
Z_WARN_M   = 0.050   # ≤ 50 mm  → WARN  (> 50 mm  → FAIL)

XY_SNAP_M  = 2.0     # max XY distance to consider a vertex "matched"

LAYERS_TO_COMPARE = ["PLOTS FFL", "PLOTS EXTERNAL LEVEL", "3D_LINES"]


# ── DXF vertex extraction ─────────────────────────────────────────────────────

def _iter_entity_verts_xyz(ent):
    """Yield (x, y, z) for every vertex of a supported entity type."""
    t = ent.dxftype()
    if t == "POLYLINE":
        for v in ent.vertices:
            loc = v.dxf.location
            yield (loc.x, loc.y, loc.z)
    elif t == "LWPOLYLINE":
        elev = ent.dxf.get("elevation", 0.0)
        for x, y, *_ in ent.get_points():
            yield (x, y, elev)
    elif t == "LINE":
        s = ent.dxf.start
        e = ent.dxf.end
        yield (s.x, s.y, getattr(s, "z", 0.0))
        yield (e.x, e.y, getattr(e, "z", 0.0))


def read_layer_verts_xyz(dxf_path, layer_name):
    """Return flat list of (x, y, z) for all vertices on *layer_name*."""
    doc = ezdxf.readfile(dxf_path)
    msp = doc.modelspace()
    result = []
    for ent in msp:
        if ent.dxf.layer.upper() != layer_name.upper():
            continue
        result.extend(_iter_entity_verts_xyz(ent))
    return result


def read_layer_polylines_xyz(dxf_path, layer_name):
    """Return list of lists of (x, y, z) for each polyline on *layer_name*."""
    doc = ezdxf.readfile(dxf_path)
    msp = doc.modelspace()
    result = []
    for ent in msp:
        if ent.dxf.layer.upper() != layer_name.upper():
            continue
        verts = list(_iter_entity_verts_xyz(ent))
        if len(verts) >= 2:
            result.append(verts)
    return result


# ── Spatial grid for fast XY nearest-neighbour ────────────────────────────────

def build_grid(verts_xyz, cell=1.0):
    grid = {}
    for i, (x, y, z) in enumerate(verts_xyz):
        key = (int(math.floor(x / cell)), int(math.floor(y / cell)))
        grid.setdefault(key, []).append(i)
    return grid, cell


def nearest_in_grid(qx, qy, verts_xyz, grid, cell, max_d):
    """Return (dist_xy, index) for nearest vertex within max_d of (qx, qy)."""
    cx = int(math.floor(qx / cell))
    cy = int(math.floor(qy / cell))
    r  = int(math.ceil(max_d / cell)) + 1
    best_d, best_i = max_d, None
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            for i in grid.get((cx + dx, cy + dy), []):
                ox, oy, _oz = verts_xyz[i]
                d = math.sqrt((qx - ox) ** 2 + (qy - oy) ** 2)
                if d < best_d:
                    best_d = d
                    best_i = i
    return best_d, best_i


# ── Per-layer comparison ──────────────────────────────────────────────────────

def compare_layer(out_path, ref_path, layer_name):
    """Compare all vertex coordinates for a single layer."""
    print("=" * 70)
    print(f"  LAYER: {layer_name}")
    print(f"  XY  OK ≤{XY_OK_M*1000:.0f}mm  WARN ≤{XY_WARN_M*1000:.0f}mm  FAIL >{XY_WARN_M*1000:.0f}mm")
    print(f"  Z   OK ≤{Z_OK_M*1000:.0f}mm   WARN ≤{Z_WARN_M*1000:.0f}mm  FAIL >{Z_WARN_M*1000:.0f}mm")
    print("=" * 70)

    out_verts = read_layer_verts_xyz(out_path, layer_name)
    ref_verts = read_layer_verts_xyz(ref_path, layer_name)

    out_polys = read_layer_polylines_xyz(out_path, layer_name)
    ref_polys = read_layer_polylines_xyz(ref_path, layer_name)

    print(f"  Output: {len(out_polys)} polylines, {len(out_verts)} vertices")
    print(f"  Ref:    {len(ref_polys)} polylines, {len(ref_verts)} vertices\n")

    if not out_verts or not ref_verts:
        print("  [WARN] No vertices in one or both files — skipping.\n")
        return

    grid, cell = build_grid(out_verts, cell=max(0.5, XY_SNAP_M / 4))

    xy_errors = []
    z_errors  = []
    unmatched = 0
    matched_rows = []  # (xy_err, z_err, rx, ry, rz, ox, oy, oz)

    for rx, ry, rz in ref_verts:
        d_xy, best_i = nearest_in_grid(rx, ry, out_verts, grid, cell, XY_SNAP_M)
        if best_i is not None:
            ox, oy, oz = out_verts[best_i]
            dz = abs(rz - oz)
            xy_errors.append(d_xy)
            z_errors.append(dz)
            matched_rows.append((d_xy, dz, rx, ry, rz, ox, oy, oz))
        else:
            unmatched += 1

    if not xy_errors:
        print("  [WARN] No vertex pairs found within snap radius.\n")
        return

    n = len(xy_errors)

    # XY summary
    xy_ok   = sum(1 for e in xy_errors if e <= XY_OK_M)
    xy_warn = sum(1 for e in xy_errors if XY_OK_M < e <= XY_WARN_M)
    xy_fail = sum(1 for e in xy_errors if e > XY_WARN_M)

    # Z summary
    z_ok   = sum(1 for e in z_errors if e <= Z_OK_M)
    z_warn = sum(1 for e in z_errors if Z_OK_M < e <= Z_WARN_M)
    z_fail = sum(1 for e in z_errors if e > Z_WARN_M)

    print(f"  Matched: {n}   Unmatched (>{XY_SNAP_M*1000:.0f}mm XY): {unmatched}")
    print()
    print(f"  XY accuracy:")
    print(f"    Mean {sum(xy_errors)/n*1000:.2f} mm   Median {sorted(xy_errors)[n//2]*1000:.2f} mm   Max {max(xy_errors)*1000:.2f} mm")
    print(f"    OK  : {xy_ok:4d} ({100*xy_ok/n:5.1f}%)")
    print(f"    WARN: {xy_warn:4d} ({100*xy_warn/n:5.1f}%)")
    print(f"    FAIL: {xy_fail:4d} ({100*xy_fail/n:5.1f}%)")
    print()
    print(f"  Z accuracy:")
    print(f"    Mean {sum(z_errors)/n*1000:.2f} mm   Median {sorted(z_errors)[n//2]*1000:.2f} mm   Max {max(z_errors)*1000:.2f} mm")
    print(f"    OK  : {z_ok:4d} ({100*z_ok/n:5.1f}%)")
    print(f"    WARN: {z_warn:4d} ({100*z_warn/n:5.1f}%)")
    print(f"    FAIL: {z_fail:4d} ({100*z_fail/n:5.1f}%)")

    # Worst mismatches sorted by combined XY+Z error
    worst = sorted(matched_rows, key=lambda r: r[0] + r[1], reverse=True)
    print(f"\n  Worst 20 vertex mismatches (sorted by XY+Z error):")
    print(f"  {'Ref XY':>24}  Ref_Z     XY_err    Z_err   Out_Z")
    for xy_e, z_e, rx, ry, rz, ox, oy, oz in worst[:20]:
        print(f"  ({rx:10.3f},{ry:10.3f})  {rz:7.3f}   {xy_e*1000:7.2f}mm  {z_e*1000:7.2f}mm  {oz:7.3f}")
    print()

    # Show summary of unmatched reference vertices
    if unmatched > 0:
        print(f"  {unmatched} reference vertices had NO output vertex within {XY_SNAP_M*1000:.0f}mm XY.")
        unmatched_verts = []
        for rx, ry, rz in ref_verts:
            d_xy, best_i = nearest_in_grid(rx, ry, out_verts, grid, cell, XY_SNAP_M)
            if best_i is None:
                unmatched_verts.append((rx, ry, rz))
        print(f"  First 10 unmatched ref vertices:")
        for rx, ry, rz in unmatched_verts[:10]:
            print(f"    ({rx:.3f}, {ry:.3f}, {rz:.3f})")
        print()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    out = sys.argv[1] if len(sys.argv) > 1 else OUTPUT
    ref = sys.argv[2] if len(sys.argv) > 2 else REF
    print(f"\nOutput:    {out}")
    print(f"Reference: {ref}\n")
    for layer in LAYERS_TO_COMPARE:
        compare_layer(out, ref, layer)


if __name__ == "__main__":
    main()
