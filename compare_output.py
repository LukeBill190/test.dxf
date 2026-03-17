#!/usr/bin/env python3
"""
DXF linework comparison.

For PLOTS FFL / PLOTS EXTERNAL LEVEL (pad layers): vertex-to-vertex 3D comparison.
For 3D_LINES (drive/fence/road linework): each ref vertex is projected onto the
  nearest output SEGMENT so that differences in vertex density don't cause false
  failures.  The 3D distance from the ref vertex to its foot-point on the segment
  must be ≤ TOLERANCE.

Usage: python compare_output.py [output.dxf [reference.dxf]]
"""

import math, sys, ezdxf
from collections import defaultdict

OUTPUT = "/home/user/test.dxf/Test_v4_Output.dxf"
REF    = "/home/user/test.dxf/FINISHED MODEL.dxf"
TOLERANCE = 0.010   # 10 mm


def get_polylines(doc, layer):
    polys = []
    for ent in doc.modelspace():
        if ent.dxf.layer != layer:
            continue
        dtype = ent.dxftype()
        if dtype == "POLYLINE":
            # 3D polyline (add_polyline3d) or 2D polyline
            try:
                verts = [(v.dxf.location.x, v.dxf.location.y, v.dxf.location.z)
                         for v in ent.vertices]
            except Exception:
                verts = []
        elif dtype == "LWPOLYLINE":
            elev = ent.dxf.get("elevation", 0.0)
            try:
                verts = [(x, y, z or elev) for x, y, z, *_ in ent.get_points(format="xyzb")]
            except Exception:
                verts = [(x, y, elev) for x, y in ent.get_points()]
        elif dtype == "LINE":
            s = ent.dxf.start; e = ent.dxf.end
            verts = [(s.x, s.y, s.z), (e.x, e.y, e.z)]
        else:
            continue
        if verts:
            polys.append(verts)
    return polys


def build_3d_grid(verts, cell=0.5):
    grid = {}
    for i, (x, y, z) in enumerate(verts):
        k = (int(x // cell), int(y // cell), int(z // cell))
        grid.setdefault(k, []).append(i)
    return grid, cell


def nearest_3d(qx, qy, qz, verts, grid, cell, max_d):
    cx = int(qx // cell); cy = int(qy // cell); cz = int(qz // cell)
    r = int(math.ceil(max_d / cell)) + 1
    best = max_d; bi = None
    for dx in range(-r, r+1):
        for dy in range(-r, r+1):
            for dz in range(-r, r+1):
                for idx in grid.get((cx+dx, cy+dy, cz+dz), []):
                    ox, oy, oz = verts[idx]
                    d = math.sqrt((qx-ox)**2+(qy-oy)**2+(qz-oz)**2)
                    if d < best:
                        best = d; bi = idx
    return best, bi


def pt_to_seg_3d(px, py, pz, ax, ay, az, bx, by, bz):
    """
    3D distance from point P to segment A–B.
    Returns (distance, foot_x, foot_y, foot_z).
    """
    dx, dy, dz = bx - ax, by - ay, bz - az
    len_sq = dx*dx + dy*dy + dz*dz
    if len_sq < 1e-18:
        d = math.sqrt((px-ax)**2 + (py-ay)**2 + (pz-az)**2)
        return d, ax, ay, az
    t = max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy + (pz-az)*dz) / len_sq))
    fx, fy, fz = ax + t*dx, ay + t*dy, az + t*dz
    d = math.sqrt((px-fx)**2 + (py-fy)**2 + (pz-fz)**2)
    return d, fx, fy, fz


def compare_layer(out_polys, ref_polys, use_segment_projection=False):
    out_verts = [(x, y, z) for p in out_polys for x, y, z in p]
    ref_verts = [(x, y, z) for p in ref_polys for x, y, z in p]
    ref_poly_idx = [pi for pi, p in enumerate(ref_polys) for _ in p]

    if not out_verts or not ref_verts:
        return out_verts, ref_verts, []

    if use_segment_projection:
        # For linework: project each ref vertex onto nearest output segment.
        # Build a spatial index on output segment midpoints (XY only) for candidate search.
        segs = []  # (ax,ay,az,bx,by,bz,mid_x,mid_y)
        for poly in out_polys:
            for i in range(len(poly) - 1):
                ax, ay, az = poly[i]; bx, by, bz = poly[i+1]
                segs.append((ax, ay, az, bx, by, bz, (ax+bx)/2, (ay+by)/2))

        SEARCH = 5.0  # 5 m candidate radius
        violations = []
        for ri, (rx, ry, rz) in enumerate(ref_verts):
            best_d = SEARCH; best_foot = None
            for ax, ay, az, bx, by, bz, mx, my in segs:
                # Quick XY prefilter on segment midpoint
                if abs(mx - rx) > SEARCH or abs(my - ry) > SEARCH:
                    continue
                d, fx, fy, fz = pt_to_seg_3d(rx, ry, rz, ax, ay, az, bx, by, bz)
                if d < best_d:
                    best_d = d; best_foot = (fx, fy, fz)
            if best_foot is None or best_d > TOLERANCE:
                foot = best_foot or (rx, ry, rz)
                violations.append({
                    "ref_poly": ref_poly_idx[ri],
                    "ref": (rx, ry, rz),
                    "out": best_foot,
                    "d3d": best_d if best_foot is not None else 9999,
                    "dx": abs(rx - foot[0]),
                    "dy": abs(ry - foot[1]),
                    "dz": abs(rz - foot[2]),
                })
        return out_verts, ref_verts, violations

    # Vertex-to-vertex comparison (pads)
    grid, cell = build_3d_grid(out_verts, cell=0.05)
    SNAP = TOLERANCE * 3
    grid_wide, cell_wide = build_3d_grid(out_verts, cell=0.2)

    violations = []
    for ri, (rx, ry, rz) in enumerate(ref_verts):
        d, oi = nearest_3d(rx, ry, rz, out_verts, grid, cell, SNAP)
        if oi is None or d > TOLERANCE:
            d2, oi2 = nearest_3d(rx, ry, rz, out_verts, grid_wide, cell_wide, 5.0)
            if oi2 is not None and (oi is None or d2 < d):
                d, oi = d2, oi2
        if oi is None or d > TOLERANCE:
            ox, oy, oz = out_verts[oi] if oi is not None else (rx, ry, rz)
            violations.append({
                "ref_poly": ref_poly_idx[ri],
                "ref": (rx, ry, rz),
                "out": (ox, oy, oz) if oi is not None else None,
                "d3d": d if oi is not None else 9999,
                "dx": abs(rx - ox) if oi is not None else None,
                "dy": abs(ry - oy) if oi is not None else None,
                "dz": abs(rz - oz) if oi is not None else None,
            })

    return out_verts, ref_verts, violations


def main():
    out_file = sys.argv[1] if len(sys.argv) > 1 else OUTPUT
    ref_file = sys.argv[2] if len(sys.argv) > 2 else REF

    print(f"Output:    {out_file}")
    print(f"Reference: {ref_file}")
    print(f"Tolerance: {TOLERANCE*1000:.0f}mm (3D distance)\n")

    out_doc = ezdxf.readfile(out_file)
    ref_doc = ezdxf.readfile(ref_file)

    all_layers = sorted(
        set(e.dxf.layer for e in out_doc.modelspace()) |
        set(e.dxf.layer for e in ref_doc.modelspace())
    )
    # Only care about layers in the reference
    ref_layers = sorted(set(e.dxf.layer for e in ref_doc.modelspace()))

    print("=" * 100)
    print(f"{'Layer':<40} {'OutP':>5} {'RefP':>5} {'OutV':>6} {'RefV':>6} {'Viols':>6}  {'MaxD mm':>8}  Status")
    print("=" * 100)

    # Use segment projection for 3D_LINES (linework may have different vertex density)
    SEGMENT_PROJ_LAYERS = {"3D_LINES"}

    all_viols = {}
    for layer in ref_layers:
        out_p = get_polylines(out_doc, layer)
        ref_p = get_polylines(ref_doc, layer)
        use_seg = layer in SEGMENT_PROJ_LAYERS
        out_v, ref_v, viols = compare_layer(out_p, ref_p, use_segment_projection=use_seg)
        all_viols[layer] = viols
        max_d = max((v["d3d"] for v in viols), default=0)
        status = "PASS" if not viols else f"FAIL ({len(viols)} verts)"
        flag = " <--" if viols else ""
        print(f"{layer:<40} {len(out_p):5d} {len(ref_p):5d} {len(out_v):6d} {len(ref_v):6d} "
              f"{len(viols):6d}  {max_d*1000:8.1f}  {status}{flag}")

    # Detailed failures
    total_viols = sum(len(v) for v in all_viols.values())
    if total_viols:
        print(f"\n{'='*100}")
        print("DETAILED FAILURES")
        print(f"{'='*100}")
        for layer, viols in all_viols.items():
            if not viols:
                continue
            print(f"\nLayer: {layer}  ({len(viols)} violations, showing worst 30)")
            print(f"  {'Poly':>5} {'Ref X':>12} {'Ref Y':>12} {'Ref Z':>9} {'Out X':>12} {'Out Y':>12} {'Out Z':>9} {'ΔX mm':>7} {'ΔY mm':>7} {'ΔZ mm':>7} {'3D mm':>7}")
            print(f"  {'-'*5} {'-'*12} {'-'*12} {'-'*9} {'-'*12} {'-'*12} {'-'*9} {'-'*7} {'-'*7} {'-'*7} {'-'*7}")
            sorted_v = sorted(viols, key=lambda v: v["d3d"], reverse=True)
            for v in sorted_v[:30]:
                rx, ry, rz = v["ref"]
                if v["out"]:
                    ox, oy, oz = v["out"]
                    dx = v["dx"]*1000; dy = v["dy"]*1000; dz = v["dz"]*1000
                else:
                    ox = oy = oz = float("nan")
                    dx = dy = dz = float("nan")
                d3d = v["d3d"]*1000
                print(f"  {v['ref_poly']:5d} {rx:12.4f} {ry:12.4f} {rz:9.4f} {ox:12.4f} {oy:12.4f} {oz:9.4f} {dx:7.1f} {dy:7.1f} {dz:7.1f} {d3d:7.1f}")
            if len(viols) > 30:
                print(f"  ... and {len(viols)-30} more")

    print(f"\n{'='*100}")
    if total_viols == 0:
        print("OVERALL: PASS - All reference vertices within 10mm 3D tolerance")
    else:
        print(f"OVERALL: FAIL - {total_viols} reference vertices exceed 10mm 3D tolerance")
    print("=" * 100)


if __name__ == "__main__":
    main()
