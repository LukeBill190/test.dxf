#!/usr/bin/env python3
"""Debug ELMSLIE-HOUSE-MID outer polygon generation."""
import math
import ezdxf
from shapely.geometry import MultiPoint

INPUT_DXF = "/home/user/test.dxf/Test.dxf"

def dist_2d(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def _chain_lines_to_outlines(segs, snap_tol=0.05):
    """Chain line segments into closed polygons."""
    from collections import deque
    remaining = list(segs)
    chains = []
    while remaining:
        chain = list(remaining.pop(0))
        changed = True
        while changed:
            changed = False
            for i, seg in enumerate(remaining):
                a, b = seg
                if dist_2d(chain[-1], a) <= snap_tol:
                    chain.append(b)
                    remaining.pop(i)
                    changed = True
                    break
                elif dist_2d(chain[-1], b) <= snap_tol:
                    chain.append(a)
                    remaining.pop(i)
                    changed = True
                    break
                elif dist_2d(chain[0], b) <= snap_tol:
                    chain.insert(0, a)
                    remaining.pop(i)
                    changed = True
                    break
                elif dist_2d(chain[0], a) <= snap_tol:
                    chain.insert(0, b)
                    remaining.pop(i)
                    changed = True
                    break
        chains.append(chain)
    return [c for c in chains if dist_2d(c[0], c[-1]) <= snap_tol]

def get_block_entities(doc, block_name):
    """Get all entities in a block with their layer info."""
    try:
        blk = doc.blocks[block_name]
    except KeyError:
        return []
    return list(blk)

def apply_transform(x, y, tx, ty, sx, sy, rot_deg):
    """Apply scale + rotation + translation transform."""
    rad = math.radians(rot_deg)
    cos_r, sin_r = math.cos(rad), math.sin(rad)
    x_s = x * sx
    y_s = y * sy
    x_r = x_s * cos_r - y_s * sin_r
    y_r = x_s * sin_r + y_s * cos_r
    return (tx + x_r, ty + y_r)

def main():
    doc = ezdxf.readfile(INPUT_DXF)
    msp = doc.modelspace()

    # Find ELMSLIE-HOUSE-MID inserts in modelspace
    print("=== ELMSLIE-HOUSE-MID inserts in modelspace ===")
    elmslie_mid_inserts = []
    for ent in msp:
        if ent.dxftype() == "INSERT":
            bname = ent.dxf.name
            if "ELMSLIE" in bname.upper() and "MID" in bname.upper():
                ins = ent.dxf.insert
                rot = ent.dxf.get("rotation", 0.0)
                sx = ent.dxf.get("xscale", 1.0)
                sy = ent.dxf.get("yscale", 1.0)
                print(f"  INSERT {bname}: pos=({ins.x:.3f},{ins.y:.3f}) rot={rot:.2f} sx={sx} sy={sy}")
                elmslie_mid_inserts.append((bname, ins.x, ins.y, rot, sx, sy))

    # For each ELMSLIE-MID block type, check what entities it has
    print("\n=== Block entity analysis ===")
    block_names = set(b for b, *_ in elmslie_mid_inserts)
    for bname in block_names:
        print(f"\nBlock: {bname}")
        try:
            blk = doc.blocks[bname]
        except KeyError:
            print(f"  NOT FOUND")
            continue

        outer_line_segs = []
        party352_segs = []

        for ent in blk:
            if ent.dxftype() == "LINE":
                lyr = ent.dxf.layer.upper()
                s = ent.dxf.start
                e = ent.dxf.end
                is_outer = "H-EXTERNAL WALL" in lyr and "PARTY" not in lyr
                is_party352 = "EXTERNAL PARTY WALL 352" in lyr

                print(f"  LINE {lyr}: ({s.x:.4f},{s.y:.4f})->({e.x:.4f},{e.y:.4f})")

                if is_outer:
                    outer_line_segs.append(((s.x, s.y), (e.x, e.y)))
                elif is_party352:
                    party352_segs.append(((s.x, s.y), (e.x, e.y)))

        print(f"  outer_line_segs: {outer_line_segs}")
        print(f"  party352_segs: {party352_segs}")

        # Test chaining logic
        outer_added_by_lwpoly = False  # No LWPOLYLINE outer in MID block

        if outer_line_segs and not outer_added_by_lwpoly:
            all_outer_segs = outer_line_segs + party352_segs
            print(f"\n  Testing chaining with {len(all_outer_segs)} segments:")
            try:
                closed_outer = _chain_lines_to_outlines(all_outer_segs, snap_tol=0.05)
                print(f"  Chaining produced {len(closed_outer)} closed chains")
                for i, chain in enumerate(closed_outer):
                    if len(chain) >= 3:
                        print(f"    Chain {i}: {len(chain)} pts -> outer polygon ADDED, outer_added_by_lwpoly=True")
                        print(f"    First 5 pts: {chain[:5]}")
                        outer_added_by_lwpoly = True
                    else:
                        print(f"    Chain {i}: {len(chain)} pts -> too short, skipped")
            except Exception as ex:
                print(f"  Chaining FAILED: {ex}")

        # Test hull fallback
        if not outer_added_by_lwpoly and party352_segs:
            party_pts = set()
            for (ax, ay), (bx, by) in party352_segs:
                party_pts.add((round(ax, 3), round(ay, 3)))
                party_pts.add((round(bx, 3), round(by, 3)))
            print(f"\n  Hull fallback: {len(party_pts)} unique party352 points")
            print(f"  Points: {party_pts}")
            if len(party_pts) >= 4:
                hull = MultiPoint(list(party_pts)).convex_hull
                print(f"  Hull geom_type: {hull.geom_type}")
                if hull.geom_type == "Polygon":
                    print(f"  Hull area: {hull.area:.4f} m²")
                    print(f"  Hull coords: {list(hull.exterior.coords)}")
        elif not outer_added_by_lwpoly:
            print(f"\n  No outer polygon generated for this block!")

    # Now check the actual output polygon area for the failing region
    print("\n=== Output polygon check for failing area ===")
    out_doc = ezdxf.readfile("/home/user/test.dxf/Test_v3_Output.dxf")
    out_msp = out_doc.modelspace()

    # Failing ref vertex: (368419.726, 357747.965)
    fx, fy = 368419.726, 357747.965

    for ent in out_msp:
        if ent.dxf.layer.upper() != "PLOTS FFL":
            continue
        if ent.dxftype() == "LWPOLYLINE":
            pts = list(ent.get_points())
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            if min(xs) < fx < max(xs) and min(ys) < fy < max(ys):
                print(f"  Found containing LWPOLYLINE with {len(pts)} pts")
                print(f"  X range: [{min(xs):.3f}, {max(xs):.3f}]")
                print(f"  Y range: [{min(ys):.3f}, {max(ys):.3f}]")
                print(f"  Vertices:")
                for p in pts:
                    print(f"    ({p[0]:.4f}, {p[1]:.4f})")

if __name__ == "__main__":
    main()
