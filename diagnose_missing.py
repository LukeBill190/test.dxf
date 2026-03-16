#!/usr/bin/env python3
"""
Diagnose why two large reference pads are missing from the output.
Ref[13] at (368387.2, 357823.5) area=187.7m²
Ref[33] at (368370.7, 357701.8) area=200.9m²
"""
import math
import ezdxf
from collections import deque, defaultdict
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union

SOURCE = "/home/user/test.dxf/Test.dxf"
REF    = "/home/user/test.dxf/FINISHED MODEL.dxf"

MISSING_PADS = [
    ("Ref[13]", (368387.2, 357823.5), 187.7),
    ("Ref[33]", (368370.7, 357701.8), 200.9),
]

ADJACENCY_TOL = 0.35
FFL_PROXIMITY = 5.0
WALL_BUFFER   = 0.15

def _apply_xform(bx, by, tx, ty, sx, sy, rot_deg):
    rot = math.radians(rot_deg)
    c, s = math.cos(rot), math.sin(rot)
    return (tx + sx*c*bx - sy*s*by, ty + sx*s*bx + sy*c*by)


def collect_outlines(doc, msp):
    outlines = []
    seen = set()

    def add(verts):
        if len(verts) < 3:
            return
        cx = round(sum(v[0] for v in verts) / len(verts), 2)
        cy = round(sum(v[1] for v in verts) / len(verts), 2)
        if (cx, cy) not in seen:
            seen.add((cx, cy))
            outlines.append(verts)

    def walk(ins, parent_xform):
        try:
            block = doc.blocks.get(ins.dxf.name)
        except Exception:
            return
        if block is None:
            return
        sx = ins.dxf.get("xscale", 1.0); sy = ins.dxf.get("yscale", 1.0)
        rot = ins.dxf.get("rotation", 0.0)
        tx, ty = ins.dxf.insert.x, ins.dxf.insert.y
        for ent in block:
            if ent.dxftype() == "LWPOLYLINE" and "PLOT OUTLINE INNER" in ent.dxf.layer.upper():
                raw = [(x, y) for x, y, *_ in ent.get_points()]
                s1 = [_apply_xform(bx, by, tx, ty, sx, sy, rot) for bx, by in raw]
                px, py, psx, psy, prot = parent_xform
                s2 = [_apply_xform(wx, wy, px, py, psx, psy, prot) for wx, wy in s1]
                add(s2)
            elif ent.dxftype() == "INSERT":
                walk(ent, (tx, ty, sx, sy, rot))

    for ent in msp:
        if ent.dxftype() == "LWPOLYLINE" and "PLOT OUTLINE INNER" in ent.dxf.layer.upper():
            add([(x, y) for x, y, *_ in ent.get_points()])
        elif ent.dxftype() == "INSERT":
            walk(ent, (0.0, 0.0, 1.0, 1.0, 0.0))
    return outlines


def collect_ffl_annotations(msp):
    import re
    FFL_PATTERN = re.compile(r"FFL[\s\\P]*(\d{2,3}\.\d{1,4})", re.IGNORECASE)
    results = []
    for ent in msp:
        if ent.dxf.layer.upper() not in ("LR LLFA FFL",):
            continue
        if ent.dxftype() == "MTEXT":
            text = ent.plain_text()
            m = FFL_PATTERN.search(text)
            if m:
                pt = ent.dxf.insert
                results.append(((pt.x, pt.y), float(m.group(1))))
    return results


def dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


def main():
    print("Loading...")
    doc = ezdxf.readfile(SOURCE)
    msp = doc.modelspace()

    outlines = collect_outlines(doc, msp)
    ffls     = collect_ffl_annotations(msp)

    print(f"  {len(outlines)} inner polygons, {len(ffls)} FFL annotations")

    # Build shapely polygons
    polys = [Polygon(v) for v in outlines if len(v) >= 3]
    polys = [p.buffer(0) if not p.is_valid else p for p in polys]
    n = len(polys)

    # Show inner polygons near each missing pad
    for label, (mx, my), marea in MISSING_PADS:
        print(f"\n=== {label} — ref at ({mx:.1f}, {my:.1f}) area={marea:.1f}m² ===")
        nearby_inner = []
        for i, poly in enumerate(polys):
            d = dist((poly.centroid.x, poly.centroid.y), (mx, my))
            if d <= 15.0:
                nearby_inner.append((d, i, poly))
        nearby_inner.sort()
        print(f"  Inner polygons within 15m: {len(nearby_inner)}")
        for d, i, poly in nearby_inner:
            print(f"    poly[{i:2d}] centroid=({poly.centroid.x:.1f},{poly.centroid.y:.1f}) "
                  f"area={poly.area:.1f}m² dist={d:.1f}m")

    # FFL annotations near missing pads
    print("\n=== FFL annotations near missing pads ===")
    for label, (mx, my), marea in MISSING_PADS:
        print(f"\n  {label} ({mx:.1f},{my:.1f}) area={marea:.1f}m²:")
        nearby_ffls = [(dist((fx, fy), (mx, my)), fv, (fx, fy)) for (fx, fy), fv in ffls]
        nearby_ffls.sort()
        for d, fv, (fx, fy) in nearby_ffls[:5]:
            print(f"    FFL={fv:.2f} at ({fx:.1f},{fy:.1f}) dist={d:.1f}m")

    # Run the FFL assignment and show what each inner polygon gets
    print("\n=== FFL assignment per inner polygon ===")
    ffl_assigned = [None] * n

    for (fx, fy), ffl_val in ffls:
        pt = Point(fx, fy)
        best_i, best_d = None, FFL_PROXIMITY
        for i, poly in enumerate(polys):
            d = poly.distance(pt)
            if d < best_d:
                best_d, best_i = d, i
        if best_i is not None and ffl_assigned[best_i] is None:
            ffl_assigned[best_i] = ffl_val

    # BFS
    adj = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i+1, n):
            if polys[i].distance(polys[j]) < ADJACENCY_TOL:
                adj[i].append(j); adj[j].append(i)

    queue = deque(i for i in range(n) if ffl_assigned[i] is not None)
    while queue:
        i = queue.popleft()
        for j in adj[i]:
            if ffl_assigned[j] is None:
                ffl_assigned[j] = ffl_assigned[i]
                queue.append(j)

    # Pass 3
    for i in range(n):
        if ffl_assigned[i] is not None:
            continue
        cx, cy = polys[i].centroid.x, polys[i].centroid.y
        best_ffl, best_d = None, FFL_PROXIMITY
        for (fx, fy), fv in ffls:
            d = dist((cx, cy), (fx, fy))
            if d < best_d:
                best_d, best_ffl = d, fv
        ffl_assigned[i] = best_ffl

    # Show assignments for polygons near missing pads
    for label, (mx, my), marea in MISSING_PADS:
        print(f"\n  Near {label} ({mx:.1f},{my:.1f}):")
        for i, poly in enumerate(polys):
            d = dist((poly.centroid.x, poly.centroid.y), (mx, my))
            if d <= 15.0:
                ffl = ffl_assigned[i]
                print(f"    poly[{i:2d}] centroid=({poly.centroid.x:.1f},{poly.centroid.y:.1f}) "
                      f"area={poly.area:.1f}m² dist={d:.1f}m FFL={ffl}")

    # Show merged groups
    print("\n=== Merged group sizes ===")
    groups = defaultdict(list)
    for i, ffl_val in enumerate(ffl_assigned):
        if ffl_val is not None:
            groups[ffl_val].append(i)
    for ffl_val, idxs in sorted(groups.items()):
        total_area = sum(polys[i].area for i in idxs)
        cx = sum(polys[i].centroid.x for i in idxs) / len(idxs)
        cy = sum(polys[i].centroid.y for i in idxs) / len(idxs)
        print(f"  FFL={ffl_val:.2f}: {len(idxs)} plots, total_inner_area={total_area:.1f}m² "
              f"center=({cx:.1f},{cy:.1f})")


if __name__ == "__main__":
    main()
