#!/usr/bin/env python3
"""List all 62 collected H-PLOT OUTLINE INNER polygon centroids."""
import math
import ezdxf

SOURCE = "/home/user/test.dxf/Test.dxf"

def _apply_xform(bx, by, tx, ty, sx, sy, rot_deg):
    rot = math.radians(rot_deg)
    c, s = math.cos(rot), math.sin(rot)
    return (tx + sx*c*bx - sy*s*by, ty + sx*s*bx + sy*c*by)


def main():
    doc = ezdxf.readfile(SOURCE)
    msp = doc.modelspace()
    outlines = []
    seen = set()

    def add(verts, source):
        if len(verts) < 3:
            return
        cx = round(sum(v[0] for v in verts) / len(verts), 2)
        cy = round(sum(v[1] for v in verts) / len(verts), 2)
        key = (cx, cy)
        if key not in seen:
            seen.add(key)
            area = 0.5 * abs(sum(verts[i][0]*(verts[(i+1)%len(verts)][1]-verts[(i-1)%len(verts)][1])
                                 for i in range(len(verts))))
            outlines.append((cx, cy, area, source, key))
        else:
            print(f"  DEDUP: centroid ({cx},{cy}) from {source}")

    def walk(ins, parent_xform, depth=0, parent_name="msp"):
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
                add(s2, f"{parent_name}/{ins.dxf.name}(depth={depth})")
            elif ent.dxftype() == "INSERT":
                walk(ent, (tx, ty, sx, sy, rot), depth+1, ins.dxf.name)

    for ent in msp:
        if ent.dxftype() == "LWPOLYLINE" and "PLOT OUTLINE INNER" in ent.dxf.layer.upper():
            verts = [(x, y) for x, y, *_ in ent.get_points()]
            add(verts, "msp_direct")
        elif ent.dxftype() == "INSERT":
            walk(ent, (0.0, 0.0, 1.0, 1.0, 0.0), depth=0, parent_name="msp")

    print(f"Total collected: {len(outlines)}")
    print("\nAll polygon centroids (sorted by Y, X):")
    for i, (cx, cy, area, source, key) in enumerate(sorted(outlines, key=lambda x: (round(x[1]), round(x[0])))):
        print(f"  [{i:2d}] ({cx:.2f},{cy:.2f}) area={area:.1f}m²  from {source}")

    # Focus: what's near missing pads?
    missing = [("Ref[13]", 368387.2, 357823.5), ("Ref[33]", 368370.7, 357701.8)]
    print("\nInner polygons within 20m of each missing pad:")
    for label, mx, my in missing:
        print(f"\n  {label} ({mx:.1f},{my:.1f}):")
        for cx, cy, area, source, key in outlines:
            d = math.sqrt((cx-mx)**2 + (cy-my)**2)
            if d <= 20.0:
                print(f"    ({cx:.2f},{cy:.2f}) area={area:.1f}m² dist={d:.1f}m  {source}")


if __name__ == "__main__":
    main()
