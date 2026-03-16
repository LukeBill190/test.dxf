#!/usr/bin/env python3
"""
Find all INSERT blocks near missing pads and list their layer geometry.
"""
import math
import ezdxf
from collections import defaultdict

SOURCE = "/home/user/test.dxf/Test.dxf"
MISSING = [
    ("Ref[13]", (368387.2, 357823.5), 187.7),
    ("Ref[33]", (368370.7, 357701.8), 200.9),
]
RADIUS = 20.0


def dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


def main():
    doc = ezdxf.readfile(SOURCE)
    msp = doc.modelspace()

    # Find INSERT entities near missing pads
    for label, (mx, my), marea in MISSING:
        print(f"\n=== {label} at ({mx:.1f},{my:.1f}) area={marea:.1f}m² ===")
        nearby = []
        for ent in msp:
            if ent.dxftype() == "INSERT":
                ix, iy = ent.dxf.insert.x, ent.dxf.insert.y
                d = dist((ix, iy), (mx, my))
                if d <= RADIUS:
                    nearby.append((d, ent.dxf.name, ix, iy,
                                   ent.dxf.get("xscale", 1.0),
                                   ent.dxf.get("rotation", 0.0)))
        nearby.sort()
        print(f"  {len(nearby)} INSERT entities within {RADIUS}m:")
        for d, name, ix, iy, sx, rot in nearby:
            print(f"    {name:30s} insert=({ix:.1f},{iy:.1f}) sx={sx:+.2f} rot={rot:.1f}° dist={d:.1f}m")

        # Show what layers exist in the nearest blocks
        print(f"\n  Layers in those blocks:")
        block_layers = defaultdict(set)
        for d, name, ix, iy, sx, rot in nearby:
            try:
                block = doc.blocks.get(name)
            except Exception:
                continue
            if block is None:
                continue
            for ent in block:
                if hasattr(ent, 'dxf') and hasattr(ent.dxf, 'layer'):
                    block_layers[name].add(ent.dxf.layer)

        for d, name, *_ in nearby:
            layers = block_layers.get(name, set())
            print(f"    {name:30s}: {sorted(layers)}")

    # Also show all layer names available in the DXF that contain "outline", "external", "wall"
    print("\n=== Potentially relevant layers in source DXF ===")
    for layer in doc.layers:
        ln = layer.dxf.name.upper()
        if any(k in ln for k in ["OUTLINE", "EXTERNAL", "WALL", "PLOT", "BUILDING", "FLOOR", "SLAB"]):
            print(f"  {layer.dxf.name}")


if __name__ == "__main__":
    main()
