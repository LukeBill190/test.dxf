#!/usr/bin/env python3
"""
Diagnose H-EXTERNAL PARTY WALL 352 and H-EXTERNAL WALL polygons.
Compare their centroids/shapes to the reference PLOTS FFL pads.
"""
import math
import ezdxf
from shapely.geometry import Polygon
from shapely.ops import unary_union
from collections import defaultdict

SOURCE = "/home/user/test.dxf/Test.dxf"
REF    = "/home/user/test.dxf/FINISHED MODEL.dxf"

TARGET_LAYERS = [
    "EXTERNAL PARTY WALL",   # catches H-EXTERNAL PARTY WALL 352
    "EXTERNAL WALL",         # catches H-EXTERNAL WALL
    "PLOT OUTLINE INNER",    # current source (inner floor area)
]


def _apply_xform(bx, by, tx, ty, sx, sy, rot_deg):
    rot = math.radians(rot_deg)
    c, s = math.cos(rot), math.sin(rot)
    return (tx + sx * c * bx - sy * s * by,
            ty + sx * s * bx + sy * c * by)


def collect_layer_polys(dxf_path, layer_substrings):
    """Collect LWPOLYLINEs from blocks whose layer matches any substring."""
    doc = ezdxf.readfile(dxf_path)
    msp = doc.modelspace()

    results = defaultdict(list)   # layer_key -> list of vertex lists

    def layer_key(name):
        nu = name.upper()
        for sub in layer_substrings:
            if sub.upper() in nu:
                return sub
        return None

    def try_add(verts, layer):
        k = layer_key(layer)
        if k and len(verts) >= 3:
            results[k].append(verts)

    def walk_insert(ins, parent_xform):
        try:
            block = doc.blocks.get(ins.dxf.name)
        except Exception:
            return
        if block is None:
            return
        sx = ins.dxf.get("xscale", 1.0)
        sy = ins.dxf.get("yscale", 1.0)
        rot = ins.dxf.get("rotation", 0.0)
        tx, ty = ins.dxf.insert.x, ins.dxf.insert.y

        for ent in block:
            if ent.dxftype() == "LWPOLYLINE":
                k = layer_key(ent.dxf.layer)
                if k:
                    raw = [(x, y) for x, y, *_ in ent.get_points()]
                    step1 = [_apply_xform(bx, by, tx, ty, sx, sy, rot) for bx, by in raw]
                    px, py, psx, psy, prot = parent_xform
                    step2 = [_apply_xform(wx, wy, px, py, psx, psy, prot) for wx, wy in step1]
                    if len(step2) >= 3:
                        results[k].append(step2)
            elif ent.dxftype() == "INSERT":
                walk_insert(ent, (tx, ty, sx, sy, rot))

    for ent in msp:
        if ent.dxftype() == "LWPOLYLINE":
            try_add([(x, y) for x, y, *_ in ent.get_points()], ent.dxf.layer)
        elif ent.dxftype() == "INSERT":
            walk_insert(ent, (0.0, 0.0, 1.0, 1.0, 0.0))

    return results


def collect_ref_pads(dxf_path):
    """Read PLOTS FFL polylines from the reference file."""
    doc = ezdxf.readfile(dxf_path)
    msp = doc.modelspace()
    pads = []
    for ent in msp:
        if "PLOTS FFL" in ent.dxf.layer.upper():
            if ent.dxftype() == "LWPOLYLINE":
                verts = [(x, y) for x, y, *_ in ent.get_points()]
                if len(verts) >= 3:
                    pads.append(verts)
            elif ent.dxftype() in ("POLYLINE", "POLYLINE3D"):
                try:
                    verts = [(v.dxf.location.x, v.dxf.location.y) for v in ent.vertices]
                    if len(verts) >= 3:
                        pads.append(verts)
                except Exception:
                    pass
    return pads


def centroid(verts):
    n = len(verts)
    return (sum(x for x, y in verts) / n, sum(y for x, y in verts) / n)


def dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


def main():
    print("Loading source DXF...")
    poly_groups = collect_layer_polys(SOURCE, TARGET_LAYERS)

    print("\n=== Polygon counts per layer ===")
    for k, v in poly_groups.items():
        areas = []
        for verts in v:
            try:
                p = Polygon(verts)
                if p.is_valid and not p.is_empty:
                    areas.append(p.area)
            except Exception:
                pass
        print(f"  {k!r:35s}: {len(v):3d} polygons, "
              f"area range: {min(areas):.1f}–{max(areas):.1f} m²"
              if areas else f"  {k!r:35s}: {len(v):3d} polygons")

    print("\nLoading reference PLOTS FFL...")
    ref_verts = collect_ref_pads(REF)
    print(f"  {len(ref_verts)} reference PLOTS FFL pads")

    ref_centroids = [centroid(v) for v in ref_verts]
    ref_areas = []
    for v in ref_verts:
        try:
            p = Polygon(v)
            ref_areas.append(p.area if p.is_valid else 0.0)
        except Exception:
            ref_areas.append(0.0)

    print(f"\n  Reference area range: {min(ref_areas):.1f}–{max(ref_areas):.1f} m²")

    # For each layer, report how well its centroids match reference
    print("\n=== Centroid match analysis ===")
    for k, vlist in poly_groups.items():
        if not vlist:
            continue
        src_centroids = [centroid(v) for v in vlist]

        # For each reference pad, find nearest source centroid
        matched = 0
        unmatched = []
        total_err = 0.0
        for rc in ref_centroids:
            best_d = min(dist(rc, sc) for sc in src_centroids)
            total_err += best_d
            if best_d <= 5.0:
                matched += 1
            else:
                unmatched.append((rc, best_d))

        print(f"\n  Layer: {k!r}")
        print(f"    Source polygons: {len(src_centroids)}")
        print(f"    Ref pads matched within 5m: {matched}/{len(ref_centroids)}")
        print(f"    Mean centroid error: {total_err/len(ref_centroids):.2f} m")
        if unmatched:
            print(f"    Unmatched ref pads (>{5}m):")
            for rc, d in sorted(unmatched, key=lambda x: -x[1])[:10]:
                print(f"      ref @ ({rc[0]:.1f}, {rc[1]:.1f}) → nearest source {d:.1f} m away")

    # Deep analysis for EXTERNAL PARTY WALL only
    pw_key = "EXTERNAL PARTY WALL"
    if pw_key in poly_groups:
        print(f"\n=== H-EXTERNAL PARTY WALL 352 detail ===")
        pw_polys = []
        for v in poly_groups[pw_key]:
            try:
                p = Polygon(v)
                if p.is_valid and not p.is_empty:
                    pw_polys.append((p, v))
            except Exception:
                pass

        print(f"  Valid Shapely polygons: {len(pw_polys)}")

        # Check adjacency / grouping
        # Try buffering and union to see how many merged groups we get
        buffered = [p.buffer(0.15) for p, _ in pw_polys]
        merged = unary_union(buffered)
        if merged.geom_type == "Polygon":
            n_groups = 1
        elif merged.geom_type == "MultiPolygon":
            n_groups = len(list(merged.geoms))
        else:
            n_groups = 0
        print(f"  Merged groups (150mm buffer): {n_groups}")

        # Match merged groups to reference pads
        if merged.geom_type == "MultiPolygon":
            group_list = list(merged.geoms)
        elif merged.geom_type == "Polygon":
            group_list = [merged]
        else:
            group_list = []

        matched2 = 0
        for rc in ref_centroids:
            best_d = min(g.centroid.distance(Polygon([(rc[0], rc[1])])) for g in group_list) if group_list else 999
            # Actually use distance from point
            from shapely.geometry import Point
            pt = Point(rc)
            best_d = min(g.centroid.distance(pt) for g in group_list) if group_list else 999
            if best_d <= 5.0:
                matched2 += 1
        print(f"  Merged group centroids matched within 5m to reference: {matched2}/{len(ref_centroids)}")

        # Show sample centroids
        print("\n  Sample EXTERNAL PARTY WALL centroid coords:")
        sorted_polys = sorted(pw_polys, key=lambda x: (round(x[0].centroid.y), round(x[0].centroid.x)))
        for i, (p, _) in enumerate(sorted_polys[:10]):
            print(f"    [{i}] centroid=({p.centroid.x:.2f}, {p.centroid.y:.2f}) area={p.area:.1f}m² nverts={len(p.exterior.coords)}")

        print("\n  Sample reference PLOTS FFL centroids:")
        sorted_ref = sorted(zip(ref_centroids, ref_areas), key=lambda x: (round(x[0][1]), round(x[0][0])))
        for i, (rc, ra) in enumerate(sorted_ref[:10]):
            print(f"    [{i}] centroid=({rc[0]:.2f}, {rc[1]:.2f}) area={ra:.1f}m²")


if __name__ == "__main__":
    main()
