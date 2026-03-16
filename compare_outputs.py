#!/usr/bin/env python3
"""
Compare pipeline output PLOTS FFL pads to reference PLOTS FFL pads.
"""
import math
import ezdxf
from shapely.geometry import Polygon, Point

OUTPUT = "/home/user/test.dxf/Test_v3_Output.dxf"
REF    = "/home/user/test.dxf/FINISHED MODEL.dxf"


def collect_plots_ffl(dxf_path):
    doc = ezdxf.readfile(dxf_path)
    msp = doc.modelspace()
    pads = []
    for ent in msp:
        layer = ent.dxf.layer.upper()
        if "PLOTS FFL" not in layer:
            continue
        if ent.dxftype() == "LWPOLYLINE":
            verts = [(x, y) for x, y, *_ in ent.get_points()]
            if len(verts) >= 3:
                pads.append(verts)
        elif ent.dxftype() in ("POLYLINE",):
            try:
                verts = [(v.dxf.location.x, v.dxf.location.y) for v in ent.vertices]
                if len(verts) >= 3:
                    pads.append(verts)
            except Exception:
                pass
        elif ent.dxftype() == "POLYLINE3D" or hasattr(ent, 'vertices'):
            try:
                verts = [(v.dxf.location.x, v.dxf.location.y) for v in ent.vertices]
                if len(verts) >= 3:
                    pads.append(verts)
            except Exception:
                pass
    return pads


def read_3d_polylines(dxf_path, layer_name):
    """Read 3D polylines from a given layer."""
    doc = ezdxf.readfile(dxf_path)
    msp = doc.modelspace()
    pads = []
    for ent in msp:
        if ent.dxf.layer.upper() != layer_name.upper():
            continue
        t = ent.dxftype()
        if t == "POLYLINE":
            try:
                verts = [(v.dxf.location.x, v.dxf.location.y) for v in ent.vertices]
                if len(verts) >= 3:
                    pads.append(verts)
            except Exception:
                pass
        elif t == "LWPOLYLINE":
            verts = [(x, y) for x, y, *_ in ent.get_points()]
            if len(verts) >= 3:
                pads.append(verts)
    return pads


def centroid(verts):
    """Use Shapely's true geometric centroid."""
    try:
        p = Polygon(verts)
        if p.is_valid and not p.is_empty:
            c = p.centroid
            return (c.x, c.y)
    except Exception:
        pass
    n = len(verts)
    return (sum(x for x, y in verts) / n, sum(y for x, y in verts) / n)


def dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


def main():
    print("Reading output PLOTS FFL...")
    out_pads = read_3d_polylines(OUTPUT, "PLOTS FFL")
    print(f"  {len(out_pads)} output pads")

    print("Reading reference PLOTS FFL...")
    ref_pads = read_3d_polylines(REF, "PLOTS FFL")
    print(f"  {len(ref_pads)} reference pads")

    out_c = [centroid(v) for v in out_pads]
    ref_c = [centroid(v) for v in ref_pads]

    out_areas = []
    for v in out_pads:
        try:
            p = Polygon(v)
            out_areas.append(p.area if p.is_valid else 0.0)
        except Exception:
            out_areas.append(0.0)

    ref_areas = []
    for v in ref_pads:
        try:
            p = Polygon(v)
            ref_areas.append(p.area if p.is_valid else 0.0)
        except Exception:
            ref_areas.append(0.0)

    print(f"\n  Output area range: {min(out_areas):.1f}–{max(out_areas):.1f} m²")
    print(f"  Ref    area range: {min(ref_areas):.1f}–{max(ref_areas):.1f} m²")

    # Match each ref pad to nearest output pad
    print("\n=== Reference vs Output centroid comparison ===")
    matched = 0
    errors = []
    for i, rc in enumerate(ref_c):
        best_j, best_d = None, 999.0
        for j, oc in enumerate(out_c):
            d = dist(rc, oc)
            if d < best_d:
                best_d, best_j = d, j
        errors.append(best_d)
        status = "OK" if best_d <= 2.0 else "WARN" if best_d <= 5.0 else "FAIL"
        if best_d <= 2.0:
            matched += 1
        print(f"  Ref[{i:2d}] ({rc[0]:.1f},{rc[1]:.1f}) area={ref_areas[i]:.1f}m² "
              f"→ Out[{best_j}] dist={best_d:.2f}m [{status}]"
              + (f" out_area={out_areas[best_j]:.1f}m²" if best_j is not None else ""))

    print(f"\nSummary: {matched}/{len(ref_c)} matched within 2m")
    print(f"Mean error: {sum(errors)/len(errors):.2f}m, max: {max(errors):.2f}m")

    # Check for unmatched output pads (no nearby ref pad)
    print("\n=== Unmatched output pads (no ref within 5m) ===")
    for j, oc in enumerate(out_c):
        best_d = min(dist(oc, rc) for rc in ref_c)
        if best_d > 5.0:
            print(f"  Out[{j}] ({oc[0]:.1f},{oc[1]:.1f}) area={out_areas[j]:.1f}m² → nearest ref {best_d:.1f}m")

    # Vertex count comparison
    print("\n=== Vertex count comparison ===")
    out_nv = sorted([len(v) for v in out_pads])
    ref_nv = sorted([len(v) for v in ref_pads])
    print(f"  Output: min={min(out_nv)} max={max(out_nv)} mean={sum(out_nv)/len(out_nv):.1f}")
    print(f"  Ref:    min={min(ref_nv)} max={max(ref_nv)} mean={sum(ref_nv)/len(ref_nv):.1f}")
    print(f"  Output vertex dist: {out_nv}")
    print(f"  Ref    vertex dist: {ref_nv}")


if __name__ == "__main__":
    main()
