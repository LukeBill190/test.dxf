#!/usr/bin/env python3
"""
External Works DXF → 3D Model Pipeline

Reads a 2D external works DXF file and produces a 3D DXF with:
- 3D_Points: POINT entities at spot level locations with Z from text values
- 3D_Lines: 3D polylines for drives, fences, walls, roads with Z from nearby points
- 3D_Building_Pads: Building outlines elevated to FFL

Ported from AutoCAD LISP routines: c:go, c:ApplyElevations, c:Lines23D
"""

import sys
import re
import math
import ezdxf
from ezdxf.math import Vec3
from collections import defaultdict


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# Layer keyword classification (case-insensitive substring matching)
BUILDING_KEYWORDS = ["plot", "building", "house", "garage"]
DRIVE_KEYWORDS = ["drive", "path"]
FENCE_KEYWORDS = ["fence", "wall", "boundary"]
ROAD_KEYWORDS = ["road", "footpath", "kerb"]

# Elevation text layer patterns
SPOT_LEVEL_LAYERS = ["LR SPOT LEVEL", "L018 HA_ANN_FEAT_TEXT"]
FFL_LAYERS = ["LR LLFA FFL"]
DPC_LAYERS = ["LR DPC LEVEL"]

# Regex patterns for elevation text parsing
# Matches: 53.70, +53.70, 53.70+, FFL 53.80, FFL\P53.80, DPC 54.20
ELEVATION_PATTERN = re.compile(
    r"[+]?\s*(\d{2,3}\.\d{1,4})\s*[+]?"
)
FFL_PATTERN = re.compile(
    r"FFL[\s\\P]*(\d{2,3}\.\d{1,4})", re.IGNORECASE
)

# Output layer names and colors
LAYER_3D_POINTS = "3D_Points"
LAYER_3D_LINES = "3D_Lines"
LAYER_3D_BUILDING_PADS = "3D_Building_Pads"

COLOR_POINTS = 1      # Red
COLOR_LINES = 3        # Green
COLOR_PADS = 5         # Blue


# ---------------------------------------------------------------------------
# Utility functions
# ---------------------------------------------------------------------------

def dist_2d(p1, p2):
    """2D distance between two points (ignores Z)."""
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def classify_layer(layer_name):
    """Classify a layer by keyword matching. Returns category or None."""
    ln = layer_name.upper()
    for kw in BUILDING_KEYWORDS:
        if kw.upper() in ln:
            return "building"
    for kw in DRIVE_KEYWORDS:
        if kw.upper() in ln:
            return "drive"
    for kw in FENCE_KEYWORDS:
        if kw.upper() in ln:
            return "fence"
    for kw in ROAD_KEYWORDS:
        if kw.upper() in ln:
            return "road"
    return None


def parse_elevation(text_value, is_ffl=False):
    """Extract a numeric elevation from text. Returns float or None."""
    if not text_value:
        return None
    text_value = text_value.strip()

    if is_ffl:
        m = FFL_PATTERN.search(text_value)
        if m:
            return float(m.group(1))

    m = ELEVATION_PATTERN.search(text_value)
    if m:
        val = float(m.group(1))
        # Sanity check: typical UK site levels are 0-500m AOD
        if 0 < val < 500:
            return val
    return None


def get_text_value(entity):
    """Get the text string from a TEXT or MTEXT entity."""
    if entity.dxftype() == "TEXT":
        return entity.dxf.text
    elif entity.dxftype() == "MTEXT":
        return entity.plain_text()
    return None


def get_text_insertion(entity):
    """Get the insertion point of a TEXT or MTEXT entity."""
    if entity.dxftype() == "TEXT":
        return Vec3(entity.dxf.insert)
    elif entity.dxftype() == "MTEXT":
        return Vec3(entity.dxf.insert)
    return None


# ---------------------------------------------------------------------------
# Phase 1: Parse elevation text and create 3D points
# ---------------------------------------------------------------------------

def collect_elevation_text(msp, doc):
    """
    Collect all elevation text from the drawing, including text inside blocks.
    Returns list of (insertion_point_xy, elevation_value, layer_name, is_ffl).
    """
    results = []

    # Direct text entities in modelspace
    for entity in msp:
        if entity.dxftype() not in ("TEXT", "MTEXT"):
            continue
        layer = entity.dxf.layer.upper()
        text_val = get_text_value(entity)
        insert_pt = get_text_insertion(entity)
        if text_val is None or insert_pt is None:
            continue

        is_ffl = any(fl.upper() in layer for fl in FFL_LAYERS)
        is_spot = any(sl.upper() in layer for sl in SPOT_LEVEL_LAYERS)
        is_dpc = any(dl.upper() in layer for dl in DPC_LAYERS)

        if is_ffl or is_spot or is_dpc:
            elev = parse_elevation(text_val, is_ffl=is_ffl)
            if elev is not None:
                results.append((
                    (insert_pt.x, insert_pt.y),
                    elev,
                    entity.dxf.layer,
                    is_ffl
                ))

    # Also check INSERT entities (blocks) for text — explode them virtually
    for entity in msp:
        if entity.dxftype() != "INSERT":
            continue
        try:
            for sub in entity.virtual_entities():
                if sub.dxftype() not in ("TEXT", "MTEXT"):
                    continue
                layer = sub.dxf.layer.upper()
                text_val = get_text_value(sub)
                insert_pt = get_text_insertion(sub)
                if text_val is None or insert_pt is None:
                    continue

                is_ffl = any(fl.upper() in layer for fl in FFL_LAYERS)
                is_spot = any(sl.upper() in layer for sl in SPOT_LEVEL_LAYERS)
                is_dpc = any(dl.upper() in layer for dl in DPC_LAYERS)

                if is_ffl or is_spot or is_dpc:
                    elev = parse_elevation(text_val, is_ffl=is_ffl)
                    if elev is not None:
                        results.append((
                            (insert_pt.x, insert_pt.y),
                            elev,
                            sub.dxf.layer,
                            is_ffl
                        ))
        except Exception:
            continue

    return results


def create_3d_points(msp, elevation_data):
    """
    Create POINT entities at elevation text locations with Z = elevation value.
    This bridges the gap for LISP routines that expect POINT entities.
    Returns list of (x, y, z) for all created points.
    """
    points_3d = []
    for (x, y), elev, layer, is_ffl in elevation_data:
        msp.add_point(
            (x, y, elev),
            dxfattribs={"layer": LAYER_3D_POINTS}
        )
        points_3d.append((x, y, elev))

    print(f"  Created {len(points_3d)} 3D points on '{LAYER_3D_POINTS}'")
    return points_3d


# ---------------------------------------------------------------------------
# Phase 2: Collect geometry and convert to 3D polylines
# ---------------------------------------------------------------------------

def collect_line_geometry(msp, target_categories=None):
    """
    Collect LINE, LWPOLYLINE entities on classified layers.
    Returns list of (vertices_list, layer_name, category).
    Each vertices_list is [(x, y), (x, y), ...].
    """
    if target_categories is None:
        target_categories = ["drive", "fence", "road"]

    results = []

    for entity in msp:
        layer = entity.dxf.layer
        category = classify_layer(layer)
        if category not in target_categories:
            continue

        verts = []
        if entity.dxftype() == "LINE":
            start = entity.dxf.start
            end = entity.dxf.end
            verts = [(start.x, start.y), (end.x, end.y)]
        elif entity.dxftype() == "LWPOLYLINE":
            for x, y, *_ in entity.get_points():
                verts.append((x, y))
        elif entity.dxftype() == "POLYLINE":
            for v in entity.vertices:
                pt = v.dxf.location
                verts.append((pt.x, pt.y))

        if len(verts) >= 2:
            results.append((verts, layer, category))

    print(f"  Collected {len(results)} line entities for 3D conversion")
    return results


def find_nearest_z(pt_2d, points_3d, radius):
    """Find the nearest 3D point within radius and return its Z. Returns None if not found."""
    best_z = None
    best_dist = radius
    px, py = pt_2d[0], pt_2d[1]
    for (x, y, z) in points_3d:
        d = math.sqrt((px - x) ** 2 + (py - y) ** 2)
        if d <= best_dist:
            best_z = z
            best_dist = d
    return best_z


def interpolate_z(idx, pts_with_z):
    """
    Linearly interpolate Z for vertex at idx from surrounding vertices that have Z.
    Falls back to nearest known Z on the same polyline.
    """
    n = len(pts_with_z)

    # Find previous vertex with Z
    prev_z, prev_pt, prev_idx = None, None, None
    for i in range(idx - 1, -1, -1):
        if pts_with_z[i][2] is not None and pts_with_z[i][2] != 0.0:
            prev_z = pts_with_z[i][2]
            prev_pt = pts_with_z[i]
            prev_idx = i
            break

    # Find next vertex with Z
    next_z, next_pt, next_idx = None, None, None
    for i in range(idx + 1, n):
        if pts_with_z[i][2] is not None and pts_with_z[i][2] != 0.0:
            next_z = pts_with_z[i][2]
            next_pt = pts_with_z[i]
            next_idx = i
            break

    if prev_z is not None and next_z is not None:
        # Linear interpolation
        d_total = dist_2d(prev_pt, next_pt)
        if d_total < 1e-6:
            return prev_z
        d_part = dist_2d(prev_pt, pts_with_z[idx])
        return prev_z + (d_part / d_total) * (next_z - prev_z)
    elif prev_z is not None:
        return prev_z
    elif next_z is not None:
        return next_z

    return None


def convert_lines_to_3d(msp, line_geometry, points_3d, search_radius):
    """
    Convert 2D line geometry to 3D polylines using nearby point elevations.
    Equivalent to c:Lines23D LISP routine.
    """
    count = 0

    for verts_2d, layer, category in line_geometry:
        # Step 1: Assign nearest Z to each vertex
        pts_with_z = []
        for vx, vy in verts_2d:
            z = find_nearest_z((vx, vy), points_3d, search_radius)
            pts_with_z.append((vx, vy, z if z else 0.0))

        # Step 2: Interpolate missing Z values
        for i in range(len(pts_with_z)):
            if pts_with_z[i][2] == 0.0:
                interp = interpolate_z(i, pts_with_z)
                if interp is not None:
                    pts_with_z[i] = (pts_with_z[i][0], pts_with_z[i][1], interp)

        # Step 3: Fill any remaining 0.0 with nearest non-zero Z on same feature
        for i in range(len(pts_with_z)):
            if pts_with_z[i][2] == 0.0:
                best_z = 0.0
                best_d = 1e99
                for j, (ox, oy, oz) in enumerate(pts_with_z):
                    if oz != 0.0 and i != j:
                        d = dist_2d(pts_with_z[i], (ox, oy))
                        if d < best_d:
                            best_d = d
                            best_z = oz
                pts_with_z[i] = (pts_with_z[i][0], pts_with_z[i][1], best_z)

        # Step 4: Create 3D polyline
        if len(pts_with_z) >= 2:
            msp.add_polyline3d(
                pts_with_z,
                dxfattribs={"layer": LAYER_3D_LINES}
            )
            count += 1

    print(f"  Created {count} 3D polylines on '{LAYER_3D_LINES}'")
    return count


# ---------------------------------------------------------------------------
# Phase 3: Building pads — explode blocks, find outlines, elevate to FFL
# ---------------------------------------------------------------------------

def collect_building_outlines(msp, doc):
    """
    Collect building outline polylines, including those inside blocks.
    Returns list of (vertices, block_insert_layer).
    """
    outlines = []

    # Direct polylines on building layers
    for entity in msp:
        if entity.dxftype() == "LWPOLYLINE":
            if classify_layer(entity.dxf.layer) == "building":
                verts = [(x, y) for x, y, *_ in entity.get_points()]
                if len(verts) >= 3:
                    outlines.append((verts, entity.dxf.layer))

    # Explode INSERT entities on building layers to find polylines inside blocks
    for entity in msp:
        if entity.dxftype() != "INSERT":
            continue
        if classify_layer(entity.dxf.layer) != "building":
            continue

        try:
            for sub in entity.virtual_entities():
                if sub.dxftype() == "LWPOLYLINE":
                    sub_layer = sub.dxf.layer
                    # Accept polylines from building layers OR the H-PLOT OUTLINE INNER layer
                    if classify_layer(sub_layer) == "building" or "PLOT" in sub_layer.upper() or "OUTLINE" in sub_layer.upper():
                        verts = [(x, y) for x, y, *_ in sub.get_points()]
                        if len(verts) >= 3:
                            outlines.append((verts, sub_layer))
        except Exception:
            continue

    print(f"  Found {len(outlines)} building outlines (including from exploded blocks)")
    return outlines


def find_ffl_for_outline(outline_verts, ffl_data):
    """
    Find the FFL text that is inside or nearest to a building outline.
    Uses simple centroid proximity matching.
    """
    # Calculate centroid of outline
    cx = sum(v[0] for v in outline_verts) / len(outline_verts)
    cy = sum(v[1] for v in outline_verts) / len(outline_verts)

    # Also calculate bounding box for containment check
    min_x = min(v[0] for v in outline_verts)
    max_x = max(v[0] for v in outline_verts)
    min_y = min(v[1] for v in outline_verts)
    max_y = max(v[1] for v in outline_verts)

    best_ffl = None
    best_dist = float("inf")

    for (fx, fy), elev, layer, is_ffl in ffl_data:
        if not is_ffl:
            continue

        # Check if FFL text is inside bounding box (rough containment)
        if min_x <= fx <= max_x and min_y <= fy <= max_y:
            d = dist_2d((cx, cy), (fx, fy))
            if d < best_dist:
                best_dist = d
                best_ffl = elev

    # If no text inside bounding box, try proximity (within 10m of centroid)
    if best_ffl is None:
        for (fx, fy), elev, layer, is_ffl in ffl_data:
            if not is_ffl:
                continue
            d = dist_2d((cx, cy), (fx, fy))
            if d < 10.0 and d < best_dist:
                best_dist = d
                best_ffl = elev

    return best_ffl


def create_building_pads(msp, doc, elevation_data):
    """
    Create elevated building pad polylines from outlines + FFL text.
    Equivalent to c:go LISP routine (but works without BOUNDARY command).
    """
    outlines = collect_building_outlines(msp, doc)
    ffl_data = [d for d in elevation_data if d[3]]  # Only FFL entries

    count = 0
    for verts, layer in outlines:
        ffl = find_ffl_for_outline(verts, ffl_data)
        if ffl is None:
            # Try DPC levels as fallback (DPC is usually ~150mm above FFL)
            continue

        # Create elevated polyline (all vertices at FFL elevation)
        elevated_verts = [(x, y, ffl) for x, y in verts]
        # Close it
        if verts[0] != verts[-1]:
            elevated_verts.append((verts[0][0], verts[0][1], ffl))

        msp.add_polyline3d(
            elevated_verts,
            dxfattribs={"layer": LAYER_3D_BUILDING_PADS}
        )
        count += 1

    print(f"  Created {count} building pads on '{LAYER_3D_BUILDING_PADS}'")
    return count


# ---------------------------------------------------------------------------
# Setup output layers
# ---------------------------------------------------------------------------

def setup_output_layers(doc):
    """Create the output layers with appropriate colors."""
    layers = doc.layers

    for name, color in [
        (LAYER_3D_POINTS, COLOR_POINTS),
        (LAYER_3D_LINES, COLOR_LINES),
        (LAYER_3D_BUILDING_PADS, COLOR_PADS),
    ]:
        if name not in layers:
            layers.add(name, color=color)


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------

def run_pipeline(input_path, output_path, search_radius=5.0):
    """Run the full DXF → 3D pipeline."""

    print(f"\n{'='*60}")
    print(f"  External Works DXF → 3D Pipeline")
    print(f"{'='*60}")
    print(f"  Input:  {input_path}")
    print(f"  Output: {output_path}")
    print(f"  Search radius: {search_radius}m")
    print(f"{'='*60}\n")

    # Load DXF
    print("[1/6] Loading DXF file...")
    doc = ezdxf.readfile(input_path)
    msp = doc.modelspace()

    entity_count = len(list(msp))
    layer_count = len(doc.layers)
    print(f"  Loaded: {entity_count} entities, {layer_count} layers\n")

    # Setup output layers
    print("[2/6] Setting up output layers...")
    setup_output_layers(doc)
    print()

    # Phase 1: Extract elevation text and create 3D points
    print("[3/6] Extracting elevation text and creating 3D points...")
    elevation_data = collect_elevation_text(msp, doc)
    print(f"  Found {len(elevation_data)} elevation text entities")

    ffl_count = sum(1 for d in elevation_data if d[3])
    spot_count = len(elevation_data) - ffl_count
    print(f"    - {spot_count} spot levels")
    print(f"    - {ffl_count} FFL values")

    if elevation_data:
        elevations = [d[1] for d in elevation_data]
        print(f"    - Elevation range: {min(elevations):.3f}m – {max(elevations):.3f}m")

    points_3d = create_3d_points(msp, elevation_data)
    print()

    # Phase 2: Collect line geometry and convert to 3D
    print("[4/6] Collecting line geometry...")
    line_geometry = collect_line_geometry(msp, ["drive", "fence", "road"])
    print()

    print("[5/6] Converting lines to 3D polylines...")
    convert_lines_to_3d(msp, line_geometry, points_3d, search_radius)
    print()

    # Phase 3: Building pads
    print("[6/6] Creating building pads...")
    create_building_pads(msp, doc, elevation_data)
    print()

    # Save output
    print(f"Saving output to: {output_path}")
    doc.saveas(output_path)

    print(f"\n{'='*60}")
    print(f"  Pipeline complete!")
    print(f"  Open {output_path} in Civil 3D to review.")
    print(f"{'='*60}\n")


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python pipeline.py <input.dxf> [output.dxf] [search_radius]")
        print("  input.dxf      - Input DXF file path")
        print("  output.dxf     - Output file (default: <input>_3D_Output.dxf)")
        print("  search_radius  - Radius in meters for Z assignment (default: 5.0)")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else input_file.replace(".dxf", "_3D_Output.dxf")
    radius = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0

    run_pipeline(input_file, output_file, radius)
