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

# Arc tessellation: max chord length in metres (smaller = smoother)
ARC_CHORD_MAX = 0.1

# Snap tolerances
CHAIN_SNAP_TOL = 0.05       # 50mm: endpoint matching when chaining segments
VERTEX_INSERT_RADIUS = 0.3  # 300mm: insert vertex if no vertex within this radius of a spot level


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


def iter_virtual_deep(insert_entity, max_depth=6):
    """
    Recursively yield all virtual entities within an INSERT block,
    traversing nested INSERT references up to max_depth levels.
    """
    if max_depth <= 0:
        return
    try:
        for sub in insert_entity.virtual_entities():
            yield sub
            if sub.dxftype() == "INSERT":
                yield from iter_virtual_deep(sub, max_depth - 1)
    except Exception:
        pass


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

    # Also check INSERT entities (blocks) for text — recurse through nested blocks
    for entity in msp:
        if entity.dxftype() != "INSERT":
            continue
        try:
            for sub in iter_virtual_deep(entity):
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

def tessellate_arc(entity):
    """
    Convert an ARC entity into a list of 2D points.
    Uses ARC_CHORD_MAX to control segment density.
    """
    cx = entity.dxf.center.x
    cy = entity.dxf.center.y
    r = entity.dxf.radius
    sa = math.radians(entity.dxf.start_angle)
    ea = math.radians(entity.dxf.end_angle)

    # Handle wrap-around (arc crosses 0°)
    if ea <= sa:
        ea += 2 * math.pi

    arc_len = r * (ea - sa)
    n_segs = max(4, int(math.ceil(arc_len / ARC_CHORD_MAX)))

    pts = []
    for i in range(n_segs + 1):
        t = sa + (ea - sa) * i / n_segs
        pts.append((cx + r * math.cos(t), cy + r * math.sin(t)))
    return pts


def collect_line_geometry(msp, target_categories=None):
    """
    Collect LINE, LWPOLYLINE, POLYLINE and ARC entities on classified layers.
    ARCs are tessellated into polyline segments.
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
        elif entity.dxftype() == "ARC":
            verts = tessellate_arc(entity)

        if len(verts) >= 2:
            results.append((verts, layer, category))

    print(f"  Collected {len(results)} line/arc segments for 3D conversion")
    return results


def insert_spot_vertices(segments, elevation_data, search_radius=5.0):
    """
    For each spot elevation text with no nearby vertex within VERTEX_INSERT_RADIUS,
    find the nearest point on any segment within search_radius and split the segment
    there to create a new vertex. This ensures intermediate spot levels along a line
    are honoured when elevations are assigned.

    Returns updated segments list.
    """
    if not segments or not elevation_data:
        return segments

    # Work with mutable lists
    pts_list = [list(pts) for pts, _, _ in segments]
    meta = [(layer, cat) for _, layer, cat in segments]
    added = 0

    for (tx, ty), _elev, _layer, _is_ffl in elevation_data:
        # Skip if any vertex on any segment is already within VERTEX_INSERT_RADIUS
        has_nearby = False
        for pts in pts_list:
            for vx, vy in pts:
                if math.sqrt((tx - vx) ** 2 + (ty - vy) ** 2) <= VERTEX_INSERT_RADIUS:
                    has_nearby = True
                    break
            if has_nearby:
                break
        if has_nearby:
            continue

        # Find nearest point on any segment within search_radius
        best_dist = search_radius
        best_seg = None
        best_edge = None  # index of edge start vertex in that segment
        best_t = None

        for seg_idx, pts in enumerate(pts_list):
            for i in range(len(pts) - 1):
                ax, ay = pts[i]
                bx, by = pts[i + 1]
                dx, dy = bx - ax, by - ay
                len_sq = dx * dx + dy * dy
                if len_sq < 1e-12:
                    continue
                t = ((tx - ax) * dx + (ty - ay) * dy) / len_sq
                t = max(0.0, min(1.0, t))
                nx = ax + t * dx
                ny = ay + t * dy
                d = math.sqrt((tx - nx) ** 2 + (ty - ny) ** 2)
                if d < best_dist:
                    best_dist = d
                    best_seg = seg_idx
                    best_edge = i
                    best_t = t

        # Insert vertex if not exactly at an existing endpoint (t ≠ 0 or 1)
        if best_seg is not None and best_t is not None and 1e-4 < best_t < 1.0 - 1e-4:
            pts = pts_list[best_seg]
            ax, ay = pts[best_edge]
            bx, by = pts[best_edge + 1]
            new_pt = (ax + best_t * (bx - ax), ay + best_t * (by - ay))
            pts_list[best_seg] = pts[:best_edge + 1] + [new_pt] + pts[best_edge + 1:]
            added += 1

    print(f"  Inserted {added} intermediate vertices near elevation texts")
    return [(pts, layer, cat) for pts, (layer, cat) in zip(pts_list, meta)]


def chain_segments(segments, snap_tol=CHAIN_SNAP_TOL):
    """
    Join segments that share endpoints (within snap_tol) into continuous
    polylines, grouped by (layer, category). This is especially important
    for driveways where arcs, lines and polylines form a continuous edge.

    Returns same format: list of (point_list, layer_name, category).
    """
    if not segments:
        return segments

    def snap_key(pt):
        factor = 1.0 / snap_tol
        return (round(pt[0] * factor), round(pt[1] * factor))

    # Group indices by (layer, category)
    groups = defaultdict(list)
    for i, (pts, layer, cat) in enumerate(segments):
        groups[(layer, cat)].append(i)

    result = []

    for (layer, cat), indices in groups.items():
        # Build endpoint lookup: snap_key -> list of seg indices (head or tail)
        head_map = defaultdict(list)  # key -> [seg_idx, ...]  (key is head of that seg)
        tail_map = defaultdict(list)  # key -> [seg_idx, ...]  (key is tail of that seg)
        segs = {}

        for i in indices:
            pts = segments[i][0]
            if len(pts) < 2:
                result.append(segments[i])
                continue
            segs[i] = list(pts)
            head_map[snap_key(pts[0])].append(i)
            tail_map[snap_key(pts[-1])].append(i)

        used = set()

        for start_i in sorted(segs.keys()):
            if start_i in used:
                continue
            used.add(start_i)
            chain = list(segs[start_i])

            # Extend forward (from chain tail)
            changed = True
            while changed:
                changed = False
                tk = snap_key(chain[-1])

                # Head of another seg matches our tail
                for ni in head_map.get(tk, []):
                    if ni not in used:
                        used.add(ni)
                        chain.extend(segs[ni][1:])
                        changed = True
                        break
                if changed:
                    continue

                # Tail of another seg matches our tail (reverse it)
                for ni in tail_map.get(tk, []):
                    if ni not in used:
                        used.add(ni)
                        chain.extend(list(reversed(segs[ni]))[1:])
                        changed = True
                        break

            # Extend backward (from chain head)
            changed = True
            while changed:
                changed = False
                hk = snap_key(chain[0])

                # Tail of another seg matches our head
                for ni in tail_map.get(hk, []):
                    if ni not in used:
                        used.add(ni)
                        chain = list(segs[ni]) + chain[1:]
                        changed = True
                        break
                if changed:
                    continue

                # Head of another seg matches our head (reverse it)
                for ni in head_map.get(hk, []):
                    if ni not in used:
                        used.add(ni)
                        chain = list(reversed(segs[ni])) + chain[1:]
                        changed = True
                        break

            result.append((chain, layer, cat))

    return result


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


def polyline_dist(pts, from_idx, to_idx):
    """Sum of segment-by-segment 2D distances along a polyline from from_idx to to_idx."""
    d = 0.0
    for k in range(from_idx, to_idx):
        d += dist_2d(pts[k], pts[k + 1])
    return d


def interpolate_z(idx, pts_with_z):
    """
    Linearly interpolate Z for vertex at idx from surrounding vertices that have Z.
    Uses distance along the polyline (not straight-line) for accurate interpolation.
    Falls back to nearest known Z on the same polyline.
    """
    n = len(pts_with_z)

    # Find previous vertex with Z
    prev_z, prev_idx = None, None
    for i in range(idx - 1, -1, -1):
        if pts_with_z[i][2] is not None and pts_with_z[i][2] != 0.0:
            prev_z = pts_with_z[i][2]
            prev_idx = i
            break

    # Find next vertex with Z
    next_z, next_idx = None, None
    for i in range(idx + 1, n):
        if pts_with_z[i][2] is not None and pts_with_z[i][2] != 0.0:
            next_z = pts_with_z[i][2]
            next_idx = i
            break

    if prev_z is not None and next_z is not None:
        # Linear interpolation along polyline path
        d_total = polyline_dist(pts_with_z, prev_idx, next_idx)
        if d_total < 1e-6:
            return prev_z
        d_part = polyline_dist(pts_with_z, prev_idx, idx)
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
        # Step 1: Assign nearest Z to each vertex (None if no nearby point)
        pts_with_z = []
        for vx, vy in verts_2d:
            z = find_nearest_z((vx, vy), points_3d, search_radius)
            pts_with_z.append((vx, vy, z))

        # Step 2: Interpolate missing Z values along polyline path
        for i in range(len(pts_with_z)):
            if pts_with_z[i][2] is None:
                interp = interpolate_z(i, pts_with_z)
                if interp is not None:
                    pts_with_z[i] = (pts_with_z[i][0], pts_with_z[i][1], interp)

        # Step 3: Fill any remaining None with nearest known-Z vertex on same feature
        for i in range(len(pts_with_z)):
            if pts_with_z[i][2] is None:
                best_z = None
                best_d = 1e99
                for j, (ox, oy, oz) in enumerate(pts_with_z):
                    if oz is not None and i != j:
                        d = dist_2d(pts_with_z[i], (ox, oy))
                        if d < best_d:
                            best_d = d
                            best_z = oz
                if best_z is not None:
                    pts_with_z[i] = (pts_with_z[i][0], pts_with_z[i][1], best_z)

        # Step 4: Finalize — replace any remaining None Z with 0.0
        pts_with_z = [
            (x, y, z if z is not None else 0.0) for x, y, z in pts_with_z
        ]

        # Step 5: Create 3D polyline
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
    Collect building outline polylines, including those deeply nested inside blocks.
    H-PLOT OUTLINE INNER polylines can be 2+ levels deep inside house/garage blocks.
    Returns list of (vertices, layer_name).
    """
    outlines = []
    seen = set()  # Deduplicate by centroid (handles duplicate INSERT references)

    def add_outline(verts, layer):
        # Use centroid as a simple deduplication key
        if len(verts) < 3:
            return
        cx = round(sum(v[0] for v in verts) / len(verts), 3)
        cy = round(sum(v[1] for v in verts) / len(verts), 3)
        key = (cx, cy)
        if key not in seen:
            seen.add(key)
            outlines.append((verts, layer))

    # Direct polylines on building layers
    for entity in msp:
        if entity.dxftype() == "LWPOLYLINE":
            if classify_layer(entity.dxf.layer) == "building":
                verts = [(x, y) for x, y, *_ in entity.get_points()]
                add_outline(verts, entity.dxf.layer)

    # Recursively explode INSERT entities on building layers
    for entity in msp:
        if entity.dxftype() != "INSERT":
            continue
        if classify_layer(entity.dxf.layer) != "building":
            continue

        for sub in iter_virtual_deep(entity):
            if sub.dxftype() != "LWPOLYLINE":
                continue
            sl = sub.dxf.layer
            # Accept H-PLOT OUTLINE INNER or any building-classified layer
            if "PLOT OUTLINE INNER" in sl.upper() or classify_layer(sl) == "building":
                verts = [(x, y) for x, y, *_ in sub.get_points()]
                add_outline(verts, sl)

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

    # If no text inside bounding box, try proximity (within 20m of centroid)
    if best_ffl is None:
        for (fx, fy), elev, layer, is_ffl in ffl_data:
            if not is_ffl:
                continue
            d = dist_2d((cx, cy), (fx, fy))
            if d < 20.0 and d < best_dist:
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
    no_ffl = 0
    for verts, layer in outlines:
        ffl = find_ffl_for_outline(verts, ffl_data)
        if ffl is None:
            no_ffl += 1
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

    if no_ffl:
        print(f"  Warning: {no_ffl} outlines skipped (no FFL text found nearby)")
    print(f"  Created {count} building pads on '{LAYER_3D_BUILDING_PADS}'")
    return count


# ---------------------------------------------------------------------------
# Setup output layers / linetype cleanup
# ---------------------------------------------------------------------------

def cleanup_xref_linetypes(doc):
    """
    Fix or remove xref-dependent linetypes (names containing '|') that aren't
    properly flagged. Civil 3D rejects these with 'LTYPE Table' errors.
    """
    linetypes_to_remove = []
    for lt in doc.linetypes:
        if '|' in lt.dxf.name:
            linetypes_to_remove.append(lt.dxf.name)

    if not linetypes_to_remove:
        return

    # Reassign any layers using these linetypes to "Continuous"
    for layer in doc.layers:
        lt_name = layer.dxf.get('linetype', '')
        if lt_name in linetypes_to_remove:
            layer.dxf.linetype = "Continuous"

    # Remove the problematic linetypes
    for name in linetypes_to_remove:
        try:
            doc.linetypes.remove(name)
        except Exception:
            # If removal fails, fix the flags instead (set xref-dependent flag 16)
            try:
                lt = doc.linetypes.get(name)
                lt.dxf.flags = lt.dxf.flags | 16
            except Exception:
                pass

    print(f"  Cleaned up {len(linetypes_to_remove)} xref-dependent linetypes")


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


def set_units_metres(doc):
    """Set DXF drawing units to metres (INSUNITS=6, MEASUREMENT=1)."""
    doc.header['$INSUNITS'] = 6    # 6 = Metres
    doc.header['$MEASUREMENT'] = 1  # 1 = Metric


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
    print("[1/8] Loading DXF file...")
    doc = ezdxf.readfile(input_path)
    msp = doc.modelspace()

    entity_count = len(list(msp))
    layer_count = len(doc.layers)
    print(f"  Loaded: {entity_count} entities, {layer_count} layers\n")

    # Set output units to metres
    print("[2/8] Setting drawing units to metres...")
    set_units_metres(doc)
    print()

    # Cleanup xref-dependent linetypes (Civil 3D rejects these)
    print("[3/8] Cleaning up xref-dependent linetypes...")
    cleanup_xref_linetypes(doc)
    print()

    # Setup output layers
    print("[4/8] Setting up output layers...")
    setup_output_layers(doc)
    print()

    # Phase 1: Extract elevation text and create 3D points
    print("[5/8] Extracting elevation text and creating 3D points...")
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

    # Phase 2: Collect geometry, insert intermediate vertices, chain, convert to 3D
    print("[6/8] Collecting line/arc geometry...")
    raw_segments = collect_line_geometry(msp, ["drive", "fence", "road"])
    print()

    print("[7/8] Inserting intermediate vertices and chaining segments...")
    segments = insert_spot_vertices(raw_segments, elevation_data, search_radius)
    segments = chain_segments(segments)
    print(f"  Chained into {len(segments)} continuous polylines")
    convert_lines_to_3d(msp, segments, points_3d, search_radius)
    print()

    # Phase 3: Building pads
    print("[8/8] Creating building pads...")
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
