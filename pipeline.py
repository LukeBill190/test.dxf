#!/usr/bin/env python3
"""
External Works DXF → 3D Model Pipeline (v2)

Reads a 2D external works DXF file and produces a 3D DXF with:
  PLOTS FFL            - Building pad boundaries inset 25 mm, all vertices at FFL Z
  PLOTS EXTERNAL LEVEL - Building pad external outlines with per-vertex ground Z
  3D_LINES             - All other line/arc/polyline geometry elevated from terrain
  3D_Points            - Spot level POINT entities (reference layer)

Key behaviours
  - Adjacent plots that share a single FFL annotation are merged into one polygon
  - The FFL boundary is the merged polygon inset by FFL_OFFSET (25 mm)
  - The external-level boundary is the merged polygon with Z interpolated from spot levels
  - Every LINE / ARC / LWPOLYLINE / POLYLINE that is not a building outline is
    elevated from the terrain model and written to 3D_LINES
  - Output is written to a fresh DXF with only the 4 output layers defined
"""

import sys
import re
import math
import ezdxf
from collections import defaultdict, deque
from shapely.geometry import Polygon, Point, MultiPolygon
from shapely.ops import unary_union


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# Layer patterns that identify building plot outlines (case-insensitive substring)
# NOTE: party-wall layers are excluded here — they are used only to suppress those
# entities from 3D_LINES, not as a source of plot outline geometry.
BUILDING_OUTLINE_LAYERS = ["PLOT OUTLINE INNER"]   # source for pad geometry only
BUILDING_KEYWORDS       = ["plot", "building", "house", "garage"]
BUILDING_SUPPRESS_LAYERS = ["EXTERNAL PARTY WALL"]  # exclude from 3D_LINES but not pads

# Layer keyword classification for 3D_LINES geometry selection
DRIVE_KEYWORDS  = ["drive", "path"]
FENCE_KEYWORDS  = ["fence", "wall", "boundary"]
ROAD_KEYWORDS   = ["road", "footpath", "kerb"]

# Elevation text source layers
SPOT_LEVEL_LAYERS = ["LR SPOT LEVEL", "L018 HA_ANN_FEAT_TEXT"]
FFL_LAYERS        = ["LR LLFA FFL"]
DPC_LAYERS        = ["LR DPC LEVEL"]

# Regex patterns for elevation text
ELEVATION_PATTERN = re.compile(r"[+]?\s*(\d{2,3}\.\d{1,4})\s*[+]?")
FFL_PATTERN       = re.compile(r"FFL[\s\\P]*(\d{2,3}\.\d{1,4})", re.IGNORECASE)

# Output layer names (must match reference exactly)
LAYER_PLOTS_FFL      = "PLOTS FFL"
LAYER_PLOTS_EXTERNAL = "PLOTS EXTERNAL LEVEL"
LAYER_3D_LINES       = "3D_LINES"
LAYER_3D_POINTS      = "3D_Points"

# Building pad FFL inset (metres)
FFL_OFFSET = 0.025   # 25 mm

# Tolerances
CHAIN_SNAP_TOL      = 0.05   # 50 mm: endpoint match when chaining segments
VERTEX_INSERT_RADIUS = 0.3   # 300 mm: snap radius for spot-level vertex insertion
ADJACENCY_TOL       = 0.35   # 350 mm: plots within this distance are adjacent (crosses party walls)
TERRAIN_SEARCH_RAD  = 5.0    # 5 m: radius for terrain Z lookup

# Building pad merge parameters
WALL_BUFFER     = 0.205  # 205 mm: wall thickness buffer; expands inner→outer boundary
SIMPLIFY_TOL    = 0.20   # 200 mm: polygon simplification after merge (for clean vertex counts)

# Arc tessellation: max chord length (metres)
ARC_CHORD_MAX = 0.1


# ---------------------------------------------------------------------------
# Utility helpers
# ---------------------------------------------------------------------------

def dist_2d(a, b):
    """2-D Euclidean distance (ignores Z)."""
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def _is_pad_source_layer(layer_name):
    """True if this layer is the source of building plot outline geometry (H-PLOT OUTLINE INNER etc.)."""
    lu = layer_name.upper()
    for pat in BUILDING_OUTLINE_LAYERS:
        if pat.upper() in lu:
            return True
    return False


def _is_suppress_from_lines(layer_name):
    """True if this layer should be excluded from 3D_LINES output."""
    lu = layer_name.upper()
    for kw in BUILDING_KEYWORDS:
        if kw.upper() in lu:
            return True
    for pat in BUILDING_OUTLINE_LAYERS + BUILDING_SUPPRESS_LAYERS:
        if pat.upper() in lu:
            return True
    return False


def classify_layer(layer_name):
    """Classify a layer as drive / fence / road, or return None."""
    lu = layer_name.upper()
    for kw in DRIVE_KEYWORDS:
        if kw.upper() in lu:
            return "drive"
    for kw in FENCE_KEYWORDS:
        if kw.upper() in lu:
            return "fence"
    for kw in ROAD_KEYWORDS:
        if kw.upper() in lu:
            return "road"
    return None


def parse_elevation(text_value, is_ffl=False):
    """Extract a numeric elevation from a text string. Returns float or None."""
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
        if 0 < val < 500:   # sanity: UK AOD levels
            return val
    return None


def get_text_value(entity):
    if entity.dxftype() == "TEXT":
        return entity.dxf.text
    if entity.dxftype() == "MTEXT":
        return entity.plain_text()
    return None


def get_text_insertion(entity):
    if entity.dxftype() in ("TEXT", "MTEXT"):
        pt = entity.dxf.insert
        return (pt.x, pt.y)
    return None


def tessellate_arc(entity):
    """Convert an ARC entity into a list of (x, y) points."""
    cx, cy = entity.dxf.center.x, entity.dxf.center.y
    r = entity.dxf.radius
    sa = math.radians(entity.dxf.start_angle)
    ea = math.radians(entity.dxf.end_angle)
    if ea <= sa:
        ea += 2 * math.pi
    arc_len = r * (ea - sa)
    n = max(4, int(math.ceil(arc_len / ARC_CHORD_MAX)))
    return [
        (cx + r * math.cos(sa + (ea - sa) * i / n),
         cy + r * math.sin(sa + (ea - sa) * i / n))
        for i in range(n + 1)
    ]


def iter_virtual_deep(insert_entity, max_depth=6):
    """Yield all virtual entities from a nested INSERT, recursively."""
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
# Phase 1 – Collect elevation text and build terrain point cloud
# ---------------------------------------------------------------------------

def collect_elevation_text(msp):
    """
    Scan modelspace (and INSERT blocks) for TEXT / MTEXT on elevation layers.
    Returns list of ((x, y), elevation, layer, is_ffl).
    """
    results = []

    def process_text(entity):
        layer = entity.dxf.layer.upper()
        text_val = get_text_value(entity)
        insert_pt = get_text_insertion(entity)
        if text_val is None or insert_pt is None:
            return
        is_ffl  = any(fl.upper() in layer for fl in FFL_LAYERS)
        is_spot = any(sl.upper() in layer for sl in SPOT_LEVEL_LAYERS)
        is_dpc  = any(dl.upper() in layer for dl in DPC_LAYERS)
        if is_ffl or is_spot or is_dpc:
            elev = parse_elevation(text_val, is_ffl=is_ffl)
            if elev is not None:
                results.append((insert_pt, elev, entity.dxf.layer, is_ffl))

    for entity in msp:
        if entity.dxftype() in ("TEXT", "MTEXT"):
            process_text(entity)
        elif entity.dxftype() == "INSERT":
            for sub in iter_virtual_deep(entity):
                if sub.dxftype() in ("TEXT", "MTEXT"):
                    process_text(sub)

    return results


def build_terrain_points(elevation_data):
    """
    Convert elevation data into a flat list of (x, y, z) terrain points.
    FFL annotations are included (they are valid ground elevations too).
    """
    return [(xy[0], xy[1], elev) for xy, elev, _layer, _is_ffl in elevation_data]


# ---------------------------------------------------------------------------
# Phase 2 – Terrain Z lookup and interpolation helpers
# ---------------------------------------------------------------------------

def find_nearest_z(px, py, points_3d, radius):
    """Return the Z of the nearest terrain point within radius, or None."""
    best_z, best_d = None, radius
    for x, y, z in points_3d:
        d = math.sqrt((px - x) ** 2 + (py - y) ** 2)
        if d < best_d:
            best_d = d
            best_z = z
    return best_z


def _polyline_dist(pts, from_idx, to_idx):
    d = 0.0
    for k in range(from_idx, to_idx):
        d += dist_2d(pts[k], pts[k + 1])
    return d


def interpolate_z_along(pts_with_z, idx):
    """
    Linearly interpolate Z for vertex idx from the nearest known-Z neighbours
    along the polyline. Falls back to nearest known Z.
    """
    n = len(pts_with_z)
    prev_z = prev_i = None
    for i in range(idx - 1, -1, -1):
        if pts_with_z[i][2] is not None:
            prev_z, prev_i = pts_with_z[i][2], i
            break
    next_z = next_i = None
    for i in range(idx + 1, n):
        if pts_with_z[i][2] is not None:
            next_z, next_i = pts_with_z[i][2], i
            break
    if prev_z is not None and next_z is not None:
        d_total = _polyline_dist(pts_with_z, prev_i, next_i)
        if d_total < 1e-6:
            return prev_z
        d_part = _polyline_dist(pts_with_z, prev_i, idx)
        return prev_z + (d_part / d_total) * (next_z - prev_z)
    return prev_z if prev_z is not None else next_z


def assign_z_to_vertices(verts_2d, points_3d, search_radius=TERRAIN_SEARCH_RAD):
    """
    Assign Z to each 2-D vertex from the terrain point cloud.
    Gaps are filled by polyline interpolation, then nearest neighbour.
    Remaining unknowns fall back to 0.0.
    Returns list of (x, y, z).
    """
    # Pass 1: direct nearest-point lookup
    pts = [(x, y, find_nearest_z(x, y, points_3d, search_radius))
           for x, y in verts_2d]

    # Pass 2: interpolate along polyline for missing
    for i in range(len(pts)):
        if pts[i][2] is None:
            z = interpolate_z_along(pts, i)
            if z is not None:
                pts[i] = (pts[i][0], pts[i][1], z)

    # Pass 3: nearest-neighbour on same feature for remaining gaps
    for i in range(len(pts)):
        if pts[i][2] is None:
            best_z, best_d = None, 1e99
            for j, (ox, oy, oz) in enumerate(pts):
                if oz is not None and i != j:
                    d = dist_2d(pts[i], (ox, oy))
                    if d < best_d:
                        best_d, best_z = d, oz
            if best_z is not None:
                pts[i] = (pts[i][0], pts[i][1], best_z)

    # Pass 4: fallback 0.0
    return [(x, y, z if z is not None else 0.0) for x, y, z in pts]


def assign_z_to_ring(ring_2d, points_3d, search_radius=TERRAIN_SEARCH_RAD):
    """
    Like assign_z_to_vertices but treats the input as a closed ring —
    wraps around for interpolation so the last vertex can borrow from
    the first and vice-versa.
    """
    # Duplicate ring for wrap-around interpolation
    ring = list(ring_2d)
    closed = (len(ring) >= 2 and dist_2d(ring[0], ring[-1]) < 1e-6)
    if closed:
        ring = ring[:-1]   # strip repeated closing vertex

    # Direct lookup
    pts = [(x, y, find_nearest_z(x, y, points_3d, search_radius))
           for x, y in ring]

    n = len(pts)
    if n == 0:
        return []

    # Interpolate on the extended ring (pad with wrap-around)
    extended = pts[-n:] + pts + pts[:n]   # [wrap_back | actual | wrap_fwd]
    offset = n

    for i in range(n):
        if pts[i][2] is not None:
            continue
        ei = i + offset
        # Search prev in extended
        prev_z = prev_ei = None
        for k in range(ei - 1, ei - n, -1):
            if extended[k][2] is not None:
                prev_z, prev_ei = extended[k][2], k
                break
        next_z = next_ei = None
        for k in range(ei + 1, ei + n):
            if extended[k][2] is not None:
                next_z, next_ei = extended[k][2], k
                break
        if prev_z is not None and next_z is not None:
            d_total = _polyline_dist(extended, prev_ei, next_ei)
            if d_total > 1e-6:
                d_part = _polyline_dist(extended, prev_ei, ei)
                pts[i] = (pts[i][0], pts[i][1],
                          prev_z + (d_part / d_total) * (next_z - prev_z))
            else:
                pts[i] = (pts[i][0], pts[i][1], prev_z)
        elif prev_z is not None:
            pts[i] = (pts[i][0], pts[i][1], prev_z)
        elif next_z is not None:
            pts[i] = (pts[i][0], pts[i][1], next_z)

    # Final fallback: mean of known Z values
    known_z = [z for _, _, z in pts if z is not None]
    fallback = sum(known_z) / len(known_z) if known_z else 0.0
    pts = [(x, y, z if z is not None else fallback) for x, y, z in pts]

    # Restore closed vertex
    if closed:
        pts.append(pts[0])
    return pts


# ---------------------------------------------------------------------------
# Phase 3 – Collect all line geometry (everything that is not a building outline)
# ---------------------------------------------------------------------------

def _extract_verts(entity):
    """
    Return the 2-D vertex list for a single LINE / LWPOLYLINE / ARC / POLYLINE
    entity. Returns [] if entity type is not handled or has < 2 vertices.
    """
    t = entity.dxftype()
    if t == "LINE":
        s, e = entity.dxf.start, entity.dxf.end
        return [(s.x, s.y), (e.x, e.y)]
    if t == "LWPOLYLINE":
        return [(x, y) for x, y, *_ in entity.get_points()]
    if t == "POLYLINE":
        verts = []
        for v in entity.vertices:
            p = v.dxf.location
            verts.append((p.x, p.y))
        return verts
    if t == "ARC":
        return tessellate_arc(entity)
    return []


def collect_line_geometry(msp):
    """
    Collect LINE / LWPOLYLINE / ARC / POLYLINE entities on drive, fence and road
    layers (matching DRIVE_KEYWORDS, FENCE_KEYWORDS, ROAD_KEYWORDS).
    Building / party-wall layers are excluded.

    Returns list of (vertices_2d, layer_name).
    """
    results = []

    def add(entity):
        layer = entity.dxf.layer
        if _is_suppress_from_lines(layer):
            return
        if classify_layer(layer) is None:
            return
        verts = _extract_verts(entity)
        if len(verts) >= 2:
            results.append((verts, layer))

    for entity in msp:
        if entity.dxftype() == "INSERT":
            for sub in iter_virtual_deep(entity):
                add(sub)
        else:
            add(entity)

    return results


# ---------------------------------------------------------------------------
# Phase 4 – Insert intermediate spot-level vertices, chain, elevate → 3D_LINES
# ---------------------------------------------------------------------------

def insert_spot_vertices(segments, terrain_pts, search_radius=5.0):
    """
    For each terrain point that has no existing vertex within
    VERTEX_INSERT_RADIUS on any segment, split the nearest segment there.
    Returns updated segment list with the same (verts, layer) structure.
    """
    if not segments or not terrain_pts:
        return segments

    pts_list = [list(v) for v, _ in segments]
    layers   = [l for _, l in segments]
    added    = 0

    for tx, ty, _tz in terrain_pts:
        # Skip if a vertex already exists nearby
        nearby = any(
            dist_2d((tx, ty), vt) <= VERTEX_INSERT_RADIUS
            for verts in pts_list for vt in verts
        )
        if nearby:
            continue

        # Find nearest edge within TERRAIN_SEARCH_RAD
        best_dist = TERRAIN_SEARCH_RAD
        best_seg = best_edge = best_t = None

        for si, pts in enumerate(pts_list):
            for ei in range(len(pts) - 1):
                ax, ay = pts[ei]
                bx, by = pts[ei + 1]
                dx, dy = bx - ax, by - ay
                len_sq = dx * dx + dy * dy
                if len_sq < 1e-12:
                    continue
                t = max(0.0, min(1.0, ((tx - ax) * dx + (ty - ay) * dy) / len_sq))
                nx, ny = ax + t * dx, ay + t * dy
                d = math.sqrt((tx - nx) ** 2 + (ty - ny) ** 2)
                if d < best_dist:
                    best_dist = d
                    best_seg, best_edge, best_t = si, ei, t

        if best_seg is not None and 1e-4 < best_t < 1.0 - 1e-4:
            pts = pts_list[best_seg]
            ax, ay = pts[best_edge]
            bx, by = pts[best_edge + 1]
            new_pt = (ax + best_t * (bx - ax), ay + best_t * (by - ay))
            pts_list[best_seg] = pts[:best_edge + 1] + [new_pt] + pts[best_edge + 1:]
            added += 1

    print(f"  Inserted {added} intermediate vertices")
    return list(zip(pts_list, layers))


def chain_segments(segments, snap_tol=CHAIN_SNAP_TOL):
    """
    Join segments that share endpoints (within snap_tol) into continuous
    polylines, grouped by layer.
    Returns list of (point_list, layer_name).
    """
    if not segments:
        return segments

    def snap_key(pt):
        f = 1.0 / snap_tol
        return (round(pt[0] * f), round(pt[1] * f))

    groups = defaultdict(list)
    for i, (pts, layer) in enumerate(segments):
        groups[layer].append(i)

    result = []

    for layer, indices in groups.items():
        head_map = defaultdict(list)
        tail_map = defaultdict(list)
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

        for start_i in sorted(segs):
            if start_i in used:
                continue
            used.add(start_i)
            chain = list(segs[start_i])

            # Extend forward from chain tail
            for _ in range(10000):
                tk = snap_key(chain[-1])
                matched = False
                for ni in head_map.get(tk, []):
                    if ni not in used:
                        used.add(ni); chain.extend(segs[ni][1:]); matched = True; break
                if matched:
                    continue
                for ni in tail_map.get(tk, []):
                    if ni not in used:
                        used.add(ni); chain.extend(list(reversed(segs[ni]))[1:]); matched = True; break
                if not matched:
                    break

            # Extend backward from chain head
            for _ in range(10000):
                hk = snap_key(chain[0])
                matched = False
                for ni in tail_map.get(hk, []):
                    if ni not in used:
                        used.add(ni); chain = list(segs[ni]) + chain[1:]; matched = True; break
                if matched:
                    continue
                for ni in head_map.get(hk, []):
                    if ni not in used:
                        used.add(ni); chain = list(reversed(segs[ni])) + chain[1:]; matched = True; break
                if not matched:
                    break

            result.append((chain, layer))

    return result


def elevate_line_geometry(segments, terrain_pts):
    """
    Assign Z to every vertex in every segment from the terrain model.
    Returns list of ([(x,y,z), ...], layer_name).
    """
    result = []
    for verts_2d, layer in segments:
        verts_3d = assign_z_to_vertices(verts_2d, terrain_pts, TERRAIN_SEARCH_RAD)
        if len(verts_3d) >= 2:
            result.append((verts_3d, layer))
    return result


# ---------------------------------------------------------------------------
# Phase 5 – Building pads: collect outlines → merge by FFL → PLOTS FFL +
#            PLOTS EXTERNAL LEVEL
# ---------------------------------------------------------------------------

def _chain_lines_to_outlines(lines, snap_tol=0.05):
    """Chain ((x1,y1),(x2,y2)) LINE pairs into closed polygons."""
    seen = set()
    unique = []
    for a, b in lines:
        key = (round(min(a[0], b[0]), 3), round(min(a[1], b[1]), 3),
               round(max(a[0], b[0]), 3), round(max(a[1], b[1]), 3))
        if key not in seen:
            seen.add(key); unique.append((a, b))

    if not unique:
        return []

    def snap(p):
        f = 1.0 / snap_tol
        return (round(p[0] * f), round(p[1] * f))

    head_map = defaultdict(list)
    tail_map = defaultdict(list)
    segs = {}
    for i, (a, b) in enumerate(unique):
        segs[i] = [a, b]
        head_map[snap(a)].append(i)
        tail_map[snap(b)].append(i)

    used = set()
    closed_chains = []

    for start in sorted(segs):
        if start in used:
            continue
        used.add(start)
        chain = list(segs[start])
        for _ in range(500):
            tk = snap(chain[-1])
            found = False
            for ni in head_map.get(tk, []):
                if ni not in used:
                    used.add(ni); chain.extend(segs[ni][1:]); found = True; break
            if not found:
                for ni in tail_map.get(tk, []):
                    if ni not in used:
                        used.add(ni); chain.extend(list(reversed(segs[ni]))[1:]); found = True; break
            if not found:
                break
        if len(chain) >= 4 and dist_2d(chain[0], chain[-1]) <= snap_tol:
            closed_chains.append(chain)

    return closed_chains


def _apply_insert_xform(bx, by, insert_x, insert_y, sx, sy, rot_deg):
    """
    Apply a DXF INSERT transformation (scale → rotate → translate) to a 2-D point.
    Uses the standard DXF formula so it is correct even for negative-scale (mirrored) blocks.
    """
    rot = math.radians(rot_deg)
    cos_r = math.cos(rot)
    sin_r = math.sin(rot)
    wx = insert_x + sx * cos_r * bx - sy * sin_r * by
    wy = insert_y + sx * sin_r * bx + sy * cos_r * by
    return wx, wy


def _block_outline_verts(doc, insert_entity):
    """
    Walk a single INSERT and its nested INSERTs, collecting building outline
    polygons from PLOT OUTLINE INNER layers.  Transformations are applied
    manually (avoids the ezdxf negative-scale transformation bug).

    Two cases are handled:
    1. Block has a closed LWPOLYLINE (≥3 pts) on the pad-source layer
       → collect it directly.
    2. Block only has 2-pt LWPOLYLINE segments (outer-wall pieces) on
       PLOT OUTLINE INNER, and LINE entities on H-EXTERNAL PARTY WALL
       (middle-of-terrace "MID" block types).  In this case the convex hull
       of all segment endpoints is used to reconstruct the floor-area polygon.

    Returns list of vertex lists, each already in world coordinates.
    """
    results = []

    def recurse(ins, parent_xform):
        """parent_xform = (ins_x, ins_y, sx, sy, rot_deg) of the chain so far."""
        try:
            block = doc.blocks.get(ins.dxf.name)
        except Exception:
            return
        if block is None:
            return

        sx_ins  = ins.dxf.get("xscale", 1.0)
        sy_ins  = ins.dxf.get("yscale", 1.0)
        rot_ins = ins.dxf.get("rotation", 0.0)
        tx_ins  = ins.dxf.insert.x
        ty_ins  = ins.dxf.insert.y

        closed_found = False   # True if any ≥3-pt LWPOLYLINE was collected
        seg_pts = []           # endpoints from 2-pt segments / LINEs (MID case)

        for entity in block:
            try:
                lyr = entity.dxf.layer
            except Exception:
                lyr = ""

            is_inner  = _is_pad_source_layer(lyr)
            # Party-wall LINEs are needed to close the outline for MID blocks.
            # Use H-EXTERNAL PARTY WALL (not the 352 variant, which is the
            # outer dimensioned line and would skew the outline slightly).
            is_party  = ("EXTERNAL PARTY WALL" in lyr.upper()
                         and "352" not in lyr.upper())

            if entity.dxftype() == "LWPOLYLINE" and is_inner:
                raw = [(x, y) for x, y, *_ in entity.get_points()]
                step1 = [_apply_insert_xform(bx, by, tx_ins, ty_ins,
                                              sx_ins, sy_ins, rot_ins)
                         for bx, by in raw]
                px, py, psx, psy, prot = parent_xform
                step2 = [_apply_insert_xform(wx, wy, px, py, psx, psy, prot)
                         for wx, wy in step1]
                if len(step2) >= 3:
                    results.append(step2)
                    closed_found = True
                else:
                    # 2-pt segment: collect endpoints for convex-hull fallback
                    seg_pts.extend(step2)

            elif entity.dxftype() == "LINE" and (is_inner or is_party):
                for pt_local in [(entity.dxf.start.x, entity.dxf.start.y),
                                 (entity.dxf.end.x,   entity.dxf.end.y)]:
                    s1 = _apply_insert_xform(pt_local[0], pt_local[1],
                                             tx_ins, ty_ins,
                                             sx_ins, sy_ins, rot_ins)
                    px, py, psx, psy, prot = parent_xform
                    s2 = _apply_insert_xform(s1[0], s1[1], px, py, psx, psy, prot)
                    seg_pts.append(s2)

            elif entity.dxftype() == "INSERT":
                recurse(entity, (tx_ins, ty_ins, sx_ins, sy_ins, rot_ins))

        # MID-block fallback: build convex hull from all segment endpoints
        if not closed_found and len(seg_pts) >= 3:
            try:
                from shapely.geometry import MultiPoint
                hull = MultiPoint(seg_pts).convex_hull
                if hull.geom_type == "Polygon" and not hull.is_empty:
                    results.append(list(hull.exterior.coords))
            except Exception:
                pass

    # The top-level INSERT has no parent transform (identity)
    recurse(insert_entity, (0.0, 0.0, 1.0, 1.0, 0.0))
    return results


def collect_building_outlines(msp, doc):
    """
    Collect building plot outlines as (vertices_2d, layer_name) pairs.
    Only H-PLOT OUTLINE INNER (and similar BUILDING_OUTLINE_LAYERS) LWPOLYLINEs
    are used; party-wall LINE entities are ignored.
    Transformations for nested blocks are applied manually to avoid the ezdxf
    negative-scale (mirrored INSERT) transformation bug.
    """
    outlines = []
    seen = set()

    def add(verts, layer):
        if len(verts) < 3:
            return
        cx = round(sum(v[0] for v in verts) / len(verts), 2)
        cy = round(sum(v[1] for v in verts) / len(verts), 2)
        if (cx, cy) not in seen:
            seen.add((cx, cy))
            outlines.append((verts, layer))

    # Direct LWPOLYLINE entities in modelspace
    for entity in msp:
        if entity.dxftype() == "LWPOLYLINE" and _is_pad_source_layer(entity.dxf.layer):
            add([(x, y) for x, y, *_ in entity.get_points()], entity.dxf.layer)

    # Geometry inside INSERT blocks (all INSERT entities, any layer)
    for entity in msp:
        if entity.dxftype() != "INSERT":
            continue
        for verts in _block_outline_verts(doc, entity):
            add(verts, BUILDING_OUTLINE_LAYERS[0])

    return outlines


def _to_shapely_polygon(verts):
    """Convert a vertex list to a Shapely Polygon. Returns None on failure."""
    try:
        poly = Polygon(verts)
        if not poly.is_valid:
            poly = poly.buffer(0)   # fix self-intersections
        return poly if not poly.is_empty else None
    except Exception:
        return None


def assign_ffl_and_merge(outlines, ffl_annotations):
    """
    Group plot outlines by their FFL value and union adjacent same-FFL plots.

    ffl_annotations: list of ((x, y), ffl_value)

    Returns list of (shapely_polygon, ffl_value).

    Algorithm:
      Pass 1 – assign the FFL annotation to the nearest polygon (within FFL_PROXIMITY).
               Ties broken by distance; each annotation claims at most one polygon,
               each polygon keeps only its first (closest) assignment.
      Pass 2 – BFS flood-fill: propagates FFL outward from annotated polygons
               to unassigned neighbours within ADJACENCY_TOL, in geographic order
               (nearest-first), so that each plot gets the FFL of the nearest
               annotated polygon reachable through adjacency.
      Pass 3 – narrow fallback: any polygon still unassigned is given the FFL
               of the nearest annotation within FFL_PROXIMITY (5 m).  Polygons
               more than 5 m from every annotation are silently dropped.
    Merging:
      Within each FFL group, plots are buffered by WALL_BUFFER (≈ wall thickness)
      so that adjacent inner-floor-area polygons touch, then unioned.  The result
      is simplified to give clean low-vertex-count polygons that approximate the
      outer wall boundary.  No convex-hull bridging is used: that produced
      high-vertex artefacts that did not match the reference geometry.
    """
    # Build shapely polygons
    polys = []
    for verts, _layer in outlines:
        poly = _to_shapely_polygon(verts)
        if poly is not None and not poly.is_empty:
            polys.append(poly)

    if not polys:
        return []

    n = len(polys)
    ffl_assigned = [None] * n

    # ------------------------------------------------------------------
    # Pass 1 – assign the nearest polygon to each FFL annotation
    # ------------------------------------------------------------------
    FFL_PROXIMITY = 5.0   # metres — annotation may sit in road/garden between plots
    for (fx, fy), ffl_val in ffl_annotations:
        pt = Point(fx, fy)
        best_i, best_d = None, FFL_PROXIMITY
        for i, poly in enumerate(polys):
            if poly.contains(pt):
                d = 0.0
            else:
                d = poly.distance(pt)
            if d < best_d:
                best_d, best_i = d, i
        if best_i is not None and ffl_assigned[best_i] is None:
            ffl_assigned[best_i] = ffl_val

    # ------------------------------------------------------------------
    # Pass 2 – BFS flood-fill through adjacent polygons
    # Build adjacency list first (O(n²) is fine for n≈62)
    # ------------------------------------------------------------------
    adj = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            if polys[i].distance(polys[j]) < ADJACENCY_TOL:
                adj[i].append(j)
                adj[j].append(i)

    # Initialise BFS queue with all annotated polygons
    queue = deque()
    for i in range(n):
        if ffl_assigned[i] is not None:
            queue.append(i)

    while queue:
        i = queue.popleft()
        for j in adj[i]:
            if ffl_assigned[j] is None:
                ffl_assigned[j] = ffl_assigned[i]
                queue.append(j)

    # ------------------------------------------------------------------
    # Pass 3 – narrow fallback: nearest annotation within FFL_PROXIMITY
    # ------------------------------------------------------------------
    for i in range(n):
        if ffl_assigned[i] is not None:
            continue
        cx, cy = polys[i].centroid.x, polys[i].centroid.y
        best_ffl, best_d = None, FFL_PROXIMITY
        for (fx, fy), ffl_val in ffl_annotations:
            d = dist_2d((cx, cy), (fx, fy))
            if d < best_d:
                best_d, best_ffl = d, ffl_val
        ffl_assigned[i] = best_ffl   # may remain None → dropped below

    # ------------------------------------------------------------------
    # Group and merge
    # ------------------------------------------------------------------
    groups = defaultdict(list)
    for i, ffl_val in enumerate(ffl_assigned):
        if ffl_val is not None:
            groups[ffl_val].append(polys[i])

    result = []
    for ffl_val, group_polys in groups.items():
        # Apply convex_hull to each individual inner polygon first so that
        # duplicate/near-duplicate polygons (e.g. two coincident LWPOLYLINEs
        # in the same block) don't produce complex buffer shapes.
        # Then expand by WALL_BUFFER to approximate the outer wall face and
        # union adjacent plots.  No global convex_hull is applied to the merged
        # result so that non-convex (e.g. L-shaped) terrace footprints are
        # preserved.  A gentle opening removes thin artefacts; simplification
        # reduces vertex count to match the reference (~4–7 verts).
        expanded  = [p.convex_hull.buffer(WALL_BUFFER) for p in group_polys]
        merged    = unary_union(expanded)
        # Opening: remove thin slivers produced by the buffer
        merged    = merged.buffer(-0.01).buffer(0.01)
        parts = list(merged.geoms) if merged.geom_type == "MultiPolygon" else [merged]
        for part in parts:
            if not part.is_empty:
                final = part.simplify(SIMPLIFY_TOL, preserve_topology=False)
                result.append((final if not final.is_empty else part, ffl_val))

    return result


def generate_pad_polylines(merged_pads, terrain_pts):
    """
    For each (merged_polygon, ffl_value):
      - PLOTS FFL:            exterior inset by FFL_OFFSET, all Z = ffl_value
      - PLOTS EXTERNAL LEVEL: exterior coords with Z from terrain

    Returns two lists: ffl_polylines, external_polylines.
    Each entry is a list of (x, y, z).
    """
    ffl_polys = []
    ext_polys  = []
    skipped    = 0

    for poly, ffl_val in merged_pads:
        # --- PLOTS EXTERNAL LEVEL: original exterior with terrain Z ---
        ext_ring = list(poly.exterior.coords)
        ext_3d = assign_z_to_ring(
            [(x, y) for x, y in ext_ring],
            terrain_pts,
            TERRAIN_SEARCH_RAD
        )
        if len(ext_3d) >= 2:
            ext_polys.append(ext_3d)

        # --- PLOTS FFL: inward offset, constant Z = ffl_val ---
        try:
            inner = poly.buffer(-FFL_OFFSET)
            if inner.is_empty:
                inner = poly   # degenerate — use original
        except Exception:
            inner = poly

        # Re-simplify after the inset buffer to remove spurious vertices that
        # the inward buffer can introduce on concave boundaries.
        if inner.geom_type == "Polygon":
            inner2 = inner.simplify(0.10, preserve_topology=False)
            if not inner2.is_empty:
                inner = inner2

        if inner.geom_type == "MultiPolygon":
            inner_parts = list(inner.geoms)
        elif inner.geom_type == "Polygon":
            inner_parts = [inner]
        else:
            inner_parts = [poly]

        for ip in inner_parts:
            coords = list(ip.exterior.coords)
            if len(coords) < 2:
                continue
            ffl_polys.append([(x, y, ffl_val) for x, y in coords])

    print(f"  Merged pads: {len(merged_pads)}  →  "
          f"{len(ffl_polys)} PLOTS FFL, {len(ext_polys)} PLOTS EXTERNAL LEVEL"
          + (f"  ({skipped} skipped)" if skipped else ""))
    return ffl_polys, ext_polys


# ---------------------------------------------------------------------------
# Phase 6 – Build a clean output DXF and populate it
# ---------------------------------------------------------------------------

def create_output_doc(source_doc):
    """
    Create a fresh ezdxf document with only the 4 output layers.
    Copies INSUNITS / MEASUREMENT from the source.
    """
    out = ezdxf.new("R2010")

    # Copy units
    for var in ("$INSUNITS", "$MEASUREMENT"):
        try:
            out.header[var] = source_doc.header[var]
        except Exception:
            pass

    # Define output layers
    for name, color in (
        (LAYER_PLOTS_FFL,      5),   # blue
        (LAYER_PLOTS_EXTERNAL, 6),   # magenta
        (LAYER_3D_LINES,       3),   # green
        (LAYER_3D_POINTS,      1),   # red
    ):
        out.layers.add(name, color=color)

    return out


def write_entities(out_msp, ffl_polys, ext_polys, line_polys_3d, terrain_pts_xyz):
    """Write all generated geometry to the output modelspace."""

    for verts in ffl_polys:
        out_msp.add_polyline3d(verts, dxfattribs={"layer": LAYER_PLOTS_FFL})

    for verts in ext_polys:
        out_msp.add_polyline3d(verts, dxfattribs={"layer": LAYER_PLOTS_EXTERNAL})

    for verts, _layer in line_polys_3d:
        out_msp.add_polyline3d(verts, dxfattribs={"layer": LAYER_3D_LINES})

    for x, y, z in terrain_pts_xyz:
        out_msp.add_point((x, y, z), dxfattribs={"layer": LAYER_3D_POINTS})

    print(f"  Wrote {len(ffl_polys)} PLOTS FFL polylines")
    print(f"  Wrote {len(ext_polys)} PLOTS EXTERNAL LEVEL polylines")
    print(f"  Wrote {len(line_polys_3d)} 3D_LINES polylines")
    print(f"  Wrote {len(terrain_pts_xyz)} 3D_Points")


# ---------------------------------------------------------------------------
# Xref linetype cleanup (carried forward for compatibility)
# ---------------------------------------------------------------------------

def cleanup_xref_linetypes(doc):
    """Remove xref-dependent linetypes (names containing '|')."""
    to_remove = [lt.dxf.name for lt in doc.linetypes if "|" in lt.dxf.name]
    if not to_remove:
        return
    for layer in doc.layers:
        if layer.dxf.get("linetype", "") in to_remove:
            layer.dxf.linetype = "Continuous"
    removed = 0
    for name in to_remove:
        try:
            doc.linetypes.remove(name)
            removed += 1
        except Exception:
            pass
    print(f"  Cleaned {removed} xref-dependent linetypes")


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------

def run_pipeline(input_path, output_path):
    print(f"\n{'='*60}")
    print(f"  External Works DXF → 3D Pipeline  (v2)")
    print(f"{'='*60}")
    print(f"  Input:  {input_path}")
    print(f"  Output: {output_path}")
    print(f"{'='*60}\n")

    # ── Load ─────────────────────────────────────────────────────────────────
    print("[1/8] Loading source DXF...")
    doc = ezdxf.readfile(input_path)
    msp = doc.modelspace()
    print(f"  {len(list(msp))} entities, {len(doc.layers)} layers\n")

    print("[2/8] Cleaning xref linetypes...")
    cleanup_xref_linetypes(doc)
    print()

    # ── Elevation text ───────────────────────────────────────────────────────
    print("[3/8] Collecting elevation text...")
    elevation_data = collect_elevation_text(msp)
    ffl_data  = [(xy, elev) for xy, elev, _l, is_ffl in elevation_data if is_ffl]
    spot_data = [(xy, elev) for xy, elev, _l, is_ffl in elevation_data if not is_ffl]
    print(f"  {len(ffl_data)} FFL annotations, {len(spot_data)} spot levels")
    terrain_pts = build_terrain_points(elevation_data)
    if terrain_pts:
        zs = [z for _, _, z in terrain_pts]
        print(f"  Elevation range: {min(zs):.3f} – {max(zs):.3f} m")
    print()

    # ── Line geometry ─────────────────────────────────────────────────────────
    print("[4/8] Collecting drive / fence / road line geometry...")
    raw_segs = collect_line_geometry(msp)
    print(f"  {len(raw_segs)} raw segments\n")

    print("[5/8] Inserting spot vertices, chaining, elevating...")
    segs = insert_spot_vertices(raw_segs, terrain_pts, TERRAIN_SEARCH_RAD)
    segs = chain_segments(segs)
    print(f"  {len(segs)} chained polylines")
    line_polys_3d = elevate_line_geometry(segs, terrain_pts)
    print(f"  {len(line_polys_3d)} 3D_LINES polylines\n")

    # ── Building pads ─────────────────────────────────────────────────────────
    print("[6/8] Collecting building outlines...")
    outlines = collect_building_outlines(msp, doc)
    print(f"  {len(outlines)} outlines found\n")

    print("[7/8] Merging adjacent same-FFL plots, generating pad polylines...")
    merged_pads = assign_ffl_and_merge(outlines, ffl_data)
    no_ffl = len(outlines) - sum(
        1 for _, ffl_val in [(p, f) for p, f in merged_pads]
    )
    ffl_polys, ext_polys = generate_pad_polylines(merged_pads, terrain_pts)
    print()

    # ── Write output ──────────────────────────────────────────────────────────
    print("[8/8] Writing output DXF...")
    out_doc = create_output_doc(doc)
    out_msp = out_doc.modelspace()
    write_entities(out_msp, ffl_polys, ext_polys, line_polys_3d, terrain_pts)
    out_doc.saveas(output_path)

    print(f"\n{'='*60}")
    print(f"  Done →  {output_path}")
    print(f"{'='*60}\n")


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python pipeline.py <input.dxf> [output.dxf]")
        sys.exit(1)

    input_file  = sys.argv[1]
    output_file = (sys.argv[2] if len(sys.argv) > 2
                   else input_file.replace(".dxf", "_3D_Output.dxf"))

    run_pipeline(input_file, output_file)
