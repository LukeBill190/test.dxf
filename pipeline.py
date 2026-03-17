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

try:
    import numpy as _np
    from scipy.interpolate import LinearNDInterpolator as _LNDI
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# Layer patterns that identify building plot outlines (case-insensitive substring)
# NOTE: party-wall layers are excluded here — they are used only to suppress those
# entities from 3D_LINES, not as a source of plot outline geometry.
BUILDING_OUTLINE_LAYERS = ["PLOT OUTLINE INNER"]   # source for pad geometry only (inner face)
OUTER_WALL_LAYER        = "H-EXTERNAL WALL"        # outer wall face — used directly where available
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


def build_tin(terrain_pts):
    """
    Build a Triangulated Irregular Network (TIN) surface interpolator from a
    list of (x, y, z) terrain points using Scipy's Delaunay triangulation.

    Returns a callable TIN interpolator, or None if scipy is unavailable or
    there are fewer than 3 non-collinear terrain points.

    Usage:
        tin = build_tin(terrain_pts)
        z   = query_tin(px, py, tin, terrain_pts)
    """
    if not _HAS_SCIPY or len(terrain_pts) < 3:
        return None
    try:
        xy = _np.array([(x, y) for x, y, z in terrain_pts], dtype=float)
        z  = _np.array([z      for x, y, z in terrain_pts], dtype=float)
        interp = _LNDI(xy, z)
        return interp
    except Exception:
        return None


def query_tin(px, py, tin, fallback_pts, fallback_radius=TERRAIN_SEARCH_RAD):
    """
    Return an interpolated Z at (px, py) from the TIN surface.
    Falls back to nearest-point search if:
      - tin is None (scipy not available or too few points)
      - (px, py) lies outside the convex hull of the triangulation (returns NaN)
    """
    if tin is not None:
        try:
            z = float(tin(_np.array([[px, py]]))[0])
            if not math.isnan(z):
                return z
        except Exception:
            pass
    return find_nearest_z(px, py, fallback_pts, fallback_radius)


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


def assign_z_to_ring(ring_2d, points_3d, search_radius=TERRAIN_SEARCH_RAD, tin=None):
    """
    Like assign_z_to_vertices but treats the input as a closed ring —
    wraps around for interpolation so the last vertex can borrow from
    the first and vice-versa.

    When a TIN interpolator is supplied each vertex Z is obtained from the
    triangulated surface (more accurate than nearest-point between spot levels);
    vertices outside the TIN convex hull fall back to nearest-point lookup.
    """
    # Duplicate ring for wrap-around interpolation
    ring = list(ring_2d)
    closed = (len(ring) >= 2 and dist_2d(ring[0], ring[-1]) < 1e-6)
    if closed:
        ring = ring[:-1]   # strip repeated closing vertex

    # Direct TIN (or nearest-point fallback) lookup for each vertex
    pts = [(x, y, query_tin(x, y, tin, points_3d, search_radius))
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


def assign_z_spot_owned(verts_2d, terrain_pts, search_radius=TERRAIN_SEARCH_RAD, tin=None):
    """
    Elevation model for open polylines (drives, fences, roads).

    Phase 1 – Spot-owned assignment (search_radius = TERRAIN_SEARCH_RAD = 5 m):
      Each terrain point claims the single nearest vertex within search_radius.
      Greedy assignment (shortest distance first) so each vertex receives the Z
      of its closest terrain point and each terrain point influences exactly one
      vertex — matching the survey convention that a spot level controls only
      the one polyline node closest to it.

    Phase 2 – Linear interpolation along polyline:
      Vertices between two owned vertices receive Z by linear interpolation of
      cumulative path length, as specified for intermediate vertices between
      surveyed spot levels.

    Phase 3 – TIN / nearest-terrain fallback (search_radius):
      Vertices not claimed by any terrain point in Phase 1 receive Z from the
      TIN surface (or nearest-point if outside the TIN hull).  This covers
      polyline endpoints and vertices in gaps where interpolation cannot reach
      (no known Z on one side).

    Phase 4 – Nearest-Z-in-polyline:
      Last resort for polylines entirely outside terrain coverage.
    """
    n = len(verts_2d)
    if n == 0:
        return []

    z_assigned = [None] * n

    # ── Phase 1: spot-owned greedy assignment ─────────────────────────────────
    # Each terrain point finds its nearest vertex; sorted so closest pairs are
    # processed first and each vertex is claimed at most once.
    candidates = []
    for tx, ty, tz in terrain_pts:
        best_i, best_d = None, search_radius
        for i, (vx, vy) in enumerate(verts_2d):
            d = math.sqrt((tx - vx) ** 2 + (ty - vy) ** 2)
            if d < best_d:
                best_d = d
                best_i = i
        if best_i is not None:
            candidates.append((best_d, tz, best_i))

    candidates.sort()
    for _d, tz, i in candidates:
        if z_assigned[i] is None:
            z_assigned[i] = tz

    pts = [(verts_2d[i][0], verts_2d[i][1], z_assigned[i]) for i in range(n)]

    # ── Phase 2: linear interpolation along polyline ──────────────────────────
    for i in range(n):
        if pts[i][2] is None:
            z = interpolate_z_along(pts, i)
            if z is not None:
                pts[i] = (pts[i][0], pts[i][1], z)

    # ── Phase 3: TIN / nearest-terrain fallback for unclaimed vertices ────────
    for i in range(n):
        if pts[i][2] is None:
            z = query_tin(pts[i][0], pts[i][1], tin, terrain_pts, search_radius)
            if z is not None:
                pts[i] = (pts[i][0], pts[i][1], z)

    # ── Phase 4: nearest-Z-in-polyline for full-gap polylines ────────────────
    for i in range(n):
        if pts[i][2] is None:
            best_z, best_d = None, 1e99
            for j, (ox, oy, oz) in enumerate(pts):
                if oz is not None and j != i:
                    d = math.sqrt((pts[i][0] - ox) ** 2 + (pts[i][1] - oy) ** 2)
                    if d < best_d:
                        best_d, best_z = d, oz
            if best_z is not None:
                pts[i] = (pts[i][0], pts[i][1], best_z)

    return [(x, y, z if z is not None else 0.0) for x, y, z in pts]


def elevate_line_geometry(segments, terrain_pts, tin=None):
    """
    Assign Z to every vertex in every segment using the spot-owned model.
    Returns list of ([(x,y,z), ...], layer_name).
    """
    result = []
    for verts_2d, layer in segments:
        verts_3d = assign_z_spot_owned(verts_2d, terrain_pts, tin=tin)
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
    polygons from PLOT OUTLINE INNER and H-EXTERNAL WALL layers.
    Transformations are applied manually (avoids the ezdxf negative-scale bug).

    Outer polygon strategy (in order of preference):
      1. H-EXTERNAL WALL LWPOLYLINE (≥3 pts) — use directly.
      2. H-EXTERNAL WALL LINE + H-EXTERNAL PARTY WALL 352 LINE/LWPOLYLINE segments
         — chain into a closed polygon (exact outer wall face for terrace blocks).
      3. Fall through to inner polygons.

    Inner polygon strategy (fallback for blocks with no outer wall data):
      1. H-PLOT OUTLINE INNER LWPOLYLINE (≥3 pts) — use directly.
      2. Convex hull of all H-PLOT OUTLINE INNER segment endpoints (MID blocks).

    Returns (inner_results, outer_results):
      inner_results — vertex lists from H-PLOT OUTLINE INNER
      outer_results — vertex lists from H-EXTERNAL WALL (already outer wall face)
    """
    inner_results = []
    outer_results = []

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

        closed_found = False   # True if any ≥3-pt INNER LWPOLYLINE was collected
        seg_pts = []           # endpoints from 2-pt INNER segments / LINEs

        # Line segments for chaining into outer polygon
        outer_line_segs = []   # ((x1,y1),(x2,y2)) from H-EXTERNAL WALL LINE
        party352_segs   = []   # ((x1,y1),(x2,y2)) from H-EXTERNAL PARTY WALL 352

        def world_pt(lx, ly):
            """Transform a block-local point to world coordinates."""
            s1 = _apply_insert_xform(lx, ly, tx_ins, ty_ins, sx_ins, sy_ins, rot_ins)
            px, py, psx, psy, prot = parent_xform
            return _apply_insert_xform(s1[0], s1[1], px, py, psx, psy, prot)

        for entity in block:
            try:
                lyr = entity.dxf.layer
            except Exception:
                lyr = ""

            is_inner  = _is_pad_source_layer(lyr)
            is_outer  = OUTER_WALL_LAYER.upper() in lyr.upper()
            is_party352 = ("EXTERNAL PARTY WALL 352" in lyr.upper())
            # H-EXTERNAL PARTY WALL (non-352) LINEs close the outline for MID inner blocks
            is_party  = ("EXTERNAL PARTY WALL" in lyr.upper()
                         and "352" not in lyr.upper())

            if entity.dxftype() == "LWPOLYLINE":
                raw = [(x, y) for x, y, *_ in entity.get_points()]
                step2 = [world_pt(bx, by) for bx, by in raw]

                if is_outer and len(step2) >= 3:
                    outer_results.append(step2)

                elif is_party352 and len(step2) >= 2:
                    # Party wall 352 as polyline: treat as segment sequence
                    for k in range(len(step2) - 1):
                        party352_segs.append((step2[k], step2[k + 1]))

                elif is_inner:
                    if len(step2) >= 3:
                        inner_results.append(step2)
                        closed_found = True
                    else:
                        seg_pts.extend(step2)

            elif entity.dxftype() == "LINE":
                s_w = world_pt(entity.dxf.start.x, entity.dxf.start.y)
                e_w = world_pt(entity.dxf.end.x,   entity.dxf.end.y)

                if is_outer:
                    outer_line_segs.append((s_w, e_w))
                elif is_party352:
                    party352_segs.append((s_w, e_w))
                elif is_inner or is_party:
                    seg_pts.extend([s_w, e_w])

            elif entity.dxftype() == "INSERT":
                recurse(entity, (tx_ins, ty_ins, sx_ins, sy_ins, rot_ins))

        # --- Try to chain H-EXTERNAL WALL LINE + H-EXTERNAL PARTY WALL 352 segments ---
        # This reconstructs the outer boundary for house blocks where H-EXTERNAL WALL
        # is expressed as LINE entities rather than a single closed LWPOLYLINE.
        if outer_line_segs and not any(
            OUTER_WALL_LAYER.upper() in lyr.upper()
            for ent in block
            for lyr in [ent.dxf.get("layer", "") if hasattr(ent, "dxf") else ""]
            if ent.dxftype() == "LWPOLYLINE" and len(list(ent.get_points())) >= 3
        ):
            all_outer_segs = outer_line_segs + party352_segs
            try:
                closed_outer = _chain_lines_to_outlines(all_outer_segs, snap_tol=0.05)
                for chain in closed_outer:
                    if len(chain) >= 3:
                        outer_results.append(chain)
            except Exception:
                pass

        # --- Inner polygon fallback: convex hull from segment endpoints (MID blocks) ---
        if not closed_found and len(seg_pts) >= 3:
            try:
                from shapely.geometry import MultiPoint
                hull = MultiPoint(seg_pts).convex_hull
                if hull.geom_type == "Polygon" and not hull.is_empty:
                    inner_results.append(list(hull.exterior.coords))
            except Exception:
                pass

    # The top-level INSERT has no parent transform (identity)
    recurse(insert_entity, (0.0, 0.0, 1.0, 1.0, 0.0))
    return inner_results, outer_results


def collect_building_outlines(msp, doc):
    """
    Collect building plot outlines from both PLOT OUTLINE INNER and H-EXTERNAL WALL.
    Transformations for nested blocks are applied manually to avoid the ezdxf
    negative-scale (mirrored INSERT) transformation bug.

    Returns (inner_outlines, outer_outlines):
      inner_outlines — (verts, layer) from H-PLOT OUTLINE INNER  (inner wall face)
      outer_outlines — (verts, layer) from H-EXTERNAL WALL       (outer wall face)
    """
    inner_outlines = []
    outer_outlines = []
    seen_inner = set()
    seen_outer = set()

    def _ckey(verts):
        cx = round(sum(v[0] for v in verts) / len(verts), 2)
        cy = round(sum(v[1] for v in verts) / len(verts), 2)
        return (cx, cy)

    def add_inner(verts, layer):
        if len(verts) < 3:
            return
        k = _ckey(verts)
        if k not in seen_inner:
            seen_inner.add(k)
            inner_outlines.append((verts, layer))

    def add_outer(verts, layer):
        if len(verts) < 3:
            return
        k = _ckey(verts)
        if k not in seen_outer:
            seen_outer.add(k)
            outer_outlines.append((verts, layer))

    # Direct LWPOLYLINE entities in modelspace
    for entity in msp:
        if entity.dxftype() != "LWPOLYLINE":
            continue
        lyr = entity.dxf.layer
        if _is_pad_source_layer(lyr):
            add_inner([(x, y) for x, y, *_ in entity.get_points()], lyr)
        elif OUTER_WALL_LAYER.upper() in lyr.upper():
            add_outer([(x, y) for x, y, *_ in entity.get_points()], lyr)

    # Geometry inside INSERT blocks
    for entity in msp:
        if entity.dxftype() != "INSERT":
            continue
        inner_verts_list, outer_verts_list = _block_outline_verts(doc, entity)
        for verts in inner_verts_list:
            add_inner(verts, BUILDING_OUTLINE_LAYERS[0])
        for verts in outer_verts_list:
            add_outer(verts, OUTER_WALL_LAYER)

    return inner_outlines, outer_outlines


def _to_shapely_polygon(verts):
    """Convert a vertex list to a Shapely Polygon. Returns None on failure."""
    try:
        poly = Polygon(verts)
        if not poly.is_valid:
            poly = poly.buffer(0)   # fix self-intersections
        return poly if not poly.is_empty else None
    except Exception:
        return None


def assign_ffl_and_merge(inner_outlines, ffl_annotations, outer_outlines=None):
    """
    Group plot outlines by their FFL value and union adjacent same-FFL plots.

    inner_outlines: list of (verts, layer) from H-PLOT OUTLINE INNER
    outer_outlines: list of (verts, layer) from H-EXTERNAL WALL (or None)
    ffl_annotations: list of ((x, y), ffl_value)

    Returns list of (shapely_polygon, ffl_value).

    Strategy:
      - Prefer H-EXTERNAL WALL (outer wall face) polygons where available.
        These need only a tiny merge buffer (5 mm) to ensure adjacent units
        union correctly at touching edges.
      - Fall back to H-PLOT OUTLINE INNER + WALL_BUFFER for units without
        H-EXTERNAL WALL data.
      - FFL annotation → polygon assignment and BFS propagation run on
        ALL polygons combined (inner + outer) to correctly identify groups.
      - Minimal simplification (1 mm) is applied to clean floating-point
        artifacts without moving vertices from their intended positions.
    """
    if outer_outlines is None:
        outer_outlines = []

    # Build shapely polygons for inner and outer
    inner_polys = []
    outer_polys = []

    for verts, _layer in inner_outlines:
        poly = _to_shapely_polygon(verts)
        if poly is not None and not poly.is_empty:
            inner_polys.append(poly)

    for verts, _layer in outer_outlines:
        poly = _to_shapely_polygon(verts)
        if poly is not None and not poly.is_empty:
            outer_polys.append(poly)

    # For each outer poly, find and remove the matching inner poly (the one
    # representing the same building unit in H-PLOT OUTLINE INNER).  This avoids
    # double-counting: the inner poly is ~WALL_BUFFER smaller than the outer, so
    # if they're very close (centroid distance < WALL_BUFFER×2) the outer takes
    # precedence and the inner is suppressed.
    superseded = set()
    for op in outer_polys:
        oc = op.centroid
        best_ii, best_d = None, WALL_BUFFER * 2
        for ii, ip in enumerate(inner_polys):
            if ii in superseded:
                continue
            d = oc.distance(ip.centroid)
            if d < best_d:
                best_d, best_ii = d, ii
        if best_ii is not None:
            superseded.add(best_ii)

    # Effective poly list: outer polys + non-superseded inner polys
    all_polys    = outer_polys + [p for i, p in enumerate(inner_polys) if i not in superseded]
    all_is_outer = [True] * len(outer_polys) + [False] * (len(all_polys) - len(outer_polys))

    if not all_polys:
        return []

    n = len(all_polys)
    ffl_assigned = [None] * n

    # ------------------------------------------------------------------
    # Pass 1 – assign the nearest polygon to each FFL annotation
    # ------------------------------------------------------------------
    FFL_PROXIMITY = 5.0   # metres — annotation may sit in road/garden between plots
    for (fx, fy), ffl_val in ffl_annotations:
        pt = Point(fx, fy)
        best_i, best_d = None, FFL_PROXIMITY
        for i, poly in enumerate(all_polys):
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
    # ------------------------------------------------------------------
    adj = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            if all_polys[i].distance(all_polys[j]) < ADJACENCY_TOL:
                adj[i].append(j)
                adj[j].append(i)

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
        cx, cy = all_polys[i].centroid.x, all_polys[i].centroid.y
        best_ffl, best_d = None, FFL_PROXIMITY
        for (fx, fy), ffl_val in ffl_annotations:
            d = dist_2d((cx, cy), (fx, fy))
            if d < best_d:
                best_d, best_ffl = d, ffl_val
        ffl_assigned[i] = best_ffl   # may remain None → dropped below

    # ------------------------------------------------------------------
    # Group and merge
    # ------------------------------------------------------------------
    # groups[ffl_val] = list of (poly, is_outer) pairs
    groups = defaultdict(list)
    for i, ffl_val in enumerate(ffl_assigned):
        if ffl_val is not None:
            groups[ffl_val].append((all_polys[i], all_is_outer[i]))

    result = []
    for ffl_val, group_items in groups.items():
        outer_in_group = [poly for poly, is_outer in group_items if is_outer]
        inner_in_group = [poly for poly, is_outer in group_items if not is_outer]

        expanded = []

        # Outer polys: exact H-EXTERNAL WALL face — use directly
        expanded.extend(outer_in_group)

        # Inner polys: expand to approximate outer wall face
        for poly in inner_in_group:
            expanded.append(poly.convex_hull.buffer(WALL_BUFFER))

        merged = unary_union(expanded)

        has_inner_only = bool(inner_in_group) and not outer_in_group
        if has_inner_only:
            # Light opening to remove thin sliver artifacts from buffer
            merged = merged.buffer(-0.01).buffer(0.01)

        parts = list(merged.geoms) if merged.geom_type == "MultiPolygon" else [merged]
        for part in parts:
            if not part.is_empty:
                if has_inner_only:
                    # Simplify buffer-generated polygon to match ref vertex count
                    final = part.simplify(SIMPLIFY_TOL, preserve_topology=False)
                    result.append((final if not final.is_empty else part, ffl_val))
                else:
                    # Group has outer polys — they're exact, no simplification.
                    # But we still need to clean up the mix of outer+inner if any
                    # inner polys are in this group too.
                    if inner_in_group:
                        final = part.simplify(SIMPLIFY_TOL, preserve_topology=False)
                        result.append((final if not final.is_empty else part, ffl_val))
                    else:
                        result.append((part, ffl_val))

    return result


def generate_pad_polylines(merged_pads, terrain_pts, tin=None):
    """
    For each (merged_polygon, ffl_value):
      - PLOTS FFL:            exterior inset by FFL_OFFSET, all Z = ffl_value
      - PLOTS EXTERNAL LEVEL: exterior coords with Z from TIN terrain surface

    Returns two lists: ffl_polylines, external_polylines.
    Each entry is a list of (x, y, z).
    """
    ffl_polys = []
    ext_polys  = []
    skipped    = 0

    for poly, ffl_val in merged_pads:
        # --- PLOTS EXTERNAL LEVEL: original exterior with TIN terrain Z ---
        ext_ring = list(poly.exterior.coords)
        ext_3d = assign_z_to_ring(
            [(x, y) for x, y in ext_ring],
            terrain_pts,
            TERRAIN_SEARCH_RAD,
            tin=tin
        )
        if len(ext_3d) >= 2:
            ext_polys.append(ext_3d)

        # --- PLOTS FFL: inward offset, constant Z = ffl_val ---
        # join_style=2 (mitre) keeps corners sharp so vertex positions match the
        # reference geometry (which uses a simple inset, not rounded buffering).
        try:
            inner = poly.buffer(-FFL_OFFSET, join_style=2)
            if inner.is_empty:
                inner = poly.buffer(-FFL_OFFSET)   # fall back to round if mitre fails
            if inner.is_empty:
                inner = poly
        except Exception:
            inner = poly

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

    # Build TIN once for use throughout the pipeline
    tin = build_tin(terrain_pts)
    print(f"  TIN: {'built from ' + str(len(terrain_pts)) + ' pts' if tin is not None else 'unavailable (scipy missing)'}")
    print()

    # ── Line geometry ─────────────────────────────────────────────────────────
    print("[4/8] Collecting drive / fence / road line geometry...")
    raw_segs = collect_line_geometry(msp)
    print(f"  {len(raw_segs)} raw segments\n")

    print("[5/8] Inserting spot vertices, chaining, elevating...")
    segs = insert_spot_vertices(raw_segs, terrain_pts, TERRAIN_SEARCH_RAD)
    segs = chain_segments(segs)
    print(f"  {len(segs)} chained polylines")
    line_polys_3d = elevate_line_geometry(segs, terrain_pts, tin=tin)
    print(f"  {len(line_polys_3d)} 3D_LINES polylines\n")

    # ── Building pads ─────────────────────────────────────────────────────────
    print("[6/8] Collecting building outlines...")
    inner_outlines, outer_outlines = collect_building_outlines(msp, doc)
    print(f"  {len(inner_outlines)} inner (H-PLOT OUTLINE INNER), {len(outer_outlines)} outer (H-EXTERNAL WALL)\n")

    print("[7/8] Merging adjacent same-FFL plots, generating pad polylines...")
    merged_pads = assign_ffl_and_merge(inner_outlines, ffl_data, outer_outlines=outer_outlines)
    total_outlines = len(inner_outlines) + len(outer_outlines)
    no_ffl = total_outlines - sum(
        1 for _, ffl_val in [(p, f) for p, f in merged_pads]
    )
    ffl_polys, ext_polys = generate_pad_polylines(merged_pads, terrain_pts, tin=tin)
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
