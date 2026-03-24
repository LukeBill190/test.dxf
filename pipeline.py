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
# Additional outer-wall layer substrings for projects that use Revit/Xref-style naming
# (case-insensitive substring match, same as OUTER_WALL_LAYER).
EXTRA_OUTER_WALL_LAYERS = ["RfPl-Wall_Outline"]
BUILDING_KEYWORDS       = ["plot", "building", "house", "garage"]
BUILDING_SUPPRESS_LAYERS = ["EXTERNAL PARTY WALL"]  # exclude from 3D_LINES but not pads

# Layer keyword classification for 3D_LINES geometry selection
DRIVE_KEYWORDS  = ["drive", "path"]
FENCE_KEYWORDS  = ["fence", "wall", "boundary"]
ROAD_KEYWORDS   = ["road", "footpath", "kerb"]
POND_KEYWORDS   = ["pond", "water", "suds", "attenuation", "earthworks"]

# Elevation text source layers — substring patterns (case-insensitive).
# Multiple aliases per type handle different surveying firm conventions, e.g.:
#   "LR SPOT LEVEL" / "5_E-Spot Levels"    — both contain "SPOT LEVEL"
#   "LR LLFA FFL"                           — contains "LLFA FFL"
#   "5_E-Finished Floor Levels"             — contains "FINISHED FLOOR"
#   "REFA-EXT.W-Prop-Levels"               — contains "PROP-LEVELS"
#   "REFA-EXT.W-Exist-Levels"              — contains "EXIST-LEVELS"
#   "_REFA_ FFLs"                           — contains "FFL"
SPOT_LEVEL_LAYERS = ["SPOT LEVEL", "L018 HA_ANN_FEAT_TEXT", "PROP-LEVELS", "EXIST-LEVELS", "EXT LEVEL", "EXTERNAL LEVEL", "PV LEVEL"]
FFL_LAYERS        = ["LLFA FFL", "FINISHED FLOOR", "FFL"]
DPC_LAYERS        = ["DPC LEVEL"]

# Layers to EXCLUDE from the spot-only terrain point set used for 3D_LINES elevation.
# L018 HA_ANN_FEAT_TEXT contains road chainage survey levels (road surface, spaced
# every ~2m along the road centreline).  These are correct for road polylines but
# must not be applied to adjacent fence/boundary lines whose Z should match the
# DPC / ground level, not the lower road surface.
SPOT_EXCLUDE_3DLINES = ["L018 HA_ANN_FEAT_TEXT"]

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
TERRAIN_SEARCH_RAD       = 0.5   # 500 mm: max radius for spot-level Z lookup
FENCE_TERRAIN_SEARCH_RAD = 0.5   # 500 mm: same for fence/boundary lines

# Building pad merge parameters
WALL_BUFFER     = 0.205  # 205 mm: wall thickness buffer; expands inner→outer boundary
SIMPLIFY_TOL    = 0.005  # 5 mm: polygon simplification after merge (preserve original vertices)

# Arc tessellation: max chord length (metres)
ARC_CHORD_MAX = 0.1

# Maximum vertex spacing for 3D_LINES output (metres).
# Segments longer than this are subdivided so the output vertex density
# matches the reference model (which has vertices ~ every 1 m along polylines).
MAX_VERTEX_SPACING = 1.0

# Geometric building outline detection parameters.
# Used when keyword-based layer matching finds no outlines (e.g. projects where
# house types are on firm-specific or house-type-named layers).
GEOM_DETECT_FFL_RADIUS = 25.0   # metres: search radius around each FFL annotation
GEOM_DETECT_MIN_AREA   = 15.0   # m²: smallest plausible house footprint
GEOM_DETECT_MAX_AREA   = 800.0  # m²: largest single-building footprint

# Layer substrings that identify annotation / non-geometry layers — these are
# excluded from the geometric candidate scan so text and hatch entities don't
# pollute the closed-polygon list.
GEOM_EXCLUDE_LAYERS = [
    "SPOT LEVEL", "PROP-LEVELS", "EXIST-LEVELS", "EXT LEVEL", "EXTERNAL LEVEL",
    "PV LEVEL", "DPC LEVEL", "DPC", "L018", "LLFA FFL", "FINISHED FLOOR",
    "FFL LEVEL", "HATCH", "DEFPOINTS", "ANNO", "DIMENSION", "CHAINAGE",
]

# ML elevation model (loaded at runtime if elevation_model.pkl exists)
try:
    import ml_elevation as _ml_elevation
    _HAS_ML = True
except ImportError:
    _HAS_ML = False


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


def _is_outer_wall_layer(layer_name):
    """True if this layer contains outer wall face geometry (H-EXTERNAL WALL or Revit-style equivalents)."""
    lu = layer_name.upper()
    if OUTER_WALL_LAYER.upper() in lu:
        return True
    return any(pat.upper() in lu for pat in EXTRA_OUTER_WALL_LAYERS)


def _classify_layer_ml(layer_name):
    """Return 'fence', 'drive', 'road', 'pond', or None for ML feature extraction."""
    ll = layer_name.lower()
    if any(kw in ll for kw in FENCE_KEYWORDS): return "fence"
    if any(kw in ll for kw in DRIVE_KEYWORDS): return "drive"
    if any(kw in ll for kw in ROAD_KEYWORDS):  return "road"
    if any(kw in ll for kw in POND_KEYWORDS):  return "pond"
    return None


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
    """Classify a layer as drive / fence / road / pond, or return None."""
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
    for kw in POND_KEYWORDS:
        if kw.upper() in lu:
            return "pond"
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
    if entity.dxftype() == "MTEXT":
        return entity.plain_text()
    if entity.dxftype() in ("TEXT", "ATTRIB"):
        return entity.dxf.text
    return None


def get_text_insertion(entity):
    if entity.dxftype() in ("TEXT", "MTEXT", "ATTRIB"):
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
    Returns list of ((x, y), elevation, layer, is_ffl, is_dpc).
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
                results.append((insert_pt, elev, entity.dxf.layer, is_ffl, is_dpc))

    for entity in msp:
        if entity.dxftype() in ("TEXT", "MTEXT"):
            process_text(entity)
        elif entity.dxftype() == "INSERT":
            for sub in iter_virtual_deep(entity):
                if sub.dxftype() in ("TEXT", "MTEXT"):
                    process_text(sub)
            # ATTRIBs are not yielded by virtual_entities(); they belong to
            # the INSERT itself and store their position already in WCS.
            try:
                for att in entity.attribs:
                    process_text(att)
            except Exception:
                pass

    return results


def build_terrain_points(elevation_data):
    """
    Convert elevation data into a flat list of (x, y, z) terrain points.
    FFL annotations are included (they are valid ground elevations too).
    """
    return [(xy[0], xy[1], elev) for xy, elev, _layer, _is_ffl, _is_dpc in elevation_data]


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

def insert_spot_vertices(segments, terrain_pts, search_radius=TERRAIN_SEARCH_RAD):
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

        # Find nearest edge within search_radius
        best_dist = search_radius
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


def subdivide_segments(segments, max_spacing=MAX_VERTEX_SPACING):
    """
    Insert intermediate 2-D vertices along each polyline so that no consecutive
    pair of vertices is further apart than max_spacing metres.  This matches the
    dense vertex distribution in the reference model (~ every 1 m) and ensures
    that the Z-interpolation phase produces vertices at positions close to those
    in the reference.
    """
    result = []
    for verts, layer in segments:
        new_verts = [verts[0]]
        for i in range(len(verts) - 1):
            ax, ay = verts[i]
            bx, by = verts[i + 1]
            seg_len = math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)
            if seg_len > max_spacing:
                n_div = math.ceil(seg_len / max_spacing)
                for k in range(1, n_div):
                    t = k / n_div
                    new_verts.append((ax + t * (bx - ax), ay + t * (by - ay)))
            new_verts.append((bx, by))
        result.append((new_verts, layer))
    return result


def subdivide_segments_3d(polys_3d, max_spacing=MAX_VERTEX_SPACING):
    """
    Subdivide already-elevated 3D polylines by inserting intermediate vertices
    every max_spacing metres.  Z is linearly interpolated along each 3D segment,
    preserving the gradient established by the elevation phase.

    This is called AFTER elevation so that the sparse original vertices (one per
    spot level) get their Z assigned before subdivision; dense intermediate
    vertices then inherit Z by linear interpolation rather than competing with the
    original nodes for terrain points.
    """
    result = []
    for verts, layer in polys_3d:
        new_verts = [verts[0]]
        for i in range(len(verts) - 1):
            ax, ay, az = verts[i]
            bx, by, bz = verts[i + 1]
            seg_len = math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)
            if seg_len > max_spacing:
                n_div = math.ceil(seg_len / max_spacing)
                for k in range(1, n_div):
                    t = k / n_div
                    new_verts.append((ax + t * (bx - ax),
                                      ay + t * (by - ay),
                                      az + t * (bz - az)))
            new_verts.append((bx, by, bz))
        result.append((new_verts, layer))
    return result


def assign_z_spot_owned(verts_2d, terrain_pts, search_radius=TERRAIN_SEARCH_RAD,
                        tin=None, shared_assignment=False, z_precomputed=None):
    """
    Elevation model for open polylines (drives, fences, roads).

    Phase 1 – Spot-owned assignment:
      Three modes:

      z_precomputed is not None:
        Use pre-computed Z values (e.g. from ML model) as Phase 1 seeds.
        Only vertices with a non-None value in z_precomputed are assigned;
        the rest fall through to Phase 2/3/4 as normal.

      shared_assignment=False  (default, used for drives/roads):
        Terrain-centric greedy — each terrain point claims the single nearest
        vertex within search_radius.  Closest pairs are processed first and
        each vertex is claimed at most once.  Preserves gradient information
        because a single spot level can't be used by multiple adjacent vertices.

      shared_assignment=True  (used for fence/boundary lines):
        Vertex-centric — each vertex independently looks up its nearest terrain
        point within search_radius and takes its Z (shared use allowed).  This
        prevents a terrain point being "stolen" by a slightly-closer vertex on
        an adjacent polyline, which would leave a fence vertex to fall through
        to the TIN and pick up a wrong elevation.

    Phase 2 – Linear interpolation along polyline:
      Vertices between two owned vertices receive Z by linear interpolation of
      cumulative path length.

    Phase 3 – TIN / nearest-terrain fallback (search_radius):
      Vertices not claimed in Phase 1 receive Z from the TIN surface (or
      nearest-point if outside the TIN hull).

    Phase 4 – Nearest-Z-in-polyline:
      Last resort for polylines entirely outside terrain coverage.
    """
    n = len(verts_2d)
    if n == 0:
        return []

    z_assigned = [None] * n

    # ── Phase 1: spot-owned assignment ────────────────────────────────────────
    if z_precomputed is not None:
        # Use pre-computed Z (ML predictions) for vertices that have one.
        for i, z in enumerate(z_precomputed):
            if z is not None:
                z_assigned[i] = z
    elif shared_assignment:
        # Vertex-centric: each vertex independently finds its nearest terrain pt.
        for i, (vx, vy) in enumerate(verts_2d):
            best_d, best_z = search_radius, None
            for tx, ty, tz in terrain_pts:
                d = math.sqrt((tx - vx) ** 2 + (ty - vy) ** 2)
                if d < best_d:
                    best_d, best_z = d, tz
            if best_z is not None:
                z_assigned[i] = best_z
    else:
        # Terrain-centric greedy: each terrain pt claims its nearest unowned vertex.
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


def elevate_line_geometry(segments, terrain_pts, tin=None,
                          search_radius=TERRAIN_SEARCH_RAD,
                          shared_assignment=False,
                          ml_payload=None, ann_by_type=None,
                          bldg_verts_ml=None, site_median_z=None,
                          rwall_segs_ml=None):
    """
    Assign Z to every vertex in every segment.

    When ml_payload is provided (loaded via ml_elevation.load_model()), the ML
    model replaces Phase-1 (greedy spot-owned assignment) for vertices that are
    within search_radius of at least one terrain annotation.  Vertices further
    away (dense intermediate subdivision vertices) still go through Phases 2–4
    (linear interpolation, TIN, nearest-Z), exactly as in the algorithmic path.
    This hybrid ensures the ML improves Z accuracy at terrain-annotated vertices
    while linear interpolation fills in the gaps correctly.

    Returns list of ([(x,y,z), ...], layer_name).
    """
    use_ml = (ml_payload is not None and ann_by_type is not None and _HAS_ML)
    result = []
    for verts_2d, layer in segments:
        if use_ml:
            try:
                layer_type = _classify_layer_ml(layer)
                # Identify vertices within terrain search radius
                all_terrain = [pt for pts in ann_by_type.values() for pt in pts]
                near_mask = [False] * len(verts_2d)
                for i, (vx, vy) in enumerate(verts_2d):
                    for tx, ty, tz, *_ in all_terrain:
                        if math.sqrt((tx - vx) ** 2 + (ty - vy) ** 2) <= search_radius:
                            near_mask[i] = True
                            break
                # ML predictions for all vertices (cheap batch call)
                z_preds = _ml_elevation.predict_z_batch(
                    verts_2d, layer_type, ann_by_type,
                    bldg_verts_ml or [], site_median_z or 0.0, ml_payload,
                    rwall_segs=rwall_segs_ml or [],
                )
                # Phase 1 seeds: only use ML Z for vertices near terrain
                z_precomputed = [z_preds[i] if near_mask[i] else None
                                 for i in range(len(verts_2d))]
                # Phase 2/3/4 handled inside assign_z_spot_owned
                verts_3d = assign_z_spot_owned(verts_2d, terrain_pts, tin=tin,
                                               search_radius=search_radius,
                                               shared_assignment=shared_assignment,
                                               z_precomputed=z_precomputed)
            except Exception as e:
                print(f"  [ML fallback → algorithmic] {layer}: {e}")
                verts_3d = assign_z_spot_owned(verts_2d, terrain_pts, tin=tin,
                                               search_radius=search_radius,
                                               shared_assignment=shared_assignment)
        else:
            verts_3d = assign_z_spot_owned(verts_2d, terrain_pts, tin=tin,
                                           search_radius=search_radius,
                                           shared_assignment=shared_assignment)
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

        # Collect LWPOLYLINEs separately by type for deduplication within this scope.
        scope_outer_wall_lwpolys = []  # from H-EXTERNAL WALL layer
        scope_party352_lwpolys   = []  # from H-EXTERNAL PARTY WALL 352 layer

        # Tracks whether THIS block's LWPOLY entities (not nested INSERT recursion)
        # contributed outer polys.  Used to decide whether LINE chaining is needed.
        outer_added_by_lwpoly = False

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
            is_outer  = _is_outer_wall_layer(lyr)
            is_party352 = ("EXTERNAL PARTY WALL 352" in lyr.upper())
            # H-EXTERNAL PARTY WALL (non-352) LINEs close the outline for MID inner blocks
            is_party  = ("EXTERNAL PARTY WALL" in lyr.upper()
                         and "352" not in lyr.upper())

            if entity.dxftype() == "LWPOLYLINE":
                raw = [(x, y) for x, y, *_ in entity.get_points()]
                step2 = [world_pt(bx, by) for bx, by in raw]

                if is_outer and len(step2) >= 3:
                    scope_outer_wall_lwpolys.append(step2)

                elif is_party352 and len(step2) >= 3:
                    # Collected separately; priority decision made after entity loop.
                    scope_party352_lwpolys.append(step2)
                elif is_party352 and len(step2) == 2:
                    party352_segs.append((step2[0], step2[1]))

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

        # Add all H-EXTERNAL WALL LWPOLYLINEs unconditionally.
        for poly in scope_outer_wall_lwpolys:
            outer_results.append(poly)
            outer_added_by_lwpoly = True

        # Add party352 LWPOLYLINEs, deduplicating nearly-identical ones within this scope.
        deduped_p352 = []
        for verts in scope_party352_lwpolys:
            try:
                p_new = Polygon(verts)
                if not p_new.is_valid:
                    p_new = p_new.buffer(0)
                is_dup = False
                for p_kept in deduped_p352:
                    inter = p_new.intersection(p_kept).area
                    union = p_new.union(p_kept).area
                    if union > 0 and inter / union > 0.95:
                        is_dup = True
                        break
                if not is_dup:
                    deduped_p352.append(p_new)
                    outer_results.append(verts)
                    outer_added_by_lwpoly = True
            except Exception:
                outer_results.append(verts)
                outer_added_by_lwpoly = True

        # --- Try to chain H-EXTERNAL WALL LINE + H-EXTERNAL PARTY WALL 352 segments ---
        # This reconstructs the outer boundary for house blocks where H-EXTERNAL WALL
        # is expressed as LINE entities rather than a single closed LWPOLYLINE.
        if outer_line_segs and not outer_added_by_lwpoly:
            all_outer_segs = outer_line_segs + party352_segs
            try:
                closed_outer = _chain_lines_to_outlines(all_outer_segs, snap_tol=0.05)
                for chain in closed_outer:
                    if len(chain) >= 3:
                        # Reject degenerate chains with near-zero area
                        # (e.g. two collinear segments sharing endpoints chain into a line)
                        try:
                            _cp = Polygon(chain)
                            if _cp.area < 0.5:  # < 0.5 m² is degenerate for building pads
                                continue
                        except Exception:
                            pass
                        outer_results.append(chain)
                        outer_added_by_lwpoly = True
            except Exception:
                pass

        # --- Convex hull of party352 LINE endpoints as outer polygon fallback ---
        # Works for KIRK (3-sided incomplete LINE chain) and ALDRIDGE-MID (top+bottom LINEs).
        # The 4 corner points of the outer wall rectangle are all present as LINE endpoints.
        if not outer_added_by_lwpoly and party352_segs:
            party_pts = set()
            for (ax, ay), (bx, by) in party352_segs:
                party_pts.add((round(ax, 3), round(ay, 3)))
                party_pts.add((round(bx, 3), round(by, 3)))
            if len(party_pts) >= 4:
                try:
                    from shapely.geometry import MultiPoint
                    hull = MultiPoint(list(party_pts)).convex_hull
                    if hull.geom_type == "Polygon" and not hull.is_empty:
                        outer_results.append(list(hull.exterior.coords))
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


def _geom_exclude(layer_name):
    """True if this layer should be skipped during geometric candidate collection."""
    lu = layer_name.upper()
    return any(kw.upper() in lu for kw in GEOM_EXCLUDE_LAYERS)


def collect_closed_poly_candidates(msp, doc):
    """
    Collect ALL closed / near-closed polygons from modelspace and INSERT blocks,
    from any layer, as candidates for building outline detection.

    Excludes annotation layers (GEOM_EXCLUDE_LAYERS) and polygons whose area
    falls outside the plausible house footprint range
    (GEOM_DETECT_MIN_AREA … GEOM_DETECT_MAX_AREA).

    Returns list of (verts_2d, layer_name, area_m2).
    """
    candidates = []
    seen = set()

    def _ckey(verts):
        cx = round(sum(v[0] for v in verts) / len(verts), 1)
        cy = round(sum(v[1] for v in verts) / len(verts), 1)
        return (cx, cy)

    def _try_add(verts, layer):
        if len(verts) < 4:
            return
        if dist_2d(verts[0], verts[-1]) > CHAIN_SNAP_TOL * 2:
            return
        try:
            poly = Polygon(verts)
            if not poly.is_valid:
                poly = poly.buffer(0)
            area = poly.area
        except Exception:
            return
        if area < GEOM_DETECT_MIN_AREA or area > GEOM_DETECT_MAX_AREA:
            return
        k = _ckey(verts)
        if k not in seen:
            seen.add(k)
            candidates.append((verts, layer, area))

    # ----------------------------------------------------------------
    # Block geometry cache — process each unique block definition ONCE,
    # then apply the affine transform per INSERT.  This avoids calling
    # virtual_entities() thousands of times for drawings with many
    # repeated house-type blocks (project_006: 2229 inserts, 191 FFL).
    # ----------------------------------------------------------------
    _block_poly_cache = {}   # block_name → list of (verts_local, layer)

    def _block_closed_polys(block_name, depth=0):
        """
        Return list of (verts_in_block_coords, layer) closed polygons
        for the named block definition.  Results are cached.
        """
        if block_name in _block_poly_cache:
            return _block_poly_cache[block_name]
        if depth > 4:
            _block_poly_cache[block_name] = []
            return []
        try:
            blk = doc.blocks.get(block_name)
        except Exception:
            blk = None
        if blk is None:
            _block_poly_cache[block_name] = []
            return []

        polys = []
        local_segs = defaultdict(list)

        for sub in blk:
            try:
                layer = sub.dxf.layer
            except Exception:
                continue
            if _geom_exclude(layer):
                continue

            if sub.dxftype() == "LWPOLYLINE":
                try:
                    pts = [(x, y) for x, y, *_ in sub.get_points()]
                except Exception:
                    continue
                if len(pts) < 3:
                    continue
                is_closed = sub.closed or dist_2d(pts[0], pts[-1]) < CHAIN_SNAP_TOL * 2
                if is_closed:
                    ring = pts if dist_2d(pts[0], pts[-1]) < 1e-6 else pts + [pts[0]]
                    polys.append((ring, layer))

            elif sub.dxftype() == "LINE":
                try:
                    s, e = sub.dxf.start, sub.dxf.end
                    local_segs[layer].append(((s.x, s.y), (e.x, e.y)))
                except Exception:
                    continue

            elif sub.dxftype() == "INSERT":
                # Nested INSERT: transform child geometry into this block's space
                try:
                    csx = sub.dxf.get("xscale", 1.0)
                    csy = sub.dxf.get("yscale", 1.0)
                    crot = sub.dxf.get("rotation", 0.0)
                    ctx, cty = sub.dxf.insert.x, sub.dxf.insert.y
                except Exception:
                    continue
                for child_verts, child_layer in _block_closed_polys(sub.dxf.name, depth + 1):
                    xf = [_apply_insert_xform(x, y, ctx, cty, csx, csy, crot)
                          for x, y in child_verts]
                    polys.append((xf, child_layer))

        for layer, segs in local_segs.items():
            for chain in _chain_lines_to_outlines(segs, snap_tol=CHAIN_SNAP_TOL):
                polys.append((chain, layer))

        _block_poly_cache[block_name] = polys
        return polys

    # ----------------------------------------------------------------
    # Pass 1: direct modelspace entities
    # ----------------------------------------------------------------
    direct_line_segs = defaultdict(list)

    for entity in msp:
        if entity.dxftype() == "LWPOLYLINE":
            if _geom_exclude(entity.dxf.layer):
                continue
            pts = [(x, y) for x, y, *_ in entity.get_points()]
            if len(pts) < 3:
                continue
            is_closed = entity.closed or dist_2d(pts[0], pts[-1]) < CHAIN_SNAP_TOL * 2
            if is_closed:
                ring = pts if dist_2d(pts[0], pts[-1]) < 1e-6 else pts + [pts[0]]
                _try_add(ring, entity.dxf.layer)

        elif entity.dxftype() == "LINE":
            if not _geom_exclude(entity.dxf.layer):
                s, e = entity.dxf.start, entity.dxf.end
                direct_line_segs[entity.dxf.layer].append(((s.x, s.y), (e.x, e.y)))

        elif entity.dxftype() == "INSERT":
            # Apply INSERT's world transform to each cached local polygon
            try:
                sx = entity.dxf.get("xscale", 1.0)
                sy = entity.dxf.get("yscale", 1.0)
                rot = entity.dxf.get("rotation", 0.0)
                tx, ty = entity.dxf.insert.x, entity.dxf.insert.y
            except Exception:
                continue
            for local_verts, layer in _block_closed_polys(entity.dxf.name):
                world_verts = [_apply_insert_xform(x, y, tx, ty, sx, sy, rot)
                               for x, y in local_verts]
                _try_add(world_verts, layer)

    # Chain direct LINE segments (layer-by-layer) into closed polygons
    for layer, segs in direct_line_segs.items():
        for chain in _chain_lines_to_outlines(segs, snap_tol=CHAIN_SNAP_TOL):
            _try_add(chain, layer)

    return candidates


def select_building_outlines_geometric(ffl_annotations, candidates):
    """
    For each FFL annotation find the single best candidate closed polygon and
    return it as a building outline.

    Scoring strategy — for each FFL point we want the most specific (smallest)
    closed polygon that either *contains* the FFL or is very close to it:

        score = (GEOM_DETECT_MAX_AREA − area) − dist_to_ffl × 10

    A polygon that contains the FFL gets dist = 0, so it is strongly preferred
    over larger surrounding polygons (e.g. plot boundaries).  Among containing
    polygons the smallest area wins, which avoids picking a large site/plot
    boundary polygon over the actual house footprint.

    Duplicate polygons (same building detected by multiple nearby FFL
    annotations) are suppressed by centroid proximity (< 2 m).

    Returns list of (verts, layer_name) suitable for appending to outer_outlines.
    """
    results = []
    used_centroids = set()

    # Pre-compute Shapely polygons once so the inner FFL loop only calls
    # .contains() / .distance() rather than reconstructing polygons each time.
    prebuilt = []
    for verts, layer, area in candidates:
        try:
            poly = Polygon(verts)
            if not poly.is_valid:
                poly = poly.buffer(0)
            prebuilt.append((poly, verts, layer, area))
        except Exception:
            continue

    for (fx, fy), _ffl_val in ffl_annotations:
        ffl_pt = Point(fx, fy)

        best_score  = None
        best_verts  = None
        best_layer  = None

        for poly, verts, layer, area in prebuilt:
            try:
                if poly.contains(ffl_pt):
                    dist = 0.0
                else:
                    dist = poly.distance(ffl_pt)
                    if dist > GEOM_DETECT_FFL_RADIUS:
                        continue

                # Prefer smallest polygon closest to FFL
                score = (GEOM_DETECT_MAX_AREA - area) - dist * 10.0

                if best_score is None or score > best_score:
                    best_score = score
                    best_verts = verts
                    best_layer = layer
            except Exception:
                continue

        if best_verts is None:
            continue

        # Deduplicate: skip if a polygon with the same centroid was already added
        cx = round(sum(v[0] for v in best_verts) / len(best_verts), 1)
        cy = round(sum(v[1] for v in best_verts) / len(best_verts), 1)
        if (cx, cy) not in used_centroids:
            used_centroids.add((cx, cy))
            results.append((best_verts, best_layer))

    return results


def collect_morphological_building_outlines(msp, ffl_annotations):
    """
    Morphological building outline detection — used when no pre-formed closed
    polygons are found near FFL annotations.

    For each FFL annotation the algorithm:
      1. Collects all nearby LINE segments on non-annotation layers
         (within MORPH_SEARCH_RADIUS metres).
      2. Dilates each segment by MORPH_WALL_HALF (half a typical wall thickness)
         and unions the results into a single 'building mass' blob.
      3. Extracts the individual connected polygon(s) and selects the one that
         most closely matches the FFL point (smallest qualifying polygon that
         contains or is nearest to the FFL).

    This naturally finds the external footprint of houses where all internal
    geometry (rooms, windows, walls) shares one layer — the outer boundary of
    the unioned blob is the external wall face.

    Returns list of (verts, layer_name).
    """
    MORPH_SEARCH_RADIUS = 15.0   # metres — line collection radius per FFL
    MORPH_WALL_HALF     = 0.35   # metres — dilation half-thickness

    results = []
    used_centroids = set()

    # Pre-collect ALL non-annotation LINE segments from modelspace (with XY only)
    all_segs = []   # (layer, x1, y1, x2, y2)
    for entity in msp:
        if entity.dxftype() != "LINE":
            continue
        if _geom_exclude(entity.dxf.layer):
            continue
        s, e = entity.dxf.start, entity.dxf.end
        all_segs.append((entity.dxf.layer, s.x, s.y, e.x, e.y))

    if not all_segs:
        return results

    # Vectorised Voronoi assignment — compute all (seg_midpoint → FFL) distances
    # in one numpy matrix operation so we avoid an O(n_segs × n_ffls) Python loop.
    import numpy as _np
    mid_arr = _np.array([((x1 + x2) / 2, (y1 + y2) / 2)
                         for _, x1, y1, x2, y2 in all_segs], dtype=_np.float64)
    ffl_arr = _np.array([list(xy) for xy, _ in ffl_annotations], dtype=_np.float64)

    # dist_mat[i, j] = distance from segment-i midpoint to FFL-j
    diff = mid_arr[:, None, :] - ffl_arr[None, :, :]          # (n_segs, n_ffls, 2)
    dist_mat = _np.sqrt((diff ** 2).sum(axis=2))               # (n_segs, n_ffls)

    # For each segment, the index of the nearest FFL
    nearest_ffl_idx = dist_mat.argmin(axis=1)                  # (n_segs,)

    for fi, ((fx, fy), _ffl_val) in enumerate(ffl_annotations):
        # Voronoi: segments whose nearest FFL is THIS one, within MORPH_SEARCH_RADIUS
        mask = (nearest_ffl_idx == fi) & (dist_mat[:, fi] < MORPH_SEARCH_RADIUS)
        nearby = [
            (all_segs[si][1], all_segs[si][2], all_segs[si][3], all_segs[si][4])
            for si in _np.where(mask)[0]
        ]

        if len(nearby) < 8:
            continue

        try:
            from shapely.geometry import LineString as _LS
            dilated = [
                _LS([(x1, y1), (x2, y2)]).buffer(MORPH_WALL_HALF, cap_style=2, join_style=2)
                for x1, y1, x2, y2 in nearby
            ]
            blob = unary_union(dilated)
            # Slight erosion to separate touching blobs and clean artefacts
            blob = blob.buffer(-MORPH_WALL_HALF * 0.3)

            if blob.is_empty:
                continue

            geoms = list(blob.geoms) if blob.geom_type == "MultiPolygon" else [blob]
            ffl_pt = Point(fx, fy)

            best_poly = None
            best_score = None
            for geom in geoms:
                if geom.geom_type != "Polygon":
                    continue
                area = geom.area
                if area < GEOM_DETECT_MIN_AREA or area > GEOM_DETECT_MAX_AREA:
                    continue
                if geom.contains(ffl_pt):
                    dist = 0.0
                else:
                    dist = geom.distance(ffl_pt)
                    if dist > GEOM_DETECT_FFL_RADIUS:
                        continue
                score = (GEOM_DETECT_MAX_AREA - area) - dist * 10.0
                if best_score is None or score > best_score:
                    best_score = score
                    best_poly = geom

            if best_poly is None:
                continue

            verts = list(best_poly.exterior.coords)
            cx = round(sum(v[0] for v in verts) / len(verts), 1)
            cy = round(sum(v[1] for v in verts) / len(verts), 1)
            if (cx, cy) not in used_centroids:
                used_centroids.add((cx, cy))
                results.append((verts, "GEOM_MORPH"))

        except Exception:
            continue

    return results


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
        elif _is_outer_wall_layer(lyr):
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

    # Position-grouped LINE segment collection.
    # Some projects (e.g. Revit Xref exports) split a building outline across
    # two co-located INSERT blocks (e.g. "AS-Sold" + "Optional" variants at the
    # same XY position).  Neither block's LINE segments form a closed ring on
    # their own, but combining all segments from inserts at the same position
    # does.  We collect outer-wall LINE segments (any _is_outer_wall_layer),
    # group them by INSERT position (rounded to nearest metre), chain each
    # group, and add any resulting closed outlines.
    #
    # Block-local LINE segments are cached so repeated instances of the same
    # block definition are only scanned once (important for projects with
    # thousands of identical inserts, e.g. project_006 with ~2229 inserts).
    _blk_line_cache = {}   # block_name → list of (bx1,by1, bx2,by2) local coords

    def _outer_lines_for_block(blk_name):
        """Return local-coord LINE segments on EXTRA_OUTER_WALL_LAYERS only.

        H-EXTERNAL WALL is already handled per-INSERT by _block_outline_verts;
        we only need the extra layers here (e.g. RfPl-Wall_Outline) so that
        projects using H-EXTERNAL WALL don't incur unnecessary work.
        """
        if blk_name in _blk_line_cache:
            return _blk_line_cache[blk_name]
        blk = doc.blocks.get(blk_name)
        segs = []
        if blk is not None:
            for sub in blk:
                try:
                    lyr = sub.dxf.layer
                except Exception:
                    continue
                if sub.dxftype() != "LINE":
                    continue
                lu = lyr.upper()
                if not any(pat.upper() in lu for pat in EXTRA_OUTER_WALL_LAYERS):
                    continue
                try:
                    s, e = sub.dxf.start, sub.dxf.end
                    segs.append((s.x, s.y, e.x, e.y))
                except Exception:
                    continue
        _blk_line_cache[blk_name] = segs
        return segs

    pos_segs = defaultdict(list)
    for entity in msp:
        if entity.dxftype() != "INSERT":
            continue
        local_lines = _outer_lines_for_block(entity.dxf.name)
        if not local_lines:
            continue
        try:
            sx  = entity.dxf.get("xscale", 1.0)
            sy  = entity.dxf.get("yscale", 1.0)
            rot = entity.dxf.get("rotation", 0.0)
            tx, ty = entity.dxf.insert.x, entity.dxf.insert.y
        except Exception:
            continue
        pos_key = (round(tx, 0), round(ty, 0))
        for bx1, by1, bx2, by2 in local_lines:
            sw = _apply_insert_xform(bx1, by1, tx, ty, sx, sy, rot)
            ew = _apply_insert_xform(bx2, by2, tx, ty, sx, sy, rot)
            pos_segs[pos_key].append((sw, ew))

    for pos_key, segs in pos_segs.items():
        for chain in _chain_lines_to_outlines(segs, snap_tol=0.1):
            if len(chain) >= 3:
                add_outer(chain, OUTER_WALL_LAYER)

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
    # Group and merge — split each FFL group into connected components
    # ------------------------------------------------------------------
    # Two polygons share the same FFL but may be physically separate buildings
    # on opposite sides of the site. Only merge truly adjacent polys (connected
    # in the adjacency graph). This prevents non-adjacent same-FFL polys from
    # being incorrectly unioned, which would create extra vertices and shift corners.

    # groups[ffl_val] = list of poly indices
    groups = defaultdict(list)
    for i, ffl_val in enumerate(ffl_assigned):
        if ffl_val is not None:
            groups[ffl_val].append(i)

    result = []
    for ffl_val, group_indices in groups.items():
        # Find connected components within this FFL group using adjacency graph
        idx_set = set(group_indices)
        visited = set()
        components = []
        for start in group_indices:
            if start in visited:
                continue
            # BFS within this FFL group only
            comp = []
            q = deque([start])
            visited.add(start)
            while q:
                node = q.popleft()
                comp.append(node)
                for nb in adj[node]:
                    if nb in idx_set and nb not in visited:
                        visited.add(nb)
                        q.append(nb)
            components.append(comp)

        # Process each connected component independently
        for comp_indices in components:
            outer_in_group = [all_polys[i] for i in comp_indices if all_is_outer[i]]
            inner_in_group = [all_polys[i] for i in comp_indices if not all_is_outer[i]]

            expanded = []
            # Outer polys: exact H-EXTERNAL WALL face — use directly
            expanded.extend(outer_in_group)
            # Inner polys: expand to approximate outer wall face
            for poly in inner_in_group:
                expanded.append(poly.convex_hull.buffer(WALL_BUFFER))

            # --- All groups with at least one outer poly ---
            # Use snap to align adjacent outer polys to a shared vertex grid (1mm),
            # then union directly WITHOUT simplification.
            # Snapping closes float-precision gaps so adjacent units merge correctly.
            # No simplification: the union naturally keeps one shared-edge endpoint
            # per junction, and the comparison only checks that reference vertices
            # appear in the output (extra vertices never cause failures).
            if outer_in_group:
                from shapely import snap as _snap
                polys = list(outer_in_group)
                # Pairwise snap adjacent outer polys to align their shared edges
                for _i in range(len(polys)):
                    for _j in range(len(polys)):
                        if _i != _j and polys[_i].distance(polys[_j]) < 0.01:
                            polys[_i] = _snap(polys[_i], polys[_j], 0.001)
                merged = unary_union(polys)
                parts = list(merged.geoms) if merged.geom_type == "MultiPolygon" else [merged]
                for part in parts:
                    if not part.is_empty:
                        result.append((part, ffl_val))
                continue

            # Inner-only group: expand inner polys to approximate outer wall face
            # buffer(+d) + unary_union + buffer(-d) closes float gaps in terrace rows
            GAP_CLOSE = 0.0001  # 0.1 mm (actual float gaps are ≤ 0.03 mm)
            expanded_padded = [poly.buffer(GAP_CLOSE, join_style=2) for poly in expanded]
            merged = unary_union(expanded_padded).buffer(-GAP_CLOSE, join_style=2)
            # Light opening to remove thin sliver artifacts from buffer
            merged = merged.buffer(-0.01).buffer(0.01)

            parts = list(merged.geoms) if merged.geom_type == "MultiPolygon" else [merged]
            for part in parts:
                if not part.is_empty:
                    # Simplify buffer-generated polygon to match ref vertex count
                    final = part.simplify(SIMPLIFY_TOL, preserve_topology=False)
                    result.append((final if not final.is_empty else part, ffl_val))

    return result


def _dpc_z_for_vertex(vx, vy, dpc_pts, ffl_val, search_radius=12.0):
    """
    Find the nearest DPC annotation within search_radius of (vx, vy) that is
    at or below ffl_val.  Returns DPC Z value, or ffl_val if none found.
    """
    best_d, best_z = search_radius, None
    for dx, dy, dz in dpc_pts:
        if dz > ffl_val + 0.001:   # DPC cannot exceed FFL
            continue
        d = math.sqrt((vx - dx) ** 2 + (vy - dy) ** 2)
        if d < best_d:
            best_d, best_z = d, dz
    return best_z if best_z is not None else ffl_val


def generate_pad_polylines(merged_pads, terrain_pts, tin=None, dpc_pts=None):
    """
    For each (merged_polygon, ffl_value):
      - PLOTS FFL:            exterior inset by FFL_OFFSET, all Z = ffl_value
      - PLOTS EXTERNAL LEVEL: exterior coords with Z from nearest DPC annotation
                              (fallback: ffl_val if no DPC found nearby)

    Returns two lists: ffl_polylines, external_polylines.
    Each entry is a list of (x, y, z).
    """
    if dpc_pts is None:
        dpc_pts = []
    ffl_polys = []
    ext_polys  = []
    skipped    = 0

    # --- Pre-assign each DPC annotation to its nearest polygon (owned DPC) ---
    # This prevents annotations from neighbouring buildings being used for
    # the wrong polygon when looking up per-vertex Z values.
    from shapely.geometry import Point as _ShapelyPoint
    owned_dpc = [[] for _ in merged_pads]   # list of (dx, dy, dz) per polygon
    poly_centroids = [(poly.centroid.x, poly.centroid.y) for poly, _ in merged_pads]

    for dx, dy, dz in dpc_pts:
        best_ci, best_cd = None, float("inf")
        for ci, (cx, cy) in enumerate(poly_centroids):
            d = math.sqrt((dx - cx) ** 2 + (dy - cy) ** 2)
            if d < best_cd:
                best_cd, best_ci = d, ci
        if best_ci is not None:
            owned_dpc[best_ci].append((dx, dy, dz))

    for pi, (poly, ffl_val) in enumerate(merged_pads):
        # Use only DPC annotations owned by this polygon; fall back to full list
        # only if no owned annotations were found (edge-case: isolated building
        # with no nearby DPC annotation).
        poly_dpc = owned_dpc[pi] if owned_dpc[pi] else dpc_pts

        # --- PLOTS EXTERNAL LEVEL: exterior with per-vertex DPC annotation Z ---
        ext_ring = list(poly.exterior.coords)
        ext_3d = [
            (x, y, _dpc_z_for_vertex(x, y, poly_dpc, ffl_val))
            for x, y in ext_ring
        ]
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
    ffl_data  = [(xy, elev) for xy, elev, _l, is_ffl, _is_dpc in elevation_data if is_ffl]
    spot_data = [(xy, elev) for xy, elev, _l, is_ffl, _is_dpc in elevation_data if not is_ffl]
    dpc_pts   = [(xy[0], xy[1], elev) for xy, elev, _l, _is_ffl, is_dpc in elevation_data if is_dpc]
    print(f"  {len(ffl_data)} FFL annotations, {len(spot_data)} spot levels, {len(dpc_pts)} DPC levels")
    terrain_pts = build_terrain_points(elevation_data)
    if terrain_pts:
        zs = [z for _, _, z in terrain_pts]
        print(f"  Elevation range: {min(zs):.3f} – {max(zs):.3f} m")

    # Unified terrain point pool for 3D_LINES elevation.
    # Excludes FFL only — FFL marks the finished floor level (150-300 mm above
    # external ground) and would push drive/fence Z values too high.
    # DPC and L018 road-chainage levels are both included so that:
    #   • DPC-adjacent fence/boundary lines get the correct near-ground Z.
    #   • Road/footpath lines get the correct road-surface Z from L018.
    # The greedy Phase-1 assignment naturally resolves competition: the closest
    # terrain point wins per vertex, so DPC wins for fence vertices and L018 wins
    # for road vertices (they are typically the geometrically nearest sources).
    spot_terrain_pts = [(xy[0], xy[1], elev)
                        for xy, elev, layer, is_ffl, is_dpc in elevation_data
                        if not is_ffl]

    # Build TIN once for use throughout the pipeline (all terrain points including
    # FFL and DPC; used only as a fallback when no nearby spot level is found).
    tin = build_tin(terrain_pts)
    print(f"  TIN: {'built from ' + str(len(terrain_pts)) + ' pts' if tin is not None else 'unavailable (scipy missing)'}")
    print(f"  Terrain pts for 3D_LINES: {len(spot_terrain_pts)}")

    # Try to load the trained ML elevation model.
    _ml_payload = None
    _ann_by_type = None
    _bldg_verts_ml = None
    _rwall_segs_ml = None
    _site_median_z = None
    if _HAS_ML:
        _ml_payload = _ml_elevation.load_model()
        if _ml_payload:
            # Build annotation dict for ML feature extraction (same format as ml_elevation).
            # Use the same substring patterns as ml_elevation so all surveying firms match.
            _spot_kw = [s.upper() for s in _ml_elevation.SPOT_LAYERS]
            _dpc_kw  = [s.upper() for s in _ml_elevation.DPC_LAYERS]
            _l018_kw = [s.upper() for s in _ml_elevation.L018_LAYERS]
            _ffl_kw  = [s.upper() for s in _ml_elevation.FFL_LAYERS]
            _ann_by_type = {"SPOT": [], "DPC": [], "L018": [], "FFL": []}
            for xy, elev, layer, is_ffl, is_dpc in elevation_data:
                lu = layer.upper()
                if any(kw in lu for kw in _spot_kw):
                    _ann_by_type["SPOT"].append((xy[0], xy[1], elev, "SPOT"))
                elif any(kw in lu for kw in _dpc_kw):
                    _ann_by_type["DPC"].append((xy[0], xy[1], elev, "DPC"))
                elif any(kw in lu for kw in _l018_kw):
                    _ann_by_type["L018"].append((xy[0], xy[1], elev, "L018"))
                elif any(kw in lu for kw in _ffl_kw):
                    _ann_by_type["FFL"].append((xy[0], xy[1], elev, "FFL"))
            _bldg_verts_ml = _ml_elevation.collect_building_verts(msp)
            _rwall_segs_ml = _ml_elevation.collect_rwall_segments(msp)
            all_z = [elev for _, elev, _, _, _ in elevation_data]
            if all_z:
                sorted_z = sorted(all_z)
                n = len(sorted_z)
                _site_median_z = (sorted_z[n // 2] if n % 2
                                  else (sorted_z[n // 2 - 1] + sorted_z[n // 2]) / 2.0)
            print(f"  ML model loaded ({_ml_payload['n_projects']} project(s), "
                  f"{_ml_payload['total_verts']} training vertices) — using ML elevation")
        else:
            print("  No ML model found — using algorithmic elevation assignment")
    print()

    # ── Line geometry ─────────────────────────────────────────────────────────
    print("[4/8] Collecting drive / fence / road line geometry...")
    raw_segs = collect_line_geometry(msp)
    print(f"  {len(raw_segs)} raw segments\n")

    print("[5/8] Inserting spot vertices, chaining, subdividing, elevating...")
    segs = insert_spot_vertices(raw_segs, spot_terrain_pts, TERRAIN_SEARCH_RAD)
    segs = chain_segments(segs)
    segs = subdivide_segments(segs, MAX_VERTEX_SPACING)
    line_polys_3d = elevate_line_geometry(
        segs, spot_terrain_pts, tin=tin,
        ml_payload=_ml_payload, ann_by_type=_ann_by_type,
        bldg_verts_ml=_bldg_verts_ml, site_median_z=_site_median_z,
        rwall_segs_ml=_rwall_segs_ml,
    )
    print(f"  {len(line_polys_3d)} 3D_LINES polylines\n")

    # ── Building pads ─────────────────────────────────────────────────────────
    print("[6/8] Collecting building outlines...")
    inner_outlines, outer_outlines = collect_building_outlines(msp, doc)
    print(f"  {len(inner_outlines)} inner (H-PLOT OUTLINE INNER), {len(outer_outlines)} outer (H-EXTERNAL WALL)")

    # Geometric fallback — scan ALL closed polygons near FFL annotations.
    # Runs on every project; results are only used for FFL annotations not already
    # covered by the keyword-based detection above (dedup by centroid proximity).
    geo_candidates = collect_closed_poly_candidates(msp, doc)
    geo_outlines   = select_building_outlines_geometric(ffl_data, geo_candidates)
    existing_verts = [(v, l) for v, l in inner_outlines + outer_outlines]
    n_geo_added    = 0
    for verts, layer in geo_outlines:
        cx = sum(v[0] for v in verts) / len(verts)
        cy = sum(v[1] for v in verts) / len(verts)
        already_covered = any(
            math.sqrt((cx - sum(v[0] for v in kv) / len(kv)) ** 2 +
                      (cy - sum(v[1] for v in kv) / len(kv)) ** 2) < 2.0
            for kv, _ in existing_verts
        )
        if not already_covered:
            outer_outlines.append((verts, layer))
            existing_verts.append((verts, layer))
            n_geo_added += 1
    if n_geo_added:
        print(f"  +{n_geo_added} geometrically-detected outline(s) (layer-name-agnostic fallback)")

    # Morphological fallback — used when the closed-polygon scan above still
    # leaves FFL annotations uncovered (e.g. where all house geometry shares one
    # layer and the external outline is not a single pre-formed closed polygon).
    # We dilate+union nearby LINE segments to reconstruct the building mass.
    uncovered_ffls = []
    for (fx, fy), ffl_val in ffl_data:
        ffl_pt = Point(fx, fy)
        covered = any(
            Polygon(v).contains(ffl_pt) or Polygon(v).distance(ffl_pt) < 3.0
            for v, _ in existing_verts
            if len(v) >= 3
        )
        if not covered:
            uncovered_ffls.append(((fx, fy), ffl_val))

    if uncovered_ffls:
        morph_outlines = collect_morphological_building_outlines(msp, uncovered_ffls)
        n_morph = 0
        for verts, layer in morph_outlines:
            cx = sum(v[0] for v in verts) / len(verts)
            cy = sum(v[1] for v in verts) / len(verts)
            dup = any(
                math.sqrt((cx - sum(v[0] for v in kv) / len(kv)) ** 2 +
                          (cy - sum(v[1] for v in kv) / len(kv)) ** 2) < 2.0
                for kv, _ in existing_verts
            )
            if not dup:
                outer_outlines.append((verts, layer))
                existing_verts.append((verts, layer))
                n_morph += 1
        if n_morph:
            print(f"  +{n_morph} morphologically-detected outline(s) (buffer+union fallback)")
    print()

    print("[7/8] Merging adjacent same-FFL plots, generating pad polylines...")
    merged_pads = assign_ffl_and_merge(inner_outlines, ffl_data, outer_outlines=outer_outlines)
    total_outlines = len(inner_outlines) + len(outer_outlines)
    no_ffl = total_outlines - sum(
        1 for _, ffl_val in [(p, f) for p, f in merged_pads]
    )
    ffl_polys, ext_polys = generate_pad_polylines(merged_pads, terrain_pts, tin=tin, dpc_pts=dpc_pts)
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
