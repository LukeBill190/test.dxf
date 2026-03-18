#!/usr/bin/env python3
"""
ml_elevation.py — Machine-learning Z-elevation predictor for 3D_LINES.

Replaces the algorithmic greedy Phase-1 spot-level assignment in pipeline.py
with a gradient-boosted tree model trained on engineer-verified input/output
DXF pairs.

Usage
-----
Train (run once, or again whenever you add more project pairs):
    python ml_elevation.py train [--data-dir training_data] [--model elevation_model.pkl]

Evaluate (leave-one-project-out cross-validation, printed automatically at end of train):
    python ml_elevation.py train --eval-only

The model is saved to elevation_model.pkl and loaded automatically by pipeline.py.

Feature design
--------------
For each vertex at (vx, vy) with a known layer type (fence/drive/road) the
feature vector encodes the nearest N annotations of every type:

    [layer_one_hot(3)] +
    [for each annotation type in (SPOT, DPC, L018, FFL):
        for k in 1..N_NEAR:
            dist_k, z_offset_k]           # distance + Z relative to site median

Using site-median-relative Z values (rather than absolute) lets the model
generalise across projects at different elevation datums.

The target is also Z relative to the site median, re-absolutised at inference.
"""

import argparse
import math
import os
import pickle
import re
import sys
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# Constants — must match pipeline.py
# ---------------------------------------------------------------------------
# Substring patterns for annotation layer detection (case-insensitive).
# Multiple aliases per type handle different surveying firm conventions, e.g.:
#   "LR SPOT LEVEL"         (standard)
#   "5_E-Spot Levels"       (alternate firm convention)
SPOT_LAYERS  = ["SPOT LEVEL"]               # matches "LR SPOT LEVEL", "5_E-Spot Levels", etc.
DPC_LAYERS   = ["DPC LEVEL", "DPC"]         # matches "LR DPC LEVEL"
L018_LAYERS  = ["L018 HA_ANN_FEAT_TEXT"]
FFL_LAYERS   = ["LLFA FFL", "FINISHED FLOOR", "FFL LEVEL"]  # matches "LR LLFA FFL", "5_E-Finished Floor Levels"
BLDG_LAYERS  = ["H-PLOT OUTLINE INNER", "H-EXTERNAL WALL", "HOUSE"]

DRIVE_KW = ["drive", "path"]
FENCE_KW = ["fence", "wall", "boundary"]
ROAD_KW  = ["road", "footpath", "kerb"]

OUTPUT_LAYER = "3D_LINES"

# Number of nearest annotations of each type to include in the feature vector.
N_NEAR  = 5
# Search radius for feature extraction (metres).
FEAT_RADIUS = 20.0
# Minimum number of directly-owned vertices required to use a training pair.
MIN_OWNED_VERTS = 30
# A vertex is "directly owned" (not interpolated) if its Z matches a terrain
# annotation within this distance and Z tolerance.
OWNED_XY_RADIUS  = 2.0   # metres
OWNED_Z_TOL      = 0.012  # 12 mm

MODEL_PATH_DEFAULT = Path(__file__).parent / "elevation_model.pkl"
DATA_DIR_DEFAULT   = Path(__file__).parent / "training_data"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _classify_layer(layer_name):
    ll = layer_name.lower()
    if any(kw in ll for kw in FENCE_KW): return "fence"
    if any(kw in ll for kw in DRIVE_KW): return "drive"
    if any(kw in ll for kw in ROAD_KW):  return "road"
    return None


def _suppress(layer_name):
    lu = layer_name.upper()
    BLDG_KW = ["PLOT", "BUILDING", "HOUSE", "GARAGE"]
    if any(kw in lu for kw in BLDG_KW): return True
    for pat in ["H-PLOT OUTLINE INNER", "H-EXTERNAL WALL", "EXTERNAL PARTY WALL"]:
        if pat.upper() in lu: return True
    return False


def _parse_z(text):
    nums = re.findall(r'[+-]?\d{2,3}\.\d{1,4}', str(text))
    return float(nums[0]) if nums else None


def collect_annotations(msp):
    """Return list of (x, y, z, ann_type) for all elevation text in modelspace."""
    annotations = []
    for ent in msp:
        if ent.dxftype() not in ("TEXT", "MTEXT"):
            continue
        try:
            xy = (ent.dxf.insert.x, ent.dxf.insert.y)
        except Exception:
            continue
        try:
            txt = ent.dxf.text if ent.dxftype() == "TEXT" else ent.text
        except Exception:
            continue
        z = _parse_z(txt)
        if z is None:
            continue
        layer = ent.dxf.layer
        if   any(s.upper() in layer.upper() for s in SPOT_LAYERS):  ann_type = "SPOT"
        elif any(s.upper() in layer.upper() for s in DPC_LAYERS):   ann_type = "DPC"
        elif any(s.upper() in layer.upper() for s in L018_LAYERS):  ann_type = "L018"
        elif any(s.upper() in layer.upper() for s in FFL_LAYERS):   ann_type = "FFL"
        else: continue
        annotations.append((xy[0], xy[1], z, ann_type))
    return annotations


def collect_output_verts(msp):
    """Return list of (x, y, z, layer_type) from 3D_LINES polylines."""
    verts = []
    for ent in msp:
        if ent.dxf.layer != OUTPUT_LAYER:
            continue
        if ent.dxftype() != "POLYLINE":
            continue
        layer_type = None  # 3D_LINES doesn't carry original layer; default drive
        for v in ent.vertices:
            loc = v.dxf.location
            verts.append((loc.x, loc.y, loc.z, layer_type))
    return verts


def collect_input_line_verts(msp):
    """Return list of (x, y, layer_type) from source drive/fence/road geometry."""
    verts = []
    for ent in msp:
        layer = ent.dxf.layer
        if _suppress(layer):
            continue
        lt = _classify_layer(layer)
        if lt is None:
            continue
        if ent.dxftype() == "LINE":
            s = ent.dxf.start; e = ent.dxf.end
            verts += [(s.x, s.y, lt), (e.x, e.y, lt)]
        elif ent.dxftype() == "LWPOLYLINE":
            for pt in ent.get_points():
                verts.append((pt[0], pt[1], lt))
        elif ent.dxftype() == "POLYLINE":
            for v in ent.vertices:
                try:
                    p = v.dxf.location
                except Exception:
                    try:
                        p = v.dxf.point
                    except Exception:
                        continue
                verts.append((p.x, p.y, lt))
        elif ent.dxftype() in ("ARC", "CIRCLE"):
            # Tessellate arc into ~16 points
            cx, cy = ent.dxf.center.x, ent.dxf.center.y
            r = ent.dxf.radius
            if ent.dxftype() == "ARC":
                a0, a1 = math.radians(ent.dxf.start_angle), math.radians(ent.dxf.end_angle)
                if a1 < a0: a1 += 2 * math.pi
                angles = np.linspace(a0, a1, max(4, int((a1 - a0) / 0.4)))
            else:
                angles = np.linspace(0, 2 * math.pi, 16)
            for a in angles:
                verts.append((cx + r * math.cos(a), cy + r * math.sin(a), lt))
    return verts


def nearest_k(vx, vy, ann_list, k, radius):
    """
    Return list of (dist, z) for the k nearest annotations within radius.
    Padded with (radius, nan) if fewer than k found.
    """
    dists = []
    for ax, ay, az, _ in ann_list:
        d = math.sqrt((ax - vx) ** 2 + (ay - vy) ** 2)
        if d < radius:
            dists.append((d, az))
    dists.sort()
    result = dists[:k]
    while len(result) < k:
        result.append((radius, float('nan')))
    return result


def nearest_building_dist(vx, vy, bldg_verts):
    """XY distance to nearest building outline vertex."""
    if not bldg_verts:
        return FEAT_RADIUS
    return min(math.sqrt((bx - vx) ** 2 + (by - vy) ** 2) for bx, by in bldg_verts)


def collect_building_verts(msp):
    bv = []
    for ent in msp:
        if not any(bl.upper() in ent.dxf.layer.upper() for bl in BLDG_LAYERS):
            continue
        if ent.dxftype() == "LWPOLYLINE":
            bv += [(pt[0], pt[1]) for pt in ent.get_points()]
        elif ent.dxftype() == "LINE":
            s = ent.dxf.start; e = ent.dxf.end
            bv += [(s.x, s.y), (e.x, e.y)]
    return bv


# ---------------------------------------------------------------------------
# Feature extraction
# ---------------------------------------------------------------------------

LAYER_ENC = {"fence": 0, "drive": 1, "road": 2, None: 1}
ANN_TYPES  = ["SPOT", "DPC", "L018", "FFL"]

def _feature_names():
    names = ["is_fence", "is_drive", "is_road"]
    for at in ANN_TYPES:
        for k in range(1, N_NEAR + 1):
            names += [f"{at}_dist_{k}", f"{at}_z_off_{k}"]
    names.append("bldg_dist")
    return names


FEATURE_NAMES = _feature_names()
N_FEATURES    = len(FEATURE_NAMES)


def extract_features(vx, vy, layer_type, ann_by_type, bldg_verts, site_median_z):
    """
    Build the feature vector for a single vertex.

    Parameters
    ----------
    vx, vy         : float — vertex XY coordinates
    layer_type     : str | None — "fence", "drive", "road", or None
    ann_by_type    : dict[str, list[(x,y,z,type)]] — annotations split by type
    bldg_verts     : list[(x,y)] — building outline vertices
    site_median_z  : float — median Z of all terrain annotations for this project
    """
    enc = LAYER_ENC.get(layer_type, 1)
    feat = [
        1.0 if enc == 0 else 0.0,   # is_fence
        1.0 if enc == 1 else 0.0,   # is_drive
        1.0 if enc == 2 else 0.0,   # is_road
    ]
    for at in ANN_TYPES:
        pairs = nearest_k(vx, vy, ann_by_type.get(at, []), N_NEAR, FEAT_RADIUS)
        for dist, z in pairs:
            z_off = (z - site_median_z) if not math.isnan(z) else 0.0
            feat.append(dist)
            feat.append(z_off)
    feat.append(nearest_building_dist(vx, vy, bldg_verts))
    return feat


# ---------------------------------------------------------------------------
# Data extraction from one DXF pair
# ---------------------------------------------------------------------------

def extract_pair(input_dxf_path, output_dxf_path, project_name="?"):
    """
    Extract (features, targets) from one input/output DXF pair.

    Returns
    -------
    features : np.ndarray  shape (N, N_FEATURES)
    targets  : np.ndarray  shape (N,)   — site-median-relative Z
    n_owned  : int         — number of directly-owned (non-interpolated) vertices used
    site_med : float       — site median Z (needed for de-normalisation at inference)
    """
    import ezdxf
    try:
        in_doc  = ezdxf.readfile(input_dxf_path)
        out_doc = ezdxf.readfile(output_dxf_path)
    except Exception as e:
        print(f"  [skip] {project_name}: could not read DXF — {e}")
        return None

    in_msp  = in_doc.modelspace()
    out_msp = out_doc.modelspace()

    # --- Collect terrain annotations from input ---
    all_ann = collect_annotations(in_msp)
    if not all_ann:
        print(f"  [skip] {project_name}: no terrain annotations found in input")
        return None

    ann_by_type = {"SPOT": [], "DPC": [], "L018": [], "FFL": []}
    for item in all_ann:
        ann_by_type[item[3]].append(item)

    site_z_vals  = [a[2] for a in all_ann]
    site_median  = float(np.median(site_z_vals))

    # --- Collect building verts from input ---
    bldg_verts = collect_building_verts(in_msp)

    # --- Collect source line vertices to infer layer type at each output vertex ---
    src_line_verts = collect_input_line_verts(in_msp)  # (x, y, layer_type)

    def infer_layer_type(vx, vy, radius=1.5):
        """Find the closest source line vertex and return its layer type."""
        best_d, best_lt = radius, None
        for sx, sy, lt in src_line_verts:
            d = math.sqrt((sx - vx) ** 2 + (sy - vy) ** 2)
            if d < best_d:
                best_d, best_lt = d, lt
        return best_lt

    # --- Collect output 3D_LINES vertices ---
    out_verts = collect_output_verts(out_msp)
    if not out_verts:
        print(f"  [skip] {project_name}: no 3D_LINES vertices in output")
        return None

    # --- Filter to directly-owned vertices only ---
    # A vertex is "directly owned" if it's close to at least one terrain
    # annotation with a very similar Z value.  Interpolated vertices (between
    # two owned vertices) are excluded because training on them would teach
    # the model to reproduce linear interpolation rather than assignment logic.
    non_ffl_ann = [a for a in all_ann if a[3] != "FFL"]

    def is_owned(vx, vy, vz):
        for ax, ay, az, _ in non_ffl_ann:
            if math.sqrt((ax - vx) ** 2 + (ay - vy) ** 2) <= OWNED_XY_RADIUS:
                if abs(az - vz) <= OWNED_Z_TOL:
                    return True
        return False

    features, targets = [], []
    n_owned = 0
    for vx, vy, vz, _ in out_verts:
        if not is_owned(vx, vy, vz):
            continue
        lt = infer_layer_type(vx, vy)
        feat = extract_features(vx, vy, lt, ann_by_type, bldg_verts, site_median)
        features.append(feat)
        targets.append(vz - site_median)
        n_owned += 1

    if n_owned < MIN_OWNED_VERTS:
        print(f"  [skip] {project_name}: only {n_owned} owned vertices (min {MIN_OWNED_VERTS})")
        return None

    print(f"  {project_name}: {n_owned} owned vertices extracted")
    return (np.array(features, dtype=np.float32),
            np.array(targets,  dtype=np.float32),
            n_owned,
            site_median)


# ---------------------------------------------------------------------------
# Training
# ---------------------------------------------------------------------------

def train(data_dir=DATA_DIR_DEFAULT, model_path=MODEL_PATH_DEFAULT, eval_only=False):
    from sklearn.ensemble import GradientBoostingRegressor
    from sklearn.metrics import mean_absolute_error

    data_dir = Path(data_dir)
    pairs = sorted([
        d for d in data_dir.iterdir()
        if d.is_dir()
        and (d / "input.dxf").exists()
        and (d / "output.dxf").exists()
    ])

    if not pairs:
        print(f"No project pairs found in {data_dir}.")
        print("Add subfolders each containing input.dxf and output.dxf, then re-run.")
        return

    print(f"Found {len(pairs)} project pair(s) in {data_dir}\n")

    project_data = []
    for proj_dir in pairs:
        result = extract_pair(
            proj_dir / "input.dxf",
            proj_dir / "output.dxf",
            project_name=proj_dir.name,
        )
        if result is not None:
            feats, tgts, n, med = result
            project_data.append({"name": proj_dir.name, "X": feats, "y": tgts,
                                  "n": n, "median": med})

    if not project_data:
        print("\nNo valid project pairs after filtering. Cannot train.")
        return

    total_verts = sum(p["n"] for p in project_data)
    print(f"\nTotal training vertices: {total_verts} across {len(project_data)} project(s)")

    # --- Leave-one-project-out cross-validation ---
    if len(project_data) > 1:
        print("\nLeave-one-project-out cross-validation:")
        cv_maes = []
        for i, held_out in enumerate(project_data):
            train_X = np.vstack([p["X"] for j, p in enumerate(project_data) if j != i])
            train_y = np.concatenate([p["y"] for j, p in enumerate(project_data) if j != i])
            val_X, val_y = held_out["X"], held_out["y"]

            model_cv = GradientBoostingRegressor(
                n_estimators=400, max_depth=5, learning_rate=0.05,
                min_samples_leaf=5, subsample=0.8, random_state=42
            )
            model_cv.fit(train_X, train_y)
            pred_y = model_cv.predict(val_X)
            mae_mm = mean_absolute_error(val_y, pred_y) * 1000
            cv_maes.append(mae_mm)
            print(f"  {held_out['name']:<30} MAE = {mae_mm:6.1f} mm")

        print(f"\n  Mean CV MAE: {np.mean(cv_maes):.1f} mm  |  "
              f"Median: {np.median(cv_maes):.1f} mm  |  "
              f"Max: {np.max(cv_maes):.1f} mm")
    else:
        print("\n(Only 1 project — skipping cross-validation. Add more pairs for reliable evaluation.)")

    if eval_only:
        return

    # --- Train final model on all data ---
    print("\nTraining final model on all data...")
    all_X = np.vstack([p["X"] for p in project_data])
    all_y = np.concatenate([p["y"] for p in project_data])

    model = GradientBoostingRegressor(
        n_estimators=400, max_depth=5, learning_rate=0.05,
        min_samples_leaf=5, subsample=0.8, random_state=42
    )
    model.fit(all_X, all_y)

    # Train-set MAE (optimistic — use CV MAE for honest estimate)
    train_mae = mean_absolute_error(all_y, model.predict(all_X)) * 1000
    print(f"  Train MAE: {train_mae:.1f} mm  (use CV MAE above for honest estimate)")

    # Feature importances
    importances = sorted(zip(model.feature_importances_, FEATURE_NAMES), reverse=True)
    print("\nTop 10 feature importances:")
    for imp, name in importances[:10]:
        print(f"  {name:<25} {imp:.4f}")

    # Save model + metadata
    payload = {
        "model":         model,
        "feature_names": FEATURE_NAMES,
        "n_projects":    len(project_data),
        "total_verts":   total_verts,
        "project_medians": {p["name"]: p["median"] for p in project_data},
    }
    with open(model_path, "wb") as f:
        pickle.dump(payload, f)
    print(f"\nModel saved to {model_path}")
    return model_path


# ---------------------------------------------------------------------------
# Inference helpers (called from pipeline.py)
# ---------------------------------------------------------------------------

_cached_model = None

def load_model(model_path=MODEL_PATH_DEFAULT):
    """Load and cache the trained model. Returns None if not found."""
    global _cached_model
    if _cached_model is not None:
        return _cached_model
    model_path = Path(model_path)
    if not model_path.exists():
        return None
    with open(model_path, "rb") as f:
        payload = pickle.load(f)
    _cached_model = payload
    return payload


def predict_z_batch(verts_2d, layer_type, ann_by_type, bldg_verts, site_median_z,
                    model_payload):
    """
    Predict Z for every vertex in verts_2d using the trained model.

    Parameters
    ----------
    verts_2d      : list of (x, y)
    layer_type    : str | None
    ann_by_type   : dict — annotations split by type (from pipeline)
    bldg_verts    : list of (x, y)
    site_median_z : float — median Z of all terrain annotations for this project
    model_payload : dict  — loaded via load_model()

    Returns
    -------
    list of float — predicted Z for each vertex
    """
    model = model_payload["model"]
    X = np.array([
        extract_features(vx, vy, layer_type, ann_by_type, bldg_verts, site_median_z)
        for vx, vy in verts_2d
    ], dtype=np.float32)
    z_offsets = model.predict(X)
    return [site_median_z + float(z) for z in z_offsets]


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Train elevation prediction model")
    parser.add_argument("command", choices=["train"], help="Command to run")
    parser.add_argument("--data-dir",  default=str(DATA_DIR_DEFAULT),
                        help="Directory containing project pair subfolders")
    parser.add_argument("--model",     default=str(MODEL_PATH_DEFAULT),
                        help="Output path for trained model (.pkl)")
    parser.add_argument("--eval-only", action="store_true",
                        help="Run cross-validation only, do not save model")
    args = parser.parse_args()

    if args.command == "train":
        train(data_dir=args.data_dir, model_path=args.model, eval_only=args.eval_only)


if __name__ == "__main__":
    main()
