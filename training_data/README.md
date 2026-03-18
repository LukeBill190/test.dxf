# Training Data

Each subfolder is one completed project. The folder name is arbitrary — use
something descriptive like the site name or job number.

```
training_data/
  project_001/
    input.dxf       ← source DXF (2D, same as you'd feed to the pipeline)
    output.dxf      ← finished 3D model DXF (engineer-verified output)
  project_002/
    input.dxf
    output.dxf
  ...
```

## Requirements

- `input.dxf`  — the raw source file containing terrain annotations on layers
  `LR SPOT LEVEL`, `LR DPC LEVEL`, `L018 HA_ANN_FEAT_TEXT`, `LR LLFA FFL`,
  and line geometry on `A-STEN-*` / `H-PATH` / `H-DRIVE` layers.
- `output.dxf` — the engineer-verified output containing 3D polylines on the
  `3D_LINES` layer (same format as produced by `pipeline.py`).

## Training

```bash
python ml_elevation.py train
```

This reads all project pairs, extracts features, trains a gradient-boosted
model, runs leave-one-project-out cross-validation, and saves the model to
`elevation_model.pkl`.

Once trained, `pipeline.py` automatically loads and uses the model instead of
the algorithmic greedy assignment.

## Adding more projects

Just add new `project_NNN/` folders and re-run `python ml_elevation.py train`.
The model improves with each additional project pair.

## Minimum recommended: 5 projects for basic coverage, 20+ for robust generalisation.
