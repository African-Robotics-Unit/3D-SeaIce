# matlab — Reference MATLAB Pipeline

Reference implementation of the reconstruction pipeline, sharing the same `p3_config.json` config format as the Python pipeline. Useful as a reference or for steps that are easier to prototype interactively in MATLAB.

## Scripts (`scripts/`)

| Script | Description |
|--------|-------------|
| `pipeline.m` | Full end-to-end run |
| `preprocessing.m` | Cloud extraction and floor alignment |
| `coarseAlignment.m` | Manual/semi-automatic coarse scan registration |
| `Step1_pointcloudExtract.mlx` | Live script: extract from ROS2 bags |
| `Step3_filtering.mlx` | Live script: post-ICP filtering |
| `Step5_surfaceRoughness.mlx` | Live script: geometry analysis |

## Functions (`functions/`)

MATLAB equivalents of all Python tools: `alignFloorTform`, `runGliraICP`, `isolateSurface`, `interpolateCloud`, `geometryAnalysis`, and more.

## External (`external/GliraICP/`)

The original GliraICP library (Glira et al. 2015) that `python-3DRecon/pipeline/02_fine_alignment_glira.py` was ported from.
