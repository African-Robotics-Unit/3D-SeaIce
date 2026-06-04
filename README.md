# 3D Reconstruction of Pancake Sea Ice

**Agoritsa Spirakis** | MSc Eng Electrical Engineering, ARU/MARIS

Static 3D reconstruction of pancake sea ice from multi-viewpoint Livox LiDAR scans collected onboard a research vessel. The pipeline extracts raw sensor data from ROS bags, aligns scans using a global weighted ICP algorithm, and produces a 3D surface mesh with height statistics and roughness metrics.

---

## Repo Structure

```
3D-Sea-Ice/
├── python-3DRecon/       # Python pipeline (primary)
│   ├── config/           # p3_config.json
│   ├── pipeline/         # stage scripts 01-04
│   ├── tools/            # library modules
│   ├── utils/            # interactive utilities
│   └── output/           # generated files (gitignored)
├── matlab/               # MATLAB pipeline (reference)
│   ├── config/
│   ├── scripts/
│   ├── functions/
│   └── external/GliraICP/
└── legacy/               # original MATLAB scripts, pre-restructure
```

---

## Python Pipeline

The Python implementation is the primary active pipeline. The MATLAB version is a reference implementation sharing the same config format.

### Requirements

Python 3.9+

```bash
pip install open3d pyvista pyvistaqt rosbags numpy scipy pandas \
            trimesh pymeshfix shapely matplotlib
```

| Package | Role |
|---------|------|
| `open3d` | Point cloud I/O, voxel downsampling, Poisson reconstruction |
| `rosbags` | Read Livox LiDAR data from ROS1 `.bag` files |
| `pyvista` / `pyvistaqt` | 3D visualisation |
| `numpy` / `scipy` | Numerics, interpolation, statistics |
| `pandas` | Grid-based max-Z aggregation |
| `trimesh` / `pymeshfix` | Mesh repair and watertight closing |
| `shapely` | Convex hull masking |
| `matplotlib` | Height profile and statistics plots |

### Configuration

All parameters are in `python-3DRecon/config/p3_config.json`. Edit this before running:

```json
{
  "outputFolder": "/path/to/output/",
  "preprocessing": {
    "pathname": "/path/to/bag/files/",
    "scanOrder": "ABCDEFGH",
    "coarseALN": "/path/to/coarse_tforms.aln",
    "roi": { "x": ["-inf","inf"], "y": ["-inf","inf"], "z": ["-inf","inf"] }
  },
  "ICPOptions": {
    "UniformSamplingDistance": 0.2,
    "PlaneSearchRadius": 0.1,
    "MaxNoIt": 5
  },
  "intensityFilter": { "intensityMin": 0, "intensityMax": 200 },
  "geometry": { "downsampleGrid": 0.01, "interpolateGrid": 0.005 }
}
```

### Step 0 - Set your ROI

Before running the pipeline, define the cropping region using the interactive tool. It loads the first-frame test cloud and lets you adjust bounds with a live 3D preview.

```bash
python python-3DRecon/utils/adjustCrop.py
```

Enter bounds as `[[xmin,xmax],[ymin,ymax],[zmin,zmax]]` -- `inf` and `-inf` are supported. Accepted bounds are saved back to `p3_config.json` automatically.

### Running the Pipeline

Run all scripts from the **repo root** in order:

```bash
python python-3DRecon/pipeline/01_preprocessClouds.py

python python-3DRecon/pipeline/02_fine_alignment_glira.py \
    /path/to/BeforeICP/ \
    --config python-3DRecon/config/p3_config.json

python python-3DRecon/pipeline/03_postICP.py

python python-3DRecon/pipeline/04_geometry_analysis.py
```

### Pipeline Stages

| # | Script | Input | Output | Description |
|---|--------|-------|--------|-------------|
| 1 | `01_preprocessClouds.py` | `.bag` files | `BeforeICP/*.ply` | Extract LiDAR from ROS bags, crop ROI, floor-align with RANSAC, apply coarse alignment transforms |
| 2 | `02_fine_alignment_glira.py` | `BeforeICP/*.ply` | `AfterICP/ICPtforms.aln` | Global weighted point-to-plane ICP across all scan pairs simultaneously (Python port of GliraICP) |
| 3 | `03_postICP.py` | `AfterICP/*.ply` + `.aln` | `totalCake_with_planks.ply` | Apply ICP transforms, filter by intensity and overlap, merge into single cloud |
| 4 | `04_geometry_analysis.py` | `totalCake_with_planks.ply` | mesh + stats | Isolate top surface, interpolate Z-grid, compute height statistics, detect peaks, Poisson mesh |

**Note on coarse alignment:** step 1 requires a pre-computed coarse alignment `.aln` file (path set in config). This transform accounts for the rough relative pose between scan positions and is produced separately -- see the MATLAB `coarseAlignment.m` script or provide your own rigid transforms.

### Tools Reference

Modules in `python-3DRecon/tools/` can be imported directly in your own scripts:

```python
import tools.pcsurface as pcsurface
import tools.pcmesh as pcmesh

pc_top = pcsurface.isolate_surface(cloud)
mesh   = pcmesh.pc_to_surface_mesh(pc_top, depth=9)
```

| Module | Key functions |
|--------|---------------|
| `pcread.py` | `ros_get_raw`, `read_livox_pointcloud2`, `get_min_sz` |
| `pcio.py` | `read_aln`, `write_aln`, `read_clouds`, `save_clouds`, `read_cloud` |
| `pctransform.py` | `align_floor_tform`, `apply_rs_tforms`, `norms_analysis`, `reorder_point_cloud` |
| `pcprocess.py` | `crop_o3d_t`, `overlap_filter_optimal`, `filter_intensity_cloud`, `pccat` |
| `pcsurface.py` | `isolate_surface`, `interpolate_cloud`, `interest_peaks` |
| `pcmesh.py` | `pc_to_surface_mesh`, `fix_mesh`, `add_flat_bottom`, `add_bottom` |
| `pcdisplay.py` | `show_pointcloud_intensity`, `plotter_pcdisplay` |

---

## MATLAB Pipeline

Located in `matlab/`. Implements the same pipeline stages and uses the same `p3_config.json` config format. Useful as a reference or for steps that are easier to prototype interactively in MATLAB.

**Scripts** (`matlab/scripts/`):

| Script | Description |
|--------|-------------|
| `pipeline.m` | Full end-to-end run |
| `preprocessing.m` | Cloud extraction and floor alignment |
| `coarseAlignment.m` | Manual/semi-automatic coarse scan registration |
| `Step1_pointcloudExtract.mlx` | Live script: extract from ROS2 bags |
| `Step3_filtering.mlx` | Live script: post-ICP filtering |
| `Step5_surfaceRoughness.mlx` | Live script: geometry analysis |

**Functions** (`matlab/functions/`): MATLAB equivalents of all Python tools -- `alignFloorTform`, `runGliraICP`, `isolateSurface`, `interpolateCloud`, `geometryAnalysis`, and more.

**External** (`matlab/external/GliraICP/`): the original GliraICP library (Glira et al. 2015) that `02_fine_alignment_glira.py` was ported from.

---

## Legacy

`legacy/` contains the original MATLAB scripts predating the `matlab/` restructure (`3D Reconstruction`, `CoarseAlignment`, `DataAnalysis`, `FineAlignment`, `MeshGeneration`, `ExtractClouds`, `UserGuide`). Kept for reference only -- no active development.

---

## Reference

P. Glira, N. Pfeifer, C. Briese, C. Ressl (2015). *A Correspondence-Based Algorithm for Point Cloud Registration with High Accuracy and Speed*. ISPRS Annals of Photogrammetry, Remote Sensing and Spatial Information Sciences.
