# 3D Sea-Ice Point Cloud Reconstruction Pipeline

## Overview

This pipeline processes raw Livox LiDAR scans (from ROS `.bag` files) through floor alignment,
multi-scan ICP registration, intensity filtering, surface extraction, and geometric analysis
to produce a 3D model of a sea-ice sample. It is implemented in Python using Open3D, PyVista,
rosbags, trimesh, and scipy.

---

## Setup

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

All parameters are in `config/p3_config.json`. Edit this before running:

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

---

## Running the Pipeline

### Step 0 - Set your ROI

Before running the pipeline, define the cropping region using the interactive tool. It loads the first-frame test cloud and lets you adjust bounds with a live 3D preview.

```bash
python utils/adjustCrop.py
```

Enter bounds as `[[xmin,xmax],[ymin,ymax],[zmin,zmax]]` -- `inf` and `-inf` are supported. Accepted bounds are saved back to `p3_config.json` automatically.

### Steps 1-4

Run all scripts from the **repo root** in order:

```bash
python python-3DRecon/pipeline/01_preprocessClouds.py

python python-3DRecon/pipeline/02_fine_alignment_glira.py \
    /path/to/BeforeICP/ \
    --config python-3DRecon/config/p3_config.json

python python-3DRecon/pipeline/03_postICP.py

python python-3DRecon/pipeline/04_geometry_analysis.py
```

| # | Script | Input | Output | Description |
|---|--------|-------|--------|-------------|
| 1 | `01_preprocessClouds.py` | `.bag` files | `BeforeICP/*.ply` | Extract LiDAR from ROS bags, crop ROI, floor-align with RANSAC, apply coarse alignment transforms |
| 2 | `02_fine_alignment_glira.py` | `BeforeICP/*.ply` | `AfterICP/ICPtforms.aln` | Global weighted point-to-plane ICP across all scan pairs simultaneously (Python port of GliraICP) |
| 3 | `03_postICP.py` | `AfterICP/*.ply` + `.aln` | `totalCake_with_planks.ply` | Apply ICP transforms, filter by intensity and overlap, merge into single cloud |
| 4 | `04_geometry_analysis.py` | `totalCake_with_planks.ply` | mesh + stats | Isolate top surface, interpolate Z-grid, compute height statistics, detect peaks, Poisson mesh |

**Note on coarse alignment:** step 1 requires a pre-computed coarse alignment `.aln` file (path set in config). This transform accounts for the rough relative pose between scan positions and is produced separately -- see the MATLAB `coarseAlignment.m` script or provide your own rigid transforms.

---

## Pipeline Stages

### 1. `preprocessClouds.py` — Raw Cloud Extraction & Preparation

- Reads Livox LiDAR point clouds from ROS bag files using `rosbags`
- Applies ROI cropping from `p3_config.json`
- Merges and voxel-downsamples each viewpoint
- Reorders scans by field-notes labels (e.g. HABCDGFE → ABCDEFGH)
- Performs RANSAC-based floor alignment (fitting a horizontal plane through the pallet/floor)
- Analyzes normal distributions to locate the floor and isolate the ice surface above it
- Applies coarse alignment transforms from a pre-computed `.aln` file
- Saves preprocessed clouds as individual `.ply` files for ICP input

### 2. `fine_alignment_glira.py` — Fine ICP Alignment

A Python port of the **GliraICP** algorithm (Glira et al. 2015) — a global weighted point-to-plane
Iterative Closest Point algorithm designed for multi-scan alignment.

Key steps:
- Detects overlapping cloud pairs using **voxel hull intersection**
- Uniformly samples correspondences in overlap regions
- Estimates local plane normals and per-point **roughness** using radius search
- Applies four **rejection criteria**: NaN normals, normal angle deviation,
  MAD distance outliers, roughness threshold
- **Weights** correspondences by roughness and normal-plane angle
- Solves a **global weighted least-squares** system (all scans simultaneously)
  using omega-phi-kappa rotation parameterization
- Outputs transformation matrices to `transforms.json` and `.aln` format
- Fully configurable via `p3_config.json` or command-line JSON arguments

### 3. `p3_config.json` — Pipeline Configuration

Central JSON config file controlling all pipeline parameters:

- Output folder and ROS bag file path
- ROI bounds (supports `"inf"` / `"-inf"` strings)
- ICP options: sampling distance, plane search radius, max iterations, rejection thresholds
- Intensity filter range (min/max)
- Geometry parameters: downsample voxel size, interpolation grid resolution
- Scan labels/order and coarse alignment transform path

### 4. `adjustCrop.py` — Interactive ROI Cropping

Interactive tool for defining the Region of Interest before running the main pipeline:

- Loads a test point cloud (`testout.vtp`, generated during preprocessing)
- Accepts manual ROI bounds as `[[xmin,xmax],[ymin,ymax],[zmin,zmax]]`
- Shows a live 3D preview of the cropped cloud using PyVista
- Saves accepted bounds directly back to `p3_config.json`

See also: `roi_editor.py` — a widget-based GUI front-end using Open3D GUI for the same task.

### 5. `postICP.py` — Post-Alignment Merging & Filtering

Applies ICP results and merges all aligned scans into a single cloud:

- Loads pre-ICP clouds (`.ply`) and ICP transforms (`.aln`)
- Applies transforms to each cloud
- Filters by intensity range (removes low/high reflectivity points)
- Filters to retain only **overlapping regions** across scans
- Applies inverse floor transform to restore the original coordinate frame
- Concatenates and saves as `totalCake_with_planks.ply`

### 6. `geometry_analysis.py` — Surface Extraction & Geometry Metrics

Performs final geometric characterisation of the merged cloud:

- Downsamples to a grid resolution
- **Isolates top surface**: projects max-Z per cell → extracts Z-dominant normals
  → removes outliers → fills gaps
- Adds synthetic flat floor points for closed-surface meshing
- **Interpolates** Z values onto a regular grid within the convex hull
- Computes height statistics: mean, median, std, MAD, skewness, kurtosis
- Detects **interest peaks** using Rayleigh criterion with slope analysis
- Performs **Poisson surface reconstruction** with density-based pruning
- Outputs final mesh and visualises with diffuse lighting

---

## Internal Tools & Library Modules

Modules in `tools/` can be imported directly in your own scripts:

```python
import tools.pcsurface as pcsurface
import tools.pcmesh as pcmesh

pc_top = pcsurface.isolate_surface(cloud)
mesh   = pcmesh.pc_to_surface_mesh(pc_top, depth=9)
```

| Module | Purpose |
|--------|---------|
| `pcread.py` | ROS1 bag ingestion: `ros_get_raw`, `read_livox_pointcloud2`, `get_min_sz`; designed for ROS2 and other format extensions |
| `pcio.py` | Pipeline file I/O: read/write `.aln` transforms, load/save `.ply` clouds, dict-to-Open3D conversion |
| `pctransform.py` | Geometric transforms: floor alignment (RANSAC), axis-angle rotation, ROS coordinate conversion, scan reordering, normal-distribution analysis |
| `pcprocess.py` | Low-level cloud manipulation: ROI crop, merge, voxel downsample, concatenate, overlap filter, intensity filter |
| `pcsurface.py` | Surface extraction pipeline: max-Z projection, normal-dominant filtering, outlier removal, gap fill, scattered interpolation, peak detection |
| `pcmesh.py` | Mesh generation and repair: Poisson reconstruction, flat bottom closing, MeshFix repair, boundary loop detection |
| `pcdisplay.py` | PyVista-based 3D visualisation: intensity-coloured display, interactive plotter |
| `roiutils.py` | Encode/decode ROI bounds with `±inf` support for JSON serialisation |

## Key External Libraries

| Library | Role |
|---------|------|
| `open3d` | Point cloud I/O, KNN/radius search, voxel downsampling, ICP, Poisson reconstruction |
| `rosbags` | Reading Livox LiDAR data from ROS `.bag` files |
| `pyvista` | Interactive 3D visualisation |
| `numpy` / `scipy` | Numerical operations, scattered interpolation, statistics |
| `trimesh` / `pymeshfix` | Mesh repair (MeshFix/Netfabb), flat-bottom polygon closing |
| `shapely` | Convex hull and polygon boundary operations |
| `matplotlib` | 2D plots (histograms, height profiles) |

---

## Unused / Junk Files

The following files are candidates for deletion or relocation to a `_prototyping/` folder:

**Moved to `_prototyping/`:**
- **`extractclouds.py`** — Extraction logic is fully absorbed into `preprocessClouds.py`.
  No longer needed as a standalone script.
- **`extractPointcloud.py`** — Single-file test script for one bag file.
  Scratch code superseded by `preprocessClouds.py`.
- **`icpTest.py`** — Tests Open3D's built-in ICP, not GliraICP.
  Pure experimentation; no role in the production pipeline.
- **`testVista.py`** — One-off screenshot export script.
  Useful only for publication figures; not part of the pipeline.
- **`roi_editor.py`** — Open3D GUI front-end for ROI editing. `adjustCrop.py` covers the same
  need more directly; this is a heavier alternative kept for reference.
- **`simpleView.py`** — Command-line PLY viewer. Useful ad-hoc but not part of the pipeline.

**Moved to `output/`:**
- **`testout.vtp`** — Intermediate test cloud generated during preprocessing.
- **`totalCake.ply`** — Final output artifact.
- **`output.aln`** — Intermediate alignment output.
- **`transforms.json`** — Intermediate ICP output.
- **`floor_tforms.aln`** — Intermediate preprocessing output.

**Git hygiene:**
- **`__pycache__/`** — Should be git-ignored; do not track in version control.

---

## Folder Structure

```text
python-3DRecon/
│
├── config/
│   └── p3_config.json                  # All pipeline parameters — loaded by all pipeline scripts
│
├── pipeline/                           # Executable stage scripts — run in order
│   ├── 01_preprocessClouds.py
│   ├── 02_fine_alignment_glira.py
│   ├── 03_postICP.py
│   └── 04_geometry_analysis.py
│
├── tools/                              # Reusable library modules
│   ├── __init__.py
│   ├── pcread.py
│   ├── pcio.py
│   ├── pctransform.py
│   ├── pcprocess.py
│   ├── pcsurface.py
│   ├── pcmesh.py
│   ├── pcdisplay.py
│   └── roiutils.py
│
├── utils/                              # Interactive / standalone utilities
│   └── adjustCrop.py
│
├── output/                             # Generated files — add to .gitignore
│   ├── floor_tforms.aln
│   ├── output.aln
│   ├── transforms.json
│   ├── testout.vtp
│   ├── totalCake_with_planks.ply
│   └── totalCake.ply
│
├── _prototyping/                       # Superseded scripts — kept for reference
│   ├── preprocessClouds.py             # original pipeline scripts (pre-restructure)
│   ├── fine_alignment_glira.py
│   ├── postICP.py
│   ├── geometry_analysis.py
│   ├── pctools.py                      # original monolithic tool modules
│   ├── pcanalysis.py
│   ├── pcalign.py
│   ├── pcdisplay.py
│   ├── extractclouds.py                # experimental / one-off scripts
│   ├── extractPointcloud.py
│   ├── icpTest.py
│   ├── testVista.py
│   ├── roi_editor.py
│   └── simpleView.py
│
└── README.md
```

### Rationale

- **`pipeline/`** — Numbered scripts make execution order explicit at a glance.
- **`tools/`** — Library code cleanly separated from executable scripts.
  Import pattern: `import tools.pcprocess as pcprocess` → call as `pcprocess.crop_o3d_t(...)`.
- **`utils/`** — Interactive helpers used during data collection/review,
  not part of the automated pipeline.
- **`output/`** — All generated artifacts in one place; easily `.gitignore`d.
- **`_prototyping/`** — Keeps experimental code accessible without cluttering production.
- **`config/`** — Config isolated from code; easy to swap for different datasets.
