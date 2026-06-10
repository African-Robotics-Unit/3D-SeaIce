# Changelog
All notable changes documented here. Linked to git commits by hash.

---

## [Unreleased] -- 2026-06-10

### Added
- `show_pointcloud_color()` in `pcdisplay.py` for RGB RealSense point cloud visualisation via PyVista
- `python-3DRecon/realsense/` folder: `extractRS.py` (pyrealsense2 accelerometer reader) and `metadata_gen.py` (ROS1 bag metadata CSV generator)
- `python-3DRecon/config/2blocks_config.json` for ROS2 two-block test dataset
- `Agi-Map/` folder: four Basemap figure scripts for SCALE22 SIC/cruise-track/station visualisation (`Agi_SCALE22`, `Agi_SCALE22_field`, `_new`, `_new_field`) plus associated data files

### Changed
- `01_preprocessClouds.py`: wrapped in `main(config_name)` with argparse; ROS1 and ROS2 now dispatched via `source_type` config key; ROS2 path adds `joblib` caching
- `pcread.py`: all point cloud readers now return `o3d.t.geometry.PointCloud` instead of raw dicts; `ros2_get_raw` adds `read_rs` flag and per-bag progress printing; RealSense `itemsize` fixed to `msg.point_step`
- `pcprocess.py`: removed redundant `dict_to_o3d_t` call in `crop_merge_downsample` (clouds are now native tensor objects at read time)

---

## [2026-06-04] -- Repo Organisation and Root README

### Added
- `legacy/`: archive folder; 7 original MATLAB pipeline folders moved from repo root
  - `3D Reconstruction`, `CoarseAlignment`, `DataAnalysis`, `FineAlignment`, `MeshGeneration`, `UserGuide`, `ExtractClouds`
  - Commit: `[pending]`

### Changed
- `README.md`: full rewrite with Python pipeline quick-start, config reference, pipeline stage table with inputs/outputs, tools module reference, MATLAB pipeline overview, legacy description, GliraICP citation
  - Commit: `[pending]`

---

## [2026-06-04] -- python-3DRecon Restructure

### Added
- `python-3DRecon/README.md`: full pipeline documentation covering stages, tools, junk files, and folder structure
- `python-3DRecon/tools/` package with restructured library modules:
  - `pcread.py`: ROS1 bag ingestion (`ros_get_raw`, `read_livox_pointcloud2`, `get_min_sz`); designed for ROS2 extension
  - `pcio.py`: pipeline file I/O -- `.aln` read/write, `.ply` load/save, dict-to-Open3D conversion
  - `pctransform.py`: floor alignment (RANSAC), axis-angle rotation, ROS coordinate conversion, scan reordering, normal-distribution analysis
  - `pcprocess.py`: low-level cloud manipulation -- ROI crop, merge, voxel downsample, concatenate, overlap filter, intensity filter
  - `pcsurface.py`: surface extraction pipeline -- max-Z projection, normal filtering, outlier removal, gap fill, scattered interpolation, peak detection
  - `pcmesh.py`: mesh generation and repair -- Poisson reconstruction, flat bottom closing, MeshFix, boundary loop detection
  - `pcdisplay.py`: PyVista visualisation -- intensity-coloured display, interactive background plotter
  - `roiutils.py`: moved from project root into `tools/`
  - Commit: `462db5b`
- `python-3DRecon/pipeline/`: numbered executable stage scripts replacing flat root scripts
  - `01_preprocessClouds.py`
  - `02_fine_alignment_glira.py`
  - `03_postICP.py`
  - `04_geometry_analysis.py`
  - All scripts use `sys.path.insert` for portable imports; all import from `tools/`
  - Commit: `462db5b`
- `python-3DRecon/config/`: isolated config directory
  - `p3_config.json` moved from project root; all pipeline scripts load via `__file__`-relative path
  - Commit: `462db5b`
- `python-3DRecon/utils/`: interactive utilities directory
  - `adjustCrop.py` moved from project root; imports updated for subfolder location
  - Commit: `462db5b`
- `python-3DRecon/output/`: generated artifact directory; added to `.gitignore`
  - Received: `testout.vtp`, `totalCake.ply`, `output.aln`, `transforms.json`, `floor_tforms.aln`
  - Commit: `462db5b`
- `python-3DRecon/_prototyping/`: superseded scripts archive; added to `.gitignore`
  - Old pipeline scripts: `preprocessClouds.py`, `fine_alignment_glira.py`, `postICP.py`, `geometry_analysis.py`
  - Old monolithic tool modules: `pctools.py`, `pcanalysis.py`, `pcalign.py`, `pcdisplay.py`
  - Experimental/one-off scripts: `extractclouds.py`, `extractPointcloud.py`, `icpTest.py`, `testVista.py`, `roi_editor.py`, `simpleView.py`
  - Commit: `462db5b`

### Changed
- `.gitignore`: added `python-3DRecon/output/` and `python-3DRecon/_prototyping/`
  - Commit: `462db5b`

---