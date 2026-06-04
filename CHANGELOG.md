# Changelog
All notable changes documented here. Linked to git commits by hash.

---

## [Unreleased]
### Added
### Changed
### Fixed

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
  - Commit: `[pending]`
- `python-3DRecon/pipeline/`: numbered executable stage scripts replacing flat root scripts
  - `01_preprocessClouds.py`
  - `02_fine_alignment_glira.py`
  - `03_postICP.py`
  - `04_geometry_analysis.py`
  - All scripts use `sys.path.insert` for portable imports; all import from `tools/`
  - Commit: `[pending]`
- `python-3DRecon/config/`: isolated config directory
  - `p3_config.json` moved from project root; all pipeline scripts load via `__file__`-relative path
  - Commit: `[pending]`
- `python-3DRecon/utils/`: interactive utilities directory
  - `adjustCrop.py` moved from project root; imports updated for subfolder location
  - Commit: `[pending]`
- `python-3DRecon/output/`: generated artifact directory; added to `.gitignore`
  - Received: `testout.vtp`, `totalCake.ply`, `output.aln`, `transforms.json`, `floor_tforms.aln`
  - Commit: `[pending]`
- `python-3DRecon/_prototyping/`: superseded scripts archive; added to `.gitignore`
  - Old pipeline scripts: `preprocessClouds.py`, `fine_alignment_glira.py`, `postICP.py`, `geometry_analysis.py`
  - Old monolithic tool modules: `pctools.py`, `pcanalysis.py`, `pcalign.py`, `pcdisplay.py`
  - Experimental/one-off scripts: `extractclouds.py`, `extractPointcloud.py`, `icpTest.py`, `testVista.py`, `roi_editor.py`, `simpleView.py`
  - Commit: `[pending]`

### Changed
- `.gitignore`: added `python-3DRecon/output/` and `python-3DRecon/_prototyping/`
  - Commit: `[pending]`

---