# 3D Reconstruction of Pancake Sea Ice

**Agoritsa Spirakis** | MSc Eng Electrical Engineering, ARU/MARIS

Static 3D reconstruction of pancake sea ice from multi-viewpoint Livox LiDAR scans collected onboard a research vessel. The pipeline extracts raw sensor data from ROS bags, aligns scans using a global weighted ICP algorithm, and produces a 3D surface mesh with height statistics and roughness metrics.

---

## Repo Structure

| Folder | Contents |
|---|---|
| [python-3DRecon/](python-3DRecon/README.md) | Primary Python pipeline: config, pipeline stages, library modules, utilities |
| [matlab/](matlab/README.md) | Reference MATLAB pipeline, same config format |
| [cpp/](cpp/README.md) | RealSense + LiDAR capture programs (C++) |
| [legacy/](legacy/README.md) | Original pre-restructure MATLAB scripts, reference only |

Each folder's README has full setup, build, and run instructions.

---

## Reference

P. Glira, N. Pfeifer, C. Briese, C. Ressl (2015). *A Correspondence-Based Algorithm for Point Cloud Registration with High Accuracy and Speed*. ISPRS Annals of Photogrammetry, Remote Sensing and Spatial Information Sciences.
