# Processing Reference

## Recording

**rs_marker program**

| File | Contents |
|------|----------|
| `<YYYYMMDD_HHMMSS>.bag` | Raw sensor frames — Depth, Color, Accel, Gyro |
| `<YYYYMMDD_HHMMSS>_detections.csv` | One row per detected tag per frame |
| `<YYYYMMDD_HHMMSS>.ply` | Point cloud of the best frame (most valid depth pixels / fewest holes), fully post-processed |
| `imu.csv` | Accel + gyro sample captured alongside the best frame |

## Post-processing steps

Using the output form of rs_marker, separate the bag files and the collected data (ply, detections, imu).
`.bag` files are very large: only needed if you need to recapture something → use the smaller extracted files for processing.

### collect_plys.py

`cpp/tools` → `collect_plys.py`

```bash
python3 collect_plys.py /media/aru/Seagate/roughness/20260716
```

Default mode (no dst given):
```
/media/aru/Seagate/roughness/20260716_collected/  <- ply, detections, imu
/media/aru/Seagate/roughness/20260716_raw/  <- bag (flat)
```

### tabulate_data.py

```bash
python3 tabulate_data.py /media/aru/Seagate/roughness/20260716
```

Creates metadata table.

**Columns:**

| Column | Source |
|--------|--------|
| `Name`, `Start`, `End`, `Duration`, `Size` | bag header + filesystem |
| `Device`, `Serial`, `Firmware` | `/device_0/info` in the bag |
| `Depth_Frames`, `Color_Frames` | per-topic message counts |
| `Depth_FPS`, `Color_FPS` | frame count ÷ duration |
| `Depth_Res`, `Color_Res` | `camera_info` width×height |
| `Accel_Samples`, `Gyro_Samples` | per-topic message counts |
| `marker_csv`, `ply_file`, `imu_csv` | matching filename in `<src>_collected/`, else `NaN` |
| `num_detections`, `num_unique_markers` | row count / unique `marker_id` count in `marker_csv`, else `NaN` |

A recording that aborted early (e.g. camera disconnected mid-capture) shows up clearly this way — e.g. depth frames present but `Color_Frames = NaN` (no color stream ever arrived), which is also why `marker_csv`/`ply_file`/`imu_csv` are `NaN` for it.

### marker_points.py

```bash
python3 marker_points.py /media/aru/Seagate/roughness/20260716_raw /media/aru/Seagate/roughness/20260716_collected
```

New folder created: `20260714_collected/marker_csv`. Used in MATLAB extraction.

### plot_imu.py & plot_gyro_outliers.py

Just to get a sense of the IMU data. Ideally use the extracted imu csv to perform plane adjustment.

```bash
python3 tools/plot_imu.py /media/aru/Seagate/roughness/20260714_raw
```

With no `--graphs-dir`/`--csv-dir`, this writes two folders next to the bags:

| Folder | Contents |
|--------|----------|
| `20260714_raw/imu_graphs/` | `<stem>_imu.png` — stacked accel/gyro-over-time figure, x/y/z per panel |
| `20260714_raw/imu_total/` | `<stem>_imu.csv` — columns `timestamp_ms,stream,x,y,z`, one row per raw motion sample (`stream` is `accel` or `gyro`), sorted by timestamp |

`plot_gyro_outliers.py` takes the `imu_total/` folder written above and flags/plots gyroscope outliers across the whole dataset:

```bash
python3 tools/plot_gyro_outliers.py /media/aru/Seagate/roughness/20260714_raw/imu_total
```

Writes `gyro_outliers.png` (default: `<csv_dir>/../imu_outliers`).

### Handoff

Upload outputs to Google Drive → move onto MATLAB processing.
