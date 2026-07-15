# cpp — RealSense C++ Programs
## Prerequisites

### librealsense2

Follow the [official Intel guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) if you get stuck

```bash
sudo apt install librealsense2-dev librealsense2-utils
```

### opencv
```bash
sudo apt install libopencv-dev
```

## Check if Realsense installed
This will open the Realsense Viewer
```bash
realsense-viewer
```

## Simple instructions for rs_marker
Currently on focus on **rs_marker** program please as this has all the updated settings
In the cpp folder:

1. Compile the code:
In the cpp folder:
```
make rs_marker
```

2. Run the code:
```
./run rs_marker -n 25 -D /path/to/output_dir
```

This records 25 frames and puts the following in your output directory.


| File | Contents |
|------|----------|
| `<YYYYMMDD_HHMMSS>.bag` | Raw sensor frames — Depth, Color, Accel, Gyro |
| `<YYYYMMDD_HHMMSS>_detections.csv` | One row per detected tag per frame |
| `<YYYYMMDD_HHMMSS>.ply` | Point cloud of the best frame (most valid depth pixels / fewest holes), fully post-processed |
| `imu.csv` | Accel + gyro sample captured alongside the best frame |

## Running instructions: just realsense
```
/home/aru/agi/3D-SeaIce/cpp/run rs_marker -n 25 -D /home/aru/Documents/validationTests
```

## Lidar and Realsense
```
./run_both.sh
```


## Layout

```
cpp/
├── Makefile
├── run                        # launcher script (handles Ubuntu pthread fix)
├── include/                   # shared headers
├── src/                       # shared library sources (compiled once)
├── tools/
│   ├── summarize_markers.py   # analyse a detections CSV from rs_marker
│   ├── marker_points.py       # extract 3D marker positions from a .bag + detections CSV
│   ├── collect_plys.py        # gather .bag/.ply/detections/imu files across many recordings
│   └── tabulate_data.py       # one-row-per-recording metadata CSV from bag headers + collected files
└── programs/
    ├── rs_record/             # record depth + colour to .bag
    ├── rgb_viewer/            # live RGB preview
    ├── rec_viewer/            # record + live depth/colour preview with IMU stats
    ├── rs_clean/              # clean recorder with live display (recommended)
    └── rs_marker/             # live AprilTag detection with optional recording + CSV log
```

## Dependencies

### OpenCV (with ArUco support)

`rs_marker` requires OpenCV with the ArUco module. On Ubuntu 22.04 the apt
package includes it:

```bash
sudo apt update
sudo apt install libopencv-dev
```

Verify ArUco is present after install:

```bash
pkg-config --modversion opencv4        # should print 4.x.x
grep -r aruco /usr/include/opencv4/opencv2/ --include="*.hpp" -l
```

If ArUco headers are missing (older distros or minimal installs), build
OpenCV from source with the contrib modules:

```bash
sudo apt install cmake build-essential libgtk2.0-dev pkg-config
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
cd opencv && mkdir build && cd build
cmake .. -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
         -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

### librealsense2

Follow the [official Intel guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) to add the apt repository, then:

```bash
sudo apt install librealsense2-dev librealsense2-utils
```

## Build

```bash
# Build all programs
make

# Build one program
make rs_clean
make rs_marker

# Clean all build artefacts and binaries
make clean
```

Binaries land in `cpp/` alongside the Makefile.

## Run

Use the `./run` script rather than calling binaries directly — it applies a
pthread fix required on Ubuntu 22.04+ when snap is installed.

```bash
./run rs_record                          # record to d455_record.bag for 30 s (defaults)
./run rs_record my_capture.bag 60        # custom file and duration

./run rgb_viewer                         # live RGB preview — press q to quit

./run rec_viewer                         # record + preview until Ctrl+C
./run rec_viewer -o capture.bag -d 60    # custom file and duration
./run rec_viewer --help                  # show all options

./run rs_clean                           # record + preview (recommended recorder)
./run rs_clean -o session.bag -d 120     # named file, 2-minute capture

./run rs_marker                                      # live AprilTag detection, no recording
./run rs_marker -o session.bag                       # detect + record bag until Ctrl+C or q
./run rs_marker -o session.bag -d 60                 # detect + record for 60 s
./run rs_marker -n 25 -D /path/to/output_dir         # detect for 25 frames, save timestamped bag + CSV
./run rs_marker --help
```

Press `q` in any display window or `Ctrl+C` in the terminal to stop programs
that show a live preview.

## rs_marker — AprilTag detection

`rs_marker` detects **all** AprilTag 36h11 family tags in the live colour
stream and overlays the results on screen.

**Flags:**

| Flag | Description |
|------|-------------|
| `-o <file.bag>` | Record to a named bag file |
| `-D <dir>` | Create a timestamped session dir inside `<dir>` and save bag + CSV there |
| `-d <seconds>` | Stop after N seconds |
| `-n <frames>` | Stop after N frames |
| `-test` | RGB-only mode: stream with marker overlays, save annotated video |
| `-id` | RGB-only mode: detect all 36h11 tags and log every unique ID seen |

`-o` and `-D` cannot be combined. `-test` and `-id` cannot be combined.

**Output files with `-D`:**

When `-D <dir>` is given, a session subdirectory `<dir>/<YYYYMMDD_HHMMSS>/` is
created and both files are written there:

| File | Contents |
|------|----------|
| `<YYYYMMDD_HHMMSS>.bag` | Raw sensor frames — Depth, Color, Accel, Gyro |
| `<YYYYMMDD_HHMMSS>_detections.csv` | One row per detected tag per frame |
| `<YYYYMMDD_HHMMSS>.ply` | Point cloud of the best frame (most valid depth pixels / fewest holes), fully post-processed |
| `imu.csv` | Accel + gyro sample captured alongside the best frame |

**Best-frame selection:** in normal mode (not `-test`/`-id`), every depth frame
is fully post-processed (decimation → disparity → spatial → temporal →
disparity → hole-filling) and its valid-pixel count is compared against the
best seen so far. When the run ends, the frame with the fewest remaining
holes is exported to a `.ply` point cloud (textured with its color frame),
and the accel/gyro reading captured alongside it is written to `imu.csv`.

**CSV columns:**

```
timestamp_ms  — ms since program started (matches bag timeline)
frame         — frame counter
marker_id     — detected AprilTag ID
cx, cy        — pixel centre of the tag in the colour frame
depth_m       — depth at that pixel from the filtered depth frame (metres)
c0x..c3y      — four corner pixel coordinates (clockwise from top-left)
```

Only frames where at least one tag is visible produce rows.

**Analysing results** with the summary tool:

```bash
python3 tools/summarize_markers.py detections.csv
python3 tools/summarize_markers.py session_detections.csv --top 20
```

The summary reports which frames had the most markers visible simultaneously
and recommends the best frame to use.

## collect_plys.py — gather outputs across many recordings

Each rs_marker run leaves its `.bag`, `.ply`, `_detections.csv` and `imu.csv`
buried in its own `<YYYYMMDD_HHMMSS>/` session folder. `collect_plys.py`
walks a top-level recordings folder (recursively, across any number of
session subfolders) and gathers those files into flat, per-type output
folders — handy before batch-processing a day's worth of recordings or
archiving just the point clouds.

**Default mode — just pass the recordings folder:**

```bash
python3 tools/collect_plys.py /media/aru/Seagate/roughness/20260714
```

With no `dst`, this is the common case and needs nothing else: it creates
two folders next to `src`,

| Folder | Contents |
|--------|----------|
| `20260714_collected/ply/` | every `.ply` |
| `20260714_collected/detections/` | every `_detections.csv` |
| `20260714_collected/imu/` | every `imu.csv`, renamed `<recording_folder>_imu.csv` |
| `20260714_raw/` | every `.bag`, flat (no subfolder) |

Equivalent to running:

```bash
python3 tools/collect_plys.py /media/aru/Seagate/roughness/20260714 /media/aru/Seagate/roughness/20260714_collected --types ply detections imu
python3 tools/collect_plys.py /media/aru/Seagate/roughness/20260714 /media/aru/Seagate/roughness/20260714_raw       --types bag --flat
```

**Custom mode — pass an explicit `dst` to control which types and layout:**

```bash
# Collect everything (bag, ply, detections, imu) into dst/<type>/
python3 tools/collect_plys.py /media/aru/Seagate/roughness/20260714 /media/aru/Seagate/roughness/20260714_collected

# Collect only .ply and imu.csv
python3 tools/collect_plys.py /media/aru/Seagate/roughness/20260714 /media/aru/Seagate/roughness/20260714_collected --types ply imu

# Symlink instead of copy (saves disk space, e.g. for a quick look in CloudCompare)
python3 tools/collect_plys.py /media/aru/Seagate/roughness/20260714 /media/aru/Seagate/roughness/20260714_collected --mode symlink

# Move instead of copy (source files are removed)
python3 tools/collect_plys.py /media/aru/Seagate/roughness/20260714 /media/aru/Seagate/roughness/20260714_collected --mode move

# Collect just the raw .bag files into a plain folder (no dst/bag/ subfolder)
python3 tools/collect_plys.py /media/aru/Seagate/roughness/20260714 /media/aru/Seagate/roughness/20260714_raw --types bag --flat
```

**Flags** (`--types` and `--flat` require an explicit `dst` — omit both, and `dst`, to use default mode):

| Flag | Description |
|------|-------------|
| `--types {bag,ply,detections,imu}` | Which file type(s) to collect (default: all four) |
| `--mode {copy,symlink,move}` | How to place files in dst (default: `copy`) |
| `--flat` | Write straight into `dst` instead of `dst/<type>/` — use with a single `--types` value |
| `--prefix-parent` | Always prefix output filenames with `<recording_folder>_`, even for types that aren't normally ambiguous |

`.bag`, `.ply` and `_detections.csv` filenames already embed the recording
timestamp, so they land in the output folder unchanged unless that would
collide with another file. `imu.csv` is named identically in every session
folder, so it is always renamed to `<recording_folder>_imu.csv`.

## tabulate_data.py — one-row-per-recording metadata table

Assumes `collect_plys.py` default mode has already been run on `src`, so
`<src>_raw/` and `<src>_collected/{ply,detections,imu}/` exist next to it.
Reads each `.bag`'s header — no pyrealsense2 or ROS install needed, just the
[`rosbags`](https://pypi.org/project/rosbags/) package — and cross-references
`<src>_collected/` for the matching ply/detections/imu file, writing one CSV
row per recording (`NaN` for anything missing or unreadable).

```bash
pip install --break-system-packages rosbags   # one-time

python3 tools/tabulate_data.py /media/aru/Seagate/roughness/20260714
# -> writes /media/aru/Seagate/roughness/20260714_metadata.csv

python3 tools/tabulate_data.py /media/aru/Seagate/roughness/20260714 --out /tmp/meta.csv
```

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

A recording that aborted early (e.g. camera disconnected mid-capture) shows
up clearly this way — e.g. depth frames present but `Color_Frames = NaN`
(no color stream ever arrived), which is also why `marker_csv`/`ply_file`/
`imu_csv` are `NaN` for it.

## Running instructions used for tests: 
```
/home/aru/agi/3D-SeaIce/cpp/run rs_marker -n 25 -D /home/aru/Documents/validationTests
```

## Lidar and Realsense
```
./run_both.sh
```


## Build

```bash
# Build all programs
make

# Build one program
make rs_marker

# Clean all build artefacts and binaries
make clean
```

Binaries land in `cpp/` alongside the Makefile.

## Run

Use the `./run` script rather than calling binaries directly — it applies a
pthread fix required on Ubuntu 22.04+ when snap is installed.

```bash
./run rs_marker                                      # live AprilTag detection, no recording
./run rs_marker -o session.bag                       # detect + record bag until Ctrl+C or q
./run rs_marker -o session.bag -d 60                 # detect + record for 60 s
./run rs_marker -n 25 -D /path/to/output_dir         # detect for 25 frames, save timestamped bag + CSV
./run rs_marker --help
```

Press `q` in any display window or `Ctrl+C` in the terminal to stop programs
that show a live preview.


## Layout

```
cpp/
├── Makefile
├── run                        # launcher script (handles Ubuntu pthread fix)
├── include/                   # shared headers
├── src/                       # shared library sources (compiled once)
├── tools/
│   ├── summarize_markers.py   # analyse a detections CSV from rs_marker
│   ├── marker_points.py       # extract 3D marker positions from a .bag + detections CSV
│   ├── collect_plys.py        # gather .bag/.ply/detections/imu files across many recordings
│   └── tabulate_data.py       # one-row-per-recording metadata CSV from bag headers + collected files
└── programs/
    ├── rs_record/             # record depth + colour to .bag
    ├── rgb_viewer/            # live RGB preview
    ├── rec_viewer/            # record + live depth/colour preview with IMU stats
    ├── rs_clean/              # clean recorder with live display 
    └── rs_marker/             # live AprilTag detection with optional recording + CSV log
```

## Adding a new program

1. Create `programs/<name>/main.cpp`
2. Add `<name>` to the `PROGRAMS` line in the Makefile:
   ```makefile
   PROGRAMS := rs_record rgb_viewer rec_viewer rs_clean rs_marker <name>
   ```
3. If the program needs extra libraries, add per-program overrides below that line:
   ```makefile
   <name>_EXTRA_CFLAGS := $(shell pkg-config --cflags <lib>)
   <name>_EXTRA_LIBS   := $(shell pkg-config --libs   <lib>)
   ```
4. Build and run:
   ```bash
   make <name>
   ./run <name> [args...]
   ```

The shared library (`src/d456_recorder.cpp`) is available to every program
automatically via `-Iinclude` and linked into each binary.
