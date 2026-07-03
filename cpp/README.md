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

This records 25 frames and puts the following in your output directory. All April Tags in the 36h11 series will be detected.

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
│   └── summarize_markers.py   # analyse a detections CSV from rs_marker
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
