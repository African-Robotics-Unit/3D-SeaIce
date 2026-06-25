# cpp — RealSense C++ Programs

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
    ├── rs_clean/              # clean recorder with live display (recommended)
    └── rs_marker/             # live AprilTag detection with optional recording + CSV log
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

./run rs_marker                          # live AprilTag detection, no recording
./run rs_marker -o session.bag           # detect + record bag until Ctrl+C or q
./run rs_marker -o session.bag -d 60     # detect + record for 60 s
./run rs_marker --help
```

Press `q` in any display window or `Ctrl+C` in the terminal to stop programs
that show a live preview.

## rs_marker — AprilTag detection

`rs_marker` detects AprilTag 36h11 family tags (target IDs: 92, 93, 94, 95)
in the live colour stream and overlays the results on screen.

**Output files** (written alongside the bag, or in the working directory if
not recording):

| File | Contents |
|------|----------|
| `session.bag` | Raw sensor frames — Depth, Color, Accel, Gyro |
| `session_detections.csv` | One row per detected tag per frame |

**CSV columns:**

```
timestamp_ms  — ms since program started (matches bag timeline)
frame         — frame counter
marker_id     — detected AprilTag ID
cx, cy        — pixel centre of the tag in the colour frame
depth_m       — depth at that pixel from the filtered depth frame (metres)
c0x..c3y      — four corner pixel coordinates (clockwise from top-left)
```

Only frames where a target tag is visible produce rows — no detections, no row.

**Analysing results** with the summary tool:

```bash
python3 tools/summarize_markers.py detections.csv
python3 tools/summarize_markers.py session_detections.csv --top 20
```

The summary reports which frames had the most markers visible simultaneously
and recommends the best frame to use.

**Editing the detection logic** — the marker detection section in
`programs/rs_marker/main.cpp` is clearly fenced between `═══` banners.
To change which tag IDs are tracked, edit `TARGET_IDS`. The `detect_markers()`
and `draw_detections()` functions are the only ones that need to change for
detection experiments.

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

## Non-standard install paths

```bash
make CXXFLAGS="-std=c++14 -O2 -Iinclude -I/opt/librealsense2/include" \
     LIBS="-L/opt/librealsense2/lib -lrealsense2 -lpthread"
```
