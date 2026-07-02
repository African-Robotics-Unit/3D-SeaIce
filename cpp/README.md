# cpp — RealSense C++ Programs

## Simple instructions for rs_marker
Currently on focus on **rs_marker** program please as this has all the updated settings
In the cpp folder:

1. Compile the code:
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
│   └── summarize_markers.py   # analyse a detections CSV from rs_marker
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

**Editing the detection logic** — the marker detection section in
`programs/rs_marker/main.cpp` is clearly fenced between `═══` banners.
`detect_all_markers()` detects every visible 36h11 tag; `detect_markers()`
filters by `TARGET_IDS` and is used by `-test` mode only.

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
