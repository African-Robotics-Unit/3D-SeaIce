# cpp — RealSense C++ Programs

## Layout

```
cpp/
├── Makefile
├── run                        # launcher script (handles Ubuntu pthread fix)
├── include/                   # shared headers
├── src/                       # shared library sources (compiled once)
└── programs/
    ├── rs_record/             # record depth + colour to .bag
    ├── rgb_viewer/            # live RGB preview
    └── rec_viewer/            # record + live depth/colour preview with IMU stats
```

## Build

```bash
# Build all programs
make

# Build one program
make rs_record
make rec_viewer

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
```

## Adding a new program

1. Create `programs/<name>/main.cpp`
2. Add `<name>` to the `PROGRAMS` line in the Makefile:
   ```makefile
   PROGRAMS := rs_record rgb_viewer rec_viewer <name>
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
