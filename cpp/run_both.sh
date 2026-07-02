#!/bin/bash

# Launch C++ rs_marker
/home/aru/agi/3D-SeaIce/cpp/run rs_marker -n 25 -D /home/aru/Documents/validationTests &
CPP_PID=$!

# Launch Python software_sync_capture_new.py
python3 /home/aru/agi/Masters-Project_Antarctic_Sensor_Suite/src/Software_Sync/scripts/software_sync_capture_new.py \
    --frames 100 --lidar-only --lidar-format lvx &
PYTHON_PID=$!

echo "C++ PID: $CPP_PID"
echo "Python PID: $PYTHON_PID"

# Wait for both to finish
wait $CPP_PID; CPP_EXIT=$?
wait $PYTHON_PID; PYTHON_EXIT=$?

echo "C++ exited with code $CPP_EXIT"
echo "Python exited with code $PYTHON_EXIT"
