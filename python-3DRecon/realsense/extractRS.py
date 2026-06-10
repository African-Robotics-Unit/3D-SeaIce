import pyrealsense2 as rs
# Configure pipeline for file playback
pipe = rs.pipeline()
config = rs.config()
rs.config.enable_device_from_file(config, 'test.bag')
profile = pipe.start(config)

# Iterate through frames
try:
    while True:
        frames = pipe.wait_for_frames()
        accel = frames.first_or_default(rs.stream.accel)
        if accel:
            data = accel.as_motion_frame().get_motion_data()
            print(f"Accel: {data.x}, {data.y}, {data.z}")
except RuntimeError:
    print("End of bag file")
finally:
    pipe.stop()
