import pyrealsense2 as rs

pipeline = rs.pipeline()
config   = rs.config()
config.enable_device_from_file(
    r"C:\Users\agori\Downloads\test_converted.bag",
    repeat_playback=False
)

try:
    pipeline.start(config)
    print("Pipeline started OK")
    frames = pipeline.wait_for_frames(timeout_ms=5000)
    depth  = frames.get_depth_frame()
    print(f"Got depth frame: {depth.is_valid()}, shape: {depth.get_width()}x{depth.get_height()}")
    pipeline.stop()
except Exception as e:
    print(f"FAILED: {e}")