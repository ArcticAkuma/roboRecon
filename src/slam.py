import pyrealsense2 as rs
import rtabmap.util
import rtabmap
import time
import numpy as np

# Configuration for the Intel RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the RealSense pipeline
pipeline.start(config)

# RTAB-Map initialization
parameters = rtabmap.util.defaultParameters()
# Adjust parameters as needed
parameters["Rtabmap/DetectionRate"] = "2"  # Set frame processing rate
rtabmap_system = rtabmap.Rtabmap()
rtabmap_system.init(parameters)

try:
    print("Starting RTAB-Map mapping...")
    start_time = time.time()

    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert RealSense frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Pass the images to RTAB-Map
        success = rtabmap_system.process(
            color_image,
            depth_image,
            depth_frame.profile.as_video_stream_profile().intrinsics.fx,
            depth_frame.profile.as_video_stream_profile().intrinsics.fy,
            depth_frame.profile.as_video_stream_profile().intrinsics.ppx,
            depth_frame.profile.as_video_stream_profile().intrinsics.ppy,
            time.time() - start_time
        )

        if not success:
            print("Failed to process frame!")

        # Optionally, visualize the map or perform live monitoring
        if time.time() - start_time > 60:  # Stop after 1 minute
            break

finally:
    # Stop the RealSense pipeline
    pipeline.stop()

    # Save the map to a file
    map_file = "room_map.db"
    rtabmap_system.save(map_file)
    print(f"Map saved to {map_file}")