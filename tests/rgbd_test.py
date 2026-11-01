#!/usr/bin/env python3
"""
test_realsense.py — Test Intel RealSense Depth + RGB simultaneously.
Requires: pip install pyrealsense2 opencv-python numpy
Press 'q' to quit.
"""
import pyrealsense2 as rs
import numpy as np
import cv2

def main():
    # Create a pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable Color and Depth streams (640x480 @ 30fps)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    print("[*] Starting pipeline...")
    try:
        pipeline.start(config)
    except Exception as e:
        print(f"[!] Failed to start pipeline: {e}")
        print("    - Check USB connection (must be USB 3.0).")
        print("    - You may need to run: sudo apt install librealsense2-udev-rules")
        return

    # Create an align object to align depth to color
    align_to = rs.stream.color
    align = rs.align(align_to)

    print("[+] Camera streaming. Press 'q' to quit.")

    try:
        while True:
            # Wait for a coherent pair of frames
            frames = pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap to depth for visualization
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )

            # Stack the two images horizontally
            images = np.hstack((color_image, depth_colormap))

            # Show the images
            cv2.imshow('RealSense Test (RGB + Depth)', images)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("[*] Pipeline stopped.")

if __name__ == "__main__":
    main()
