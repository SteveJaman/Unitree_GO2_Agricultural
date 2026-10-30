#!/usr/bin/env python3
"""
test_arm_tracker_pure.py

Bypasses ROS 2 completely.
1. Reads directly from the USB RealSense camera (pyrealsense2).
2. Runs YOLO on the frame.
3. Sends joint commands to the D1 arm via cyclonedds (using d1_arm_control.py).
"""

import cv2
import time
import sys
import os
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# Import your existing D1Arm class
_here = os.path.dirname(os.path.abspath(__file__))
_repo = os.path.dirname(os.path.dirname(_here))
sys.path.insert(0, os.path.join(_repo, "scripts", "nodes"))
try:
    from d1_arm_control import D1Arm, ENABLE_FULL, FUNC_JOINTS, FUNC_JOINT, default_arm_iface
except ImportError as e:
    sys.exit(f"Could not import D1Arm: {e}")

def main():
    print("[*] Initializing YOLO...")
    model = YOLO('yolo11s.pt')

    print("[*] Connecting to D1 Arm...")
    arm = D1Arm(default_arm_iface())
    if not arm.ok:
        sys.exit("[!] Failed to connect to the arm via DDS.")

    print("[*] Moving arm to READY pose...")
    arm.power(True)
    time.sleep(1.0)
    arm.enable(ENABLE_FULL)
    time.sleep(1.0)

    # Ready pose (Arm lifted and pointing forward)
    ready_pose = {"mode": 1, "angle0": 0, "angle1": -40.0, "angle2": 60.0, "angle3": 0, "angle4": 30.0, "angle5": 0, "angle6": 0}
    arm._send(FUNC_JOINTS, ready_pose)
    time.sleep(3.0)

    print("[*] Initializing RealSense Camera...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    print("[+] Tracking started! Press 'q' to stop.")

    # State variables
    base_j0 = 0.0
    base_j1 = -40.0
    step_size = 1.5
    deadzone = 30

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert RealSense frame to OpenCV format
            frame = np.asanyarray(color_frame.get_data())

            # Run YOLO
            results = model(frame, verbose=False, classes=[0]) # class 0 = person

            target_box = None
            max_area = 0
            for box in results[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                area = (x2 - x1) * (y2 - y1)
                if area > max_area:
                    max_area = area
                    target_box = (x1, y1, x2, y2)

            if target_box:
                x1, y1, x2, y2 = target_box
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2

                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)

                err_x = cx - 320
                err_y = cy - 240

                moved = False

                # Adjust Base (Left/Right)
                if abs(err_x) > deadzone:
                    base_j0 += (step_size if err_x < 0 else -step_size)
                    moved = True

                # Adjust Shoulder (Up/Down)
                if abs(err_y) > deadzone:
                    base_j1 += (step_size if err_y < 0 else -step_size)
                    moved = True

                if moved:
                    base_j0 = max(-135.0, min(135.0, base_j0))
                    base_j1 = max(-90.0, min(90.0, base_j1))

                    arm._send(FUNC_JOINT, {"id": 0, "angle": base_j0, "delay_ms": 0})
                    arm._send(FUNC_JOINT, {"id": 1, "angle": base_j1, "delay_ms": 0})
                    print(f"Tracking: J0={base_j0:.1f}°, J1={base_j1:.1f}°")
            else:
                cv2.putText(frame, "No person detected", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            cv2.imshow('Arm Tracker', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n[!] Interrupted.")
    finally:
        print("[*] Releasing arm and closing camera...")
        arm.enable(ENABLE_FULL)
        time.sleep(0.5)
        arm.release()
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
