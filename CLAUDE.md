# Project Context: Unitree GO2 + D1 Arm
- **Hardware:** Unitree GO2 with a D1 arm mounted facing the BACK of the dog.
- **Network:** Camera (RealSense) runs on DDS Domain 64. MoveIt runs on DDS Domain 78.
- **Current Objective:** Final semester demo in Gazebo.

## The Pipeline Goal
1. Spawn a CropKing NFT 4-6 hydroponic rack with a realistic lettuce plant.
2. Dog drives to the rack (Nav2) and parks tail-first.
3. Arm lifts to a forward-facing pose.
4. Simulated camera runs YOLO.
5. Arm uses `distance_servo.py` to approach the plant to 0.50m.
6. Capture an image + JSON data.
7. Generate a matplotlib graph showing the arm's distance over time to prove the servo worked.

## Rules for the Agent
- ALWAYS test in Gazebo headless mode (`gz sim -s`) and check `ros2 topic echo` to verify logic, rather than relying on GUI.
- Do not modify real-hardware scripts (like `arm_moveit.sh --real`) unless explicitly asked.
- Save all new Gazebo assets in `gazebo/worlds/`.
