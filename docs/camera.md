# Live Camera in RViz

[← Back to README](../README.md) · [Troubleshooting](troubleshooting.md#camera) · [Waypoint/tour setup](tour.md)

The Go2's front camera publishes on `/frontvideostream` as
`unitree_go/Go2FrontVideoData` — a custom Unitree message type that RViz
cannot render. The relay node
[`scripts/nodes/go2_camera_relay.py`](../scripts/nodes/go2_camera_relay.py)
decodes it and republishes as standard `sensor_msgs/Image` on
`/camera/image_raw`.

The relay launches automatically when
[`wifi_nav.sh`](../scripts/runtime/wifi_nav.sh) runs (unless `--no-camera`).

In simulation (`wifi_nav.sh --sim`), Gazebo publishes `/camera/image_raw`
directly as `sensor_msgs/Image`, so the relay isn't started. Use the same RViz
Image display. See [gazebo/README.md](../gazebo/README.md).

## Verify the relay is publishing

```bash
go2sh
ros2 topic hz /camera/image_raw
# expect ~10–30 Hz
```

## Calibration and LiDAR overlay

To use the camera together with the LiDAR, calibrate it once
(`./scripts/tools/calibrate_camera.sh`) so the relay publishes
`/camera/camera_info`, then check the alignment with
`./scripts/tools/lidar_overlay.sh`. See [camera_lidar_fusion.md](camera_lidar_fusion.md).

## See also

- [Troubleshooting → Camera](troubleshooting.md#camera) — empty Image display, QoS mismatch, stutter
- [tour.md](tour.md) — adding the Image display to the saved RViz layout ([`config/tour.rviz`](../config/tour.rviz))
- [roadmap.md](roadmap.md#milestone-3--camera-pipeline) — camera pipeline milestone
