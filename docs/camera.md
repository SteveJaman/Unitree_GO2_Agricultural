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

## Verify the relay is publishing

```bash
go2sh
ros2 topic hz /camera/image_raw
# expect ~10–30 Hz
```

## See also

- [Troubleshooting → Camera](troubleshooting.md#camera) — empty Image display, QoS mismatch, stutter
- [tour.md](tour.md) — adding the Image display to the saved RViz layout ([`config/tour.rviz`](../config/tour.rviz))
- [roadmap.md](roadmap.md#milestone-3--camera-pipeline) — camera pipeline milestone