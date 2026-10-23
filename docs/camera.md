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

## Show it in RViz

1. Click **Add** at the bottom of the Displays panel
2. Select **Image** → OK
3. Set **Topic** to `/camera/image_raw`
4. Set **Reliability** to `Best Effort` (critical — default Reliable will drop frames)
5. **File → Save Config As** to [`config/tour.rviz`](../config/tour.rviz) so it loads next time

Or view outside RViz:

```bash
go2sh
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

> **Note:** the hyperspectral camera on the D1-T arm is a separate device from
> the Go2's front camera. Its ROS 2 / DDS interface will be documented in a
> dedicated doc once the physical mount is in place.

## Who else uses the camera

[`wifi_3d_nav.sh`](../scripts/runtime/wifi_3d_nav.sh) picks up
`/camera/image_raw` automatically ([3d_nav.md](3d_nav.md#camera-fusion-and-yolo)):

- **Map colour:** the LiDAR is projected into the image and RTAB-Map colours
  the 3D map.
- **Intrinsics:** [`go2_camera_info.py`](../scripts/nodes/go2_camera_info.py)
  publishes approximate intrinsics (`/camera/camera_info`), because the Go2
  camera has none.
- **With `--ai`:** YOLO11s publishes `/yolo/image` (boxes drawn) and the VLM
  sends frames to Ollama.
- **Web dashboard:** [`image_compressor.py`](../twin/image_compressor.py)
  makes 640 px JPEG copies (`/twin/camera/compressed`,
  `/twin/yolo/compressed`) at 5 Hz, only while a browser watches.

If the relay isn't publishing, the 3D session falls back to a LiDAR-only map,
and YOLO and the VLM stay off.

## See also

- [Troubleshooting → Camera](troubleshooting.md#camera) — empty Image display, QoS mismatch, stutter
- [tour.md](tour.md) — adding the Image display to the saved RViz layout ([`config/tour.rviz`](../config/tour.rviz))
- [3d_nav.md](3d_nav.md) — camera fusion, calibration notes, YOLO and VLM
- [roadmap.md](roadmap.md#milestone-3--camera-pipeline) — camera pipeline milestone
