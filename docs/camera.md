# Live Camera

[← Back to README](../README.md) · [Troubleshooting](troubleshooting.md#camera) · [Camera + LiDAR fusion](camera_lidar_fusion.md)

The Go2's front camera reaches the laptop as colour `sensor_msgs/Image` on
`/camera/image_raw`, in two hops:

```
Jetson (robot)                                     laptop (go2-lidar3d)
go2_camera_jpeg_cli ─► go2_camera_wifi.py ──Wi-Fi──► go2_camera_relay.py ─► /camera/image_raw  (bgr8, reliable)
 1920x1080 JPEG ~16 fps   /go2/front_camera/compressed   decode JPEG        /camera/camera_info (if calibrated)
                          960x540 JPEG, 5 fps, ~66 KB
```

1. **On the robot:** [`scripts/robot/go2_camera_wifi.sh`](../scripts/robot/go2_camera_wifi.sh)
   reads the camera's own JPEG stream (from `~/bin/go2_camera_jpeg_cli`, built
   by go2-nav2-wifi's `robot-build-camera-cli.sh`), downscales it and
   publishes `sensor_msgs/CompressedImage` on the Wi-Fi DDS domain (64). About
   0.3 MB/s, stamped with the Jetson clock like the LiDAR.
2. **On the laptop:** [`go2_camera_relay.py`](../scripts/nodes/go2_camera_relay.py)
   decodes it to `/camera/image_raw` and, once the camera is calibrated, also
   publishes `/camera/camera_info`.

`wifi_nav.sh` (and so `wifi_3d_nav.sh`) does both automatically unless you pass
`--no-camera`. It copies the robot files to `~/go2-agri-camera/` on the robot
on every start. This is needed because in 3D mode go2-nav2-wifi's `mapping.sh`
starts its robot relay with the camera off (`GO2_RELAY_CAMERA=0`).

| Setting (environment) | Default | Meaning |
|---|---|---|
| `CAMERA_WIDTH` | `960` | Width sent over Wi-Fi (1920 = native, no re-encoding). A calibration is only valid at the width it was taken at |
| `CAMERA_FPS` | `5` | Frames per second sent |

In simulation (`--sim`), Gazebo publishes `/camera/image_raw` directly, so
neither hop runs.

## Viewing it

RViz: the session layouts have a **Front camera** display (added by
`wifi_nav.sh`; see [3d_nav.md → In RViz](3d_nav.md#in-rviz)). If you add one
yourself: Add → By topic → `/camera/image_raw` → Image, **Reliability
Policy: Reliable**.

Outside RViz:

```bash
go2sh
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

## Verify it

```bash
ssh unitree@<robot-ip> tail -3 /tmp/go2_camera_wifi.log     # "50 frames in 10 s, 66 KB/frame"
docker exec go2-lidar3d tail -3 /tmp/camera_relay.log        # "published 100 frames"
```

## Calibration and LiDAR overlay

To use the camera together with the LiDAR, calibrate it once
(`./scripts/tools/calibrate_camera.sh`) so the relay publishes
`/camera/camera_info`, then check the alignment with
`./scripts/tools/lidar_overlay.sh`. See [camera_lidar_fusion.md](camera_lidar_fusion.md).

## See also

- [Troubleshooting → Camera](troubleshooting.md#camera): no image, stutter
- [roadmap.md](roadmap.md#milestone-3--camera-pipeline): camera pipeline milestone
