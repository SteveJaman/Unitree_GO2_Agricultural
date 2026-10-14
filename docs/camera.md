# Live Camera in RViz

The Go2's front camera publishes on `/frontvideostream` as
`unitree_go/Go2FrontVideoData` — a custom Unitree message type that RViz
cannot render. The relay node `scripts/nodes/go2_camera_relay.py` decodes
it and republishes as standard `sensor_msgs/Image` on `/camera/image_raw`.

The relay launches automatically when `wifi_nav.sh` runs (unless `--no-camera`).

## Verify the relay is publishing

```bash
go2sh
ros2 topic hz /camera/image_raw
# expect ~10–30 Hz