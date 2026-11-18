# Height drift on the real dog (the dog sinking in RViz)

## Why it sinks

Point-LIO on the dog runs gyro-only (`go2-nav2-wifi/go2_nav2/config/pointlio_go2.yaml`):
`use_acceleration: false`, `gravity_align: false`, `gravity: [0, 0, 0]`. There is no
gravity in the filter at all, so nothing holds its z or its tilt. A small pitch error
turns walking forward into walking downhill, and the height sinks a few cm per metre.

Tuning the filter's covariances can't fix that: with `lidar_meas_cov` raised from
0.01 to 0.1, the filter trusts the scan match less and the gyro-only prediction more,
which has no gravity in it. Point-LIO has no `fix_gravity_direction` parameter, and the
extrinsics are already the nominal L1 mount.

## The fix: levelling after Point-LIO

`scripts/nodes/go2_lidar3d_level.py` corrects Point-LIO's output on the laptop:

- tilt held to the body IMU's gravity (`/lf/lowstate`);
- height held to the floor (`GO2_LIDAR3D_HOLD_HEIGHT=0` for ramps and stairs).

The dog's relay publishes Point-LIO as `/lidar3d/raw/*`, and the node republishes the
corrected `/lidar3d/odom`, `/lidar3d/registered` and `/lidar3d/path`. Nav2, the costmap,
RViz and the web app (through TF) all use the corrected ones. `wifi_3d_nav.sh` turns it
on for the real dog (`GO2_LIDAR3D_LEVEL=0` turns it off).

Until 2026-11-18 the node got no data: `GO2_LIDAR3D_RAW` never reached the SLAM
container on the dog, so the drift went straight to RViz. `scripts/tools/relay_lidar3d_raw.py`
now passes it into that container (`mapping.sh`'s `docker exec`).

## Result (2026-11-18, about 4 m of walking)

| Time | Raw z (Point-LIO) | Levelled z |
|---|---|---|
| start | 0.289 m | 0.299 m |
| after 2 m | 0.255 m | 0.299 m |
| after the second goal | 0.17–0.19 m | 0.299 m |

The body in TF read roll −1.4°, pitch 0.7° at the end.

## Auto explore (2026-11-18, about 2.5 min)

The dog stood for 105 s, then explored about 4 m. Point-LIO's raw height fell 1.2 cm while
standing and then 2.17 m in the 45 s of exploring. The levelled height did not move, and
the body in TF stayed within 3.5 cm. So the levelling holds, but a drift that fast means
Point-LIO's own pitch is badly off when the dog moves quickly, which also distorts x and y.

## Walk test

1. Start the session and keep the dog still until "3D twin READY":

   ```bash
   ./scripts/runtime/wifi_3d_nav.sh --manual
   ```

2. In a second terminal, watch the height:

   ```bash
   docker cp scripts/tools/z_drift_monitor.py go2-lidar3d:/tmp/
   docker exec -it go2-lidar3d bash -c "source /opt/ros/humble/setup.bash && python3 /tmp/z_drift_monitor.py"
   ```

3. Walk the dog 3–5 m with the remote and back.

Pass: `level z` and `base z` stay within about 1 cm of where they started, while `raw z`
may wander. If `raw z: no data`, the levelling isn't running: check that the session
was started by `wifi_3d_nav.sh` and that `GO2_LIDAR3D_LEVEL` isn't set to 0.
