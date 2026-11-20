# Video index (Phase 6)

[← README](../../README.md) · [Docs index](../README.md) · [Orbit results](results_orbit.md)

The videos are not in git (too large). They are on the lab workstation under
`~/files/go2-nav2-wifi/ws/log/orbit_video/`, made by:

- `scripts/runtime/record_orbit_videos.sh`: V1 and V3, screen captures of the simulation desktop
  (`scripts/tools/desktop_capture.py`: the four windows tiled 2x2, then ffmpeg's X11 grab, H.264).
  Run it with `--early-k "2 9 14"` for V3c and `--only "v1 v3a v3b v3c"` to pick clips.
- `scripts/tools/wrist_video.py`: V2, from the wrist frames logged by the Phase 3 orbits, with each
  species' own detector. These are the 5 frames per waypoint that the coverage numbers use, held
  0.8 s each, not a continuous stream.

| Video | Path (under `~/files/go2-nav2-wifi/ws/log/orbit_video/`) | Duration | What it shows |
|---|---|---|---|
| V1 | `showcase/showcase.mp4` | 200 s | Full servo trial: Nav2 drive to the rack, tail-first park, distance servo to 0.50 m. 2x2 desktop: Gazebo (camera following the dog), Nav2 RViz, MoveIt RViz, camera RViz (plant detector). |
| V1 | `showcase/composed_demo.mp4` | 92 s | The same trial composed by record_demo_video.py: overview/side view, wrist camera with the tracker's YOLO box, live distance. |
| V3a | `v3_lettuce_full/v3_lettuce_full.mp4` | 476 s | Full 3-shell orbit of the control lettuce at (0.40, 0.25), all feasible waypoints, desktop capture. |
| V3c | `v3_lettuce_early_stop/v3_lettuce_early_stop.mp4` | 52 s | Early-stop orbit of the same lettuce: only the waypoints the policy keeps (k = 2, 9, 14: one per shell), executed physically (orbit_waypoints.py --only-k). |
| V3b | `v3_strawberry_under/v3_strawberry_under.mp4` | 201 s | Under shell of a strawberry plant (strawberry_0_3, world cropking_nft46_strawberry). The camera RViz runs the lettuce detector; the strawberry boxes are in V2. |
| V2 | `wrist/wrist_lettuce_eq.mp4` | 12 s | Wrist camera, lettuce, eq shell: the logged frames (5 per waypoint, after settling) with the lettuce detector's boxes (conf >= 0.5). |
| V2 | `wrist/wrist_lettuce_top.mp4` | 4 s | Wrist camera, lettuce, top shell: the logged frames (5 per waypoint, after settling) with the lettuce detector's boxes (conf >= 0.5). |
| V2 | `wrist/wrist_lettuce_under.mp4` | 8 s | Wrist camera, lettuce, under shell: the logged frames (5 per waypoint, after settling) with the lettuce detector's boxes (conf >= 0.5). |
| V2 | `wrist/wrist_basil_eq.mp4` | 12 s | Wrist camera, basil, eq shell: the logged frames (5 per waypoint, after settling) with the basil detector's boxes (conf >= 0.5). |
| V2 | `wrist/wrist_basil_top.mp4` | not made: shell not reached | Wrist camera, basil, top shell: the logged frames (5 per waypoint, after settling) with the basil detector's boxes (conf >= 0.5). |
| V2 | `wrist/wrist_basil_under.mp4` | 20 s | Wrist camera, basil, under shell: the logged frames (5 per waypoint, after settling) with the basil detector's boxes (conf >= 0.5). |
| V2 | `wrist/wrist_spinach_eq.mp4` | 12 s | Wrist camera, spinach, eq shell: the logged frames (5 per waypoint, after settling) with the spinach detector's boxes (conf >= 0.5). |
| V2 | `wrist/wrist_spinach_top.mp4` | 4 s | Wrist camera, spinach, top shell: the logged frames (5 per waypoint, after settling) with the spinach detector's boxes (conf >= 0.5). |
| V2 | `wrist/wrist_spinach_under.mp4` | 16 s | Wrist camera, spinach, under shell: the logged frames (5 per waypoint, after settling) with the spinach detector's boxes (conf >= 0.5). |
| V2 | `wrist/wrist_tomato_eq.mp4` | not made: shell not reached | Wrist camera, tomato, eq shell: the logged frames (5 per waypoint, after settling) with the tomato detector's boxes (conf >= 0.5). |
| V2 | `wrist/wrist_tomato_top.mp4` | not made: shell not reached | Wrist camera, tomato, top shell: the logged frames (5 per waypoint, after settling) with the tomato detector's boxes (conf >= 0.5). |
| V2 | `wrist/wrist_tomato_under.mp4` | not made: shell not reached | Wrist camera, tomato, under shell: the logged frames (5 per waypoint, after settling) with the tomato detector's boxes (conf >= 0.5). |
| V2 | `wrist/wrist_strawberry_eq.mp4` | 12 s | Wrist camera, strawberry, eq shell: the logged frames (5 per waypoint, after settling) with the strawberry detector's boxes (conf >= 0.5). |
| V2 | `wrist/wrist_strawberry_top.mp4` | 4 s | Wrist camera, strawberry, top shell: the logged frames (5 per waypoint, after settling) with the strawberry detector's boxes (conf >= 0.5). |
| V2 | `wrist/wrist_strawberry_under.mp4` | 16 s | Wrist camera, strawberry, under shell: the logged frames (5 per waypoint, after settling) with the strawberry detector's boxes (conf >= 0.5). |

## Notes

- V1 was trimmed to the first 200 s: the trial ran from 14:23:35 to 14:25:25. The recorder did not
  stop at the end of the trial: ffmpeg ignored its stop key in the background. That is fixed
  (`desktop_capture.py` signals ffmpeg).
- In every desktop capture, the camera RViz (bottom right) is partly covered by the MoveIt RViz,
  which cannot shrink below its minimum size. The MoveIt window also shows a harmless
  "MotionPlanningFrame could not be loaded" message from a duplicate panel.
- V3c executes the waypoints the early-stop policy keeps when it is replayed on the Phase 2 orbits
  of this plant (k = 2, 9, 14). Those orbits are full ones; the policy was not run online.
- There are no V2 clips for basil top or for tomato: the arm reached none of those waypoints.
