# RViz Waypoint Mode and Tour Setup

[← Back to README](../README.md) · [Roadmap](roadmap.md) · [Camera](camera.md) · [Troubleshooting](troubleshooting.md)

Setup guide to replace the single-goal "2D Goal Pose" tool with a queued
waypoint system that draws a visible route on the map.

The Nav2 RViz panel is already installed in the container. This guide
wires it up in the RViz layout and saves it to
[`config/tour.rviz`](../config/tour.rviz) so it loads automatically in
future sessions.

---

## What this gives you

- Numbered waypoint markers that persist on the map
- A waypoint queue you build by clicking
- **Start Waypoint Following** button to execute the route
- Live path lines from the robot to the next waypoint
- Optional live camera feed and costmap display in the same layout
- A single `config/tour.rviz` file the whole team can share

---

## Prerequisites

- A running session: [`./scripts/runtime/wifi_nav.sh`](../scripts/runtime/wifi_nav.sh)
- RViz window open with a live map and a valid TF tree
  (`lidar3d_map → base_link` resolving)
- The `nav2_rviz_plugins` package installed (it is, by default, in the
  `go2-lidar3d` container)

Verify RViz is showing the map before continuing:

```bash
go2sh
ros2 topic hz /global_costmap/costmap      # expect ~1 Hz
ros2 run tf2_ros tf2_echo lidar3d_map base_link
```

---

## Setup (one-time, in a running RViz)

### 1. Add the Nav2 panel

1. Menu: **Panels → Add New Panel**
2. Select **nav2_rviz_plugins/Navigation 2**
3. Click OK

The panel appears docked bottom-left. It contains a **Waypoint Mode**
checkbox at the top, a **Start Waypoint Following** button, and a
**Reset** button.

### 2. Add the global path

1. In the Displays panel, click **Add**
2. Select **Path** → OK
3. Expand the new Path display and set:
   - **Topic**: `/plan`
   - **Color**: yellow (`255; 255; 0`)
   - **Line Style**: Lines
   - **Line Width**: 0.03

The global path is what the planner intends. It updates each time a new
waypoint becomes the active goal.

### 3. Add the local path

Same as above but:
- **Topic**: `/local_plan`
- **Color**: cyan (`0; 255; 255`)

The local path is what the controller is following *right now*. When the
global and local paths diverge, the controller is avoiding an obstacle.

### 4. Add the costmap (optional)

1. Add **Map** → OK
2. **Topic**: `/global_costmap/costmap`
3. **Color Scheme**: `costmap`

Shows the inflated obstacle zones. Useful for understanding why the
planner routes the way it does.

### 5. Add the live camera (optional)

1. Add **Image** → OK
2. **Topic**: `/camera/image_raw`
3. **Reliability Policy**: `Best Effort`

The `Best Effort` setting is required. The relay publishes with
`BEST_EFFORT` QoS; RViz defaults to `RELIABLE`. The mismatch silently
drops every frame.

See [camera.md](camera.md) for the full camera setup.

### 6. Save the config

**File → Save Config As**

Save to:

```
~/files/Unitree_GO2_Agricultural/config/tour.rviz
```

Then commit:

```bash
cd ~/files/Unitree_GO2_Agricultural
git add config/tour.rviz
git commit -m "Add RViz layout with waypoint panel and path displays"
git push origin main
```

---

## Using waypoint mode

### Building a route

1. In the Nav2 panel, check **Waypoint Mode**
2. Select the **Nav2 Goal** tool in the RViz toolbar
3. Click on the map where you want the first waypoint
   - A numbered marker appears (1)
   - The yellow path line draws from the robot to waypoint 1
4. Click a second point
   - Marker 2 appears
   - The path line extends to it
5. Continue adding waypoints in order

### Executing the route

Press **Start Waypoint Following** in the Nav2 panel.

The robot navigates to waypoint 1, then 2, then 3, in order. Each marker
turns green as it is reached. When all waypoints are reached, the panel
returns to idle.

### Editing the route

| Action | How |
|---|---|
| Remove one waypoint | Select it in the panel list, press **Delete** |
| Clear the queue | Press **Reset** in the Nav2 panel |
| Reorder | Not supported in the standard panel — clear and rebuild |
| Pause | Press **Stop** or toggle off Waypoint Mode |

### Emergency stop

The waypoint panel doesn't have a stop button that halts the robot
immediately. Options:

- **Physical remote** — the safest, always works
- **Ctrl+C** in the terminal running `wifi_nav.sh` — stops everything
- **STOP button** in the Go2 Explore panel
  ([`src/go2_explore_panel/`](../src/go2_explore_panel/)) — publishes `Bool(false)`
  to `/explore/resume`, which stops the explorer but not an in-flight
  waypoint route

If you need a mid-route stop, the remote is the reliable choice.

---

## Saving and sharing the layout

Once `config/tour.rviz` is committed, every teammate gets the same layout
on a fresh clone. Launch RViz with it explicitly:

```bash
go2sh
rviz2 -d ~/files/Unitree_GO2_Agricultural/config/tour.rviz
```

Or modify `wifi_nav.sh` to pass `-d` when starting RViz (see the
`mapping.sh` wrapper, which handles RViz launch internally).

The layout is part of the repo now. If someone edits the panel arrangement
in RViz, they should **File → Save Config As** to the same path and commit
the diff.

---

## Helper script

[`scripts/tools/save_rviz_config.sh`](../scripts/tools/save_rviz_config.sh) prints these exact steps in the
terminal while RViz is open. Useful as a reminder if you forget the order:

```bash
./scripts/tools/save_rviz_config.sh
```

The script doesn't modify anything — it's a checklist you run alongside
RViz.

---

## Combining with the explorer

Waypoint mode and the frontier explorer
([`frontier_explorer.py`](../scripts/nodes/frontier_explorer.py)) are independent:

- **Waypoint mode** — you click, the robot goes to each point in order
- **Explorer** — the robot picks its own frontiers and sends itself goals

Both publish to `/goal_pose`. **Do not run both at once.** The explorer
will fight your waypoints, and vice versa. If you want to switch:

1. Stop waypoint following (or clear the queue)
2. Disable the explorer: publish `Bool(false)` to `/explore/resume`
3. Then start the other

If you want both available in the same session, the cleanest pattern is:
- Use waypoint mode for a specific route (e.g. a tour capture pass)
- Use the explorer for open-ended mapping

Pick one per session.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Nav2 panel not in "Add New Panel" list | `nav2_rviz_plugins` not installed | `docker exec -u root go2-lidar3d apt-get install -y ros-humble-nav2-rviz-plugins` |
| Clicking "Nav2 Goal" adds a goal, not a waypoint | Waypoint Mode unchecked | Check the box at the top of the Nav2 panel |
| Path line doesn't appear | Path display missing or wrong topic | Add Path on `/plan`; verify `/plan` publishes while a goal is active |
| Waypoints don't persist on the map | Waypoint panel closed or reset | Re-add the panel; check **File → Save Config As** was done |
| Robot doesn't start following | Panel didn't receive the click, or the BT navigator is inactive | Check `ros2 lifecycle get /bt_navigator` returns `active` |
| All waypoints turn red at once | Planner rejected them (goal inside an inflated obstacle) | Move waypoints away from walls; run [`./scripts/tools/tune_costmap.sh --safe`](../scripts/tools/tune_costmap.sh) |
| Saved `tour.rviz` doesn't load | Version mismatch or a plugin missing | Open RViz without `-d`, redo the setup, save again |

More: [troubleshooting.md](troubleshooting.md#rviz--vm).

---

## What's next after waypoints work

Waypoint mode is [Milestone 1](roadmap.md#milestone-1--visual-waypoint-system-rviz) in the roadmap. Once a 3-waypoint
route executes cleanly, the next steps are:

1. **[Milestone 2](roadmap.md#milestone-2--explorer-verification)** — verify the explorer produces goals on a live map
2. **[Milestone 3](roadmap.md#milestone-3--camera-pipeline)** — camera backend fix on the Jetson
3. **[Milestone 4](roadmap.md#milestone-4--rtab-map-reconstruction)** — RTAB-Map reconstruction (dense 3D tour)
4. **[Milestone 5](roadmap.md#milestone-5--capture-automation)** — spin-on-goal capture (`-p spin_on_goal:=true`)
5. **[Milestone 6](roadmap.md#milestone-6--end-to-end-autonomous-capture)** — one-command end-to-end tour capture

See [roadmap.md](roadmap.md) for the full list and status.