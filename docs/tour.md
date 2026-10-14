# Visual Waypoint Setup (RViz)

One-time setup to replace 2D Goal Pose with a waypoint queue and
visual route planning.

## Why

The default RViz setup only has "2D Goal Pose" — one goal at a time,
no queue, no visible route. The Nav2 RViz panel adds:

- Numbered waypoint markers that persist on the map
- A waypoint queue you build by clicking
- "Start Waypoint Following" button to execute the route
- Live path lines from the robot to the next waypoint

## Setup (once per workstation)

Launch a session as usual:

    cd ~/files/go2-nav2-wifi
    ./mapping.sh --3d --nav

Wait for RViz to open with a live map.

### 1. Add the Nav2 panel

In RViz:

1. Menu: **Panels** → **Add New Panel**
2. Select **nav2_rviz_plugins/Navigation2**
3. Click **OK**

The panel appears docked at the bottom-left. It has a
**"Waypoint Mode"** checkbox at the top.

### 2. Add the global path display

1. In the Displays panel (left side), click **Add**
2. Find **Path** in the list, click OK
3. Expand the new Path display, set:
   - **Topic**: `/plan`
   - **Color**: yellow (or anything visible)
   - **Line Style**: Lines

### 3. Add the local path display

Same as above, but:
- **Topic**: `/local_plan`
- **Color**: cyan

The global path is what the planner intends. The local path is
what the controller is actually following right now. Both visible
is useful — when they diverge, the controller is avoiding an
obstacle.

### 4. Add the costmap display (optional)

Same as above, but:
- Add **Map**
- **Topic**: `/global_costmap/costmap`
- **Color Scheme**: costmap

This shows the inflated obstacle zones. Useful for understanding
why the planner avoids certain areas.

### 5. Save the config

Menu: **File** → **Save Config As**

Save to:

    ~/files/Unitree_GO2_Agricultural/config/tour.rviz

Then commit:

    cd ~/files/Unitree_GO2_Agricultural
    git add config/tour.rviz
    git commit -m "Add RViz config with waypoint mode and path displays"
    git push origin main

## Using it every session

Once saved, load the config with RViz's `-d` flag:

    rviz2 -d ~/files/Unitree_GO2_Agricultural/config/tour.rviz

`mapping.sh` currently launches RViz with its own config. To use
yours instead, either:

- **Manually**: close RViz, relaunch with `-d`
- **Permanently**: edit `mapping.sh` to pass `-d` (see below)

## Workflow with waypoints

1. In the Nav2 panel, check **Waypoint Mode**
2. Select the **Nav2 Goal** tool in the toolbar
3. Click on the map — a numbered marker appears
4. Click more points — they queue in order
5. Watch the yellow path line draw to waypoint 1
6. Press **Start Waypoint Following** in the Nav2 panel
7. The dog navigates each waypoint in order
8. Path line updates as each one is reached

To remove a waypoint: select it and press **Delete**.
To clear the queue: press the **Reset** button in the panel.