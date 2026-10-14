#!/usr/bin/env bash
# save_rviz_config.sh — interactive guide to save the RViz layout.
# Run this while a wifi_nav.sh session is up.
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TARGET="${REPO_ROOT}/config/tour.rviz"

cat <<EOF
===============================================================
  Save RViz layout to config/tour.rviz
===============================================================

In the RViz window that's already open:

  1. Add the Nav2 waypoint panel
     Panels -> Add New Panel -> nav2_rviz_plugins/Navigation2

  2. Add the live camera
     Add -> By topic -> /camera/image_raw -> Image
     Set Reliability = Best Effort

  3. Add the global path
     Add -> By topic -> /plan -> Path
     Set Color = yellow

  4. Add the local path
     Add -> By topic -> /local_plan -> Path
     Set Color = cyan

  5. Add the costmap
     Add -> By topic -> /global_costmap/costmap -> Map
     Set Color Scheme = costmap

  6. Save:
     File -> Save Config As
     Path: ${TARGET}

Then commit:

  cd ${REPO_ROOT}
  git add config/tour.rviz
  git commit -m "Add tour RViz config"
  git push origin main

EOF