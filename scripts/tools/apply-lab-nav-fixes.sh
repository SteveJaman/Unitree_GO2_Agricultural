#!/usr/bin/env bash
# apply-lab-nav-fixes.sh — lab fixes for go2-nav2-wifi 3D navigation (--3d --plan / --nav)
#
#   1. Lidar mount fix: lidar3d_robot_viz.yaml sensor_from_base -> official Go2 URDF mount
#      (RViz dog was tilted ~20 deg and ~120 deg off in heading; Nav2 drove on that heading)
#   2. Smooth controller: DWB -> Regulated Pure Pursuit (behind the rotation shim)
#      + nav2 velocity_smoother (open loop, deadband) -> no more shuddering
#   3. Dockerfile.lidar3d installs the two extra Nav2 packages
#
# Usage, from the root of a fresh clone:
#   git clone https://github.com/dancher00/go2-nav2-wifi && cd go2-nav2-wifi
#   bash /path/to/apply-lab-nav-fixes.sh
# Safe to run twice. Undo: git checkout -- docker go2_nav2
set -euo pipefail

CONTAINER="${GO2_CONTAINER:-go2-lidar3d}"
PKGS="ros-humble-nav2-velocity-smoother ros-humble-nav2-regulated-pure-pursuit-controller"
say()  { printf '\033[1;32m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m!!\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31mERROR:\033[0m %s\n' "$*" >&2; exit 1; }

[[ -f mapping.sh && -f go2_nav2/launch/lidar3d_controller.launch.py ]] ||
  die "Run this from the root of a go2-nav2-wifi clone."
command -v git >/dev/null || die "git is required."

patch_file="$(mktemp)"; trap 'rm -f "$patch_file"' EXIT
cat >"$patch_file" <<'PATCH_EOF'
diff --git a/docker/Dockerfile.lidar3d b/docker/Dockerfile.lidar3d
index a17b43d..5bd378b 100644
--- a/docker/Dockerfile.lidar3d
+++ b/docker/Dockerfile.lidar3d
@@ -6,6 +6,7 @@ RUN apt-get update && apt-get install -y --no-install-recommends \
       git patch python3-dev python3-scipy python3-pytest libpcl-dev \
       ros-humble-pcl-ros ros-humble-tf2-eigen ros-humble-tf2-sensor-msgs \
       ros-humble-diagnostic-updater ros-humble-nav2-rotation-shim-controller \
+      ros-humble-nav2-velocity-smoother ros-humble-nav2-regulated-pure-pursuit-controller \
     && rm -rf /var/lib/apt/lists/*
 COPY ws/scripts/build-lidar3d.sh ws/scripts/pointlio-wifi.patch /opt/go2-lidar3d-build/
 RUN bash /opt/go2-lidar3d-build/build-lidar3d.sh /opt/go2-lidar3d-workspace /opt/go2-lidar3d \
diff --git a/go2_nav2/config/lidar3d_robot_viz.yaml b/go2_nav2/config/lidar3d_robot_viz.yaml
index f7906f3..bc17f29 100644
--- a/go2_nav2/config/lidar3d_robot_viz.yaml
+++ b/go2_nav2/config/lidar3d_robot_viz.yaml
@@ -8,4 +8,7 @@ go2_odom_tf:
     best_effort: true
     publish_odom: false
     smooth_alpha: 1.0
-    sensor_from_base: [-0.14056185097700238, 0.24211459251444103, -0.08014714132241024, 0.8713242364998213, -0.47301693680109264, 0.11500275477832939, 0.06183379966256489]
+    # Lab fix: author's one-off calibration (below) left the body tilted ~20 deg and ~120 deg off in yaw on our Go2.
+    # sensor_from_base: [-0.14056185097700238, 0.24211459251444103, -0.08014714132241024, 0.8713242364998213, -0.47301693680109264, 0.11500275477832939, 0.06183379966256489]
+    # Official Go2 URDF radar mount (body_from_radar rpy 0, 2.8782, 0), inverted to sensor_from_base:
+    sensor_from_base: [0.267276, 0.0, -0.120571, 0.0, -0.991341, 0.0, 0.131316]
diff --git a/go2_nav2/launch/lidar3d_controller.launch.py b/go2_nav2/launch/lidar3d_controller.launch.py
index 341e22a..ac485ea 100644
--- a/go2_nav2/launch/lidar3d_controller.launch.py
+++ b/go2_nav2/launch/lidar3d_controller.launch.py
@@ -24,24 +24,29 @@ def configure(context):
                   min_theta_velocity_threshold=0.02)
     params['progress_checker']['movement_time_allowance'] = 35.0
     params['general_goal_checker']['xy_goal_tolerance'] = 0.25
-    params['FollowPath'].update(min_vel_x=0.0, max_vel_x=0.30, min_vel_y=0.0,
-        max_vel_y=0.0, max_vel_theta=0.70, max_speed_xy=0.30,
-        acc_lim_x=0.6, decel_lim_x=-0.6, acc_lim_y=0.3, decel_lim_y=-0.3,
-        acc_lim_theta=1.0, decel_lim_theta=-1.0, vy_samples=1,
-        transform_tolerance=0.3, xy_goal_tolerance=0.25,
-        critics=['Oscillation', 'ObstacleFootprint', 'PathAlign', 'GoalAlign', 'PathDist', 'GoalDist'])
-    params['FollowPath'].update(
-        plugin='nav2_rotation_shim_controller::RotationShimController',
-        primary_controller='dwb_core::DWBLocalPlanner',
-        angular_dist_threshold=0.5, angular_disengage_threshold=0.25,
-        forward_sampling_distance=0.5, rotate_to_heading_angular_vel=0.70,
-        max_angular_accel=1.0, simulate_ahead_time=2.0,
-        rotate_to_goal_heading=False, closed_loop=False)
-    # Heading critics from the community Go2 DWB profile: turning toward a path
-    # must improve its score even when the body centre has not translated yet.
-    params['FollowPath'].update({'PathAlign.scale': 32.0,
-        'PathAlign.forward_point_distance': 0.1,
-        'GoalAlign.scale': 24.0, 'GoalAlign.forward_point_distance': 0.1})
+    # Regulated Pure Pursuit behind the rotation shim. RPP follows the path
+    # geometrically and does not build its command window from measured odom
+    # velocity, so noisy Point-LIO-derived twist cannot make it jitter like DWB.
+    params['FollowPath'] = {
+        'plugin': 'nav2_rotation_shim_controller::RotationShimController',
+        'primary_controller': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
+        # Wider hysteresis: rotate in place only for large errors, hand back early.
+        'angular_dist_threshold': 0.8, 'angular_disengage_threshold': 0.2,
+        'forward_sampling_distance': 0.5, 'rotate_to_heading_angular_vel': 0.6,
+        'max_angular_accel': 1.0, 'simulate_ahead_time': 1.0,
+        'rotate_to_goal_heading': False, 'closed_loop': False,
+        'desired_linear_vel': 0.30,
+        'lookahead_dist': 0.6, 'min_lookahead_dist': 0.4, 'max_lookahead_dist': 0.9,
+        'lookahead_time': 1.5, 'use_velocity_scaled_lookahead_dist': False,
+        'transform_tolerance': 0.3,
+        'min_approach_linear_velocity': 0.08, 'approach_velocity_scaling_dist': 0.6,
+        'use_collision_detection': True, 'max_allowed_time_to_collision_up_to_carrot': 1.0,
+        'use_regulated_linear_velocity_scaling': True,
+        'use_cost_regulated_linear_velocity_scaling': False,
+        'regulated_linear_scaling_min_radius': 0.6, 'regulated_linear_scaling_min_speed': 0.12,
+        'use_rotate_to_heading': False,  # the shim handles rotation
+        'allow_reversing': False, 'max_robot_pose_search_dist': 3.0,
+    }
     with open(pkg + '/config/lidar3d_planner.yaml') as stream:
         local = copy.deepcopy(yaml.safe_load(stream)['global_costmap']['global_costmap'])
     cost = local['ros__parameters']
@@ -69,11 +74,23 @@ def configure(context):
     return [
         RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
         Node(package='go2_nav2', executable='go2_odom_tf', name='lidar3d_body_odom', parameters=[odom]),
+        # Controller output goes through the smoother; only the smoother publishes /cmd_vel.
         Node(package='nav2_controller', executable='controller_server', name='controller_server',
-             output='screen', parameters=[path]),
+             output='screen', parameters=[path], remappings=[('cmd_vel', 'cmd_vel_nav')]),
+        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
+             output='screen',
+             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
+             parameters=[{
+                 'smoothing_frequency': 20.0, 'scale_velocities': True,
+                 'feedback': 'OPEN_LOOP',  # never trust the noisy measured twist
+                 'max_velocity': [0.30, 0.0, 0.70], 'min_velocity': [-0.30, 0.0, -0.70],
+                 'max_accel': [0.5, 0.0, 0.8], 'max_decel': [-0.5, 0.0, -0.8],
+                 # Go2 steps in place on tiny commands; zero them instead.
+                 'deadband_velocity': [0.03, 0.0, 0.08],
+                 'velocity_timeout': 1.0}]),
         Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
              name='lifecycle_manager_lidar3d_controller', parameters=[{
-                 'autostart': True, 'node_names': ['controller_server'], 'bond_timeout': 4.0}]),
+                 'autostart': True, 'node_names': ['controller_server', 'velocity_smoother'], 'bond_timeout': 4.0}]),
         Node(package='go2_nav2', executable='go2_goal_pose_nav', name='go2_goal_pose_nav',
              output='screen', parameters=[{'map_frame': 'lidar3d_map', 'pre_rotate_enable': False}]),
     ]
PATCH_EOF

# 1-3. Source changes
if git apply --reverse --check "$patch_file" 2>/dev/null; then
  say "Fixes already applied — skipping patch."
elif git apply --check "$patch_file" 2>/dev/null; then
  git apply "$patch_file"
  say "Patched: lidar mount, RPP controller + velocity smoother, Dockerfile.lidar3d."
else
  die "Patch does not apply cleanly. The upstream repo probably changed since this script was written (tested on commit 4a8a8b9). Ask the person who shared this script."
fi

# 4. Container: install packages + rebuild, if the 3D container already exists
if ! command -v docker >/dev/null; then
  warn "Docker not found. Install Docker, then follow the repo's 3D setup (docs/3D-QUICKSTART.md)."
elif ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
  warn "Container '$CONTAINER' does not exist yet."
  warn "Run the repo's 3D setup now (./setup-lidar3d.sh, see docs/3D-QUICKSTART.md)."
  warn "The patched Dockerfile.lidar3d already includes the new Nav2 packages."
else
  say "Starting $CONTAINER and installing Nav2 plugins (needs internet)..."
  docker start "$CONTAINER" >/dev/null
  docker exec -u root "$CONTAINER" bash -c "dpkg -s $PKGS >/dev/null 2>&1 || (apt-get update -qq && apt-get install -y -qq $PKGS)"
  say "Rebuilding go2_nav2 inside the container..."
  docker exec "$CONTAINER" bash -c 'source /opt/ros/humble/setup.bash && cd /ws && colcon build --symlink-install --base-paths src/go2_nav2 src/go2_description --packages-select go2_nav2 go2_description'
fi

# 5. Convenience shell alias
alias_line="alias go2sh='docker exec -it $CONTAINER bash -c \"source /ws/scripts/go2-env.sh; exec bash -i\"'"
if ! grep -qs "alias go2sh=" "$HOME/.bashrc"; then
  echo "$alias_line" >> "$HOME/.bashrc"
  say "Added 'go2sh' alias to ~/.bashrc (open a new terminal to use it)."
fi

cat <<'DONE'

Done. Next:
  1. Stand the dog up FIRST (the map frame is fixed from the floor at startup).
  2. ./mapping.sh --3d --plan   -> check RViz model matches the real dog (no motion)
  3. ./mapping.sh --3d --nav    -> 2D Goal Pose 1-2 m ahead; keep the handheld remote as e-stop
See LAB-NAV-FIXES.md (shared with this script) for checks, teleop and troubleshooting.
DONE
