# Sourced by run_trial.sh for the traversability mode.
export PYTHONPATH="$T:$PYTHONPATH"
python3 "$T/sim_box_detector.py" --ros-args -p world:="$T/cluttered_nav_test.sdf" > "$OUT/detector.log" 2>&1 &
python3 "$T/object_box_fusion.py" > "$OUT/fusion.log" 2>&1 &
python3 "$T/gait_supervisor.py" --ros-args -p dry_run:=true > "$OUT/gait.log" 2>&1 &
python3 "$T/save_frames.py" "$OUT/frames" > "$OUT/frames.log" 2>&1 &
