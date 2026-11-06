# Sourced by run_trial.sh for the traversability mode. Real YOLO runs in its own
# container (run_all.sh); the stand-in reports only classes COCO does not have.
export PYTHONPATH="$T:$PYTHONPATH"
python3 "$T/sim_box_detector.py" --ros-args -p world:="$T/cluttered_items.json" \
    -p only_classes:="power_strip,plant_saucer,cable,plant_pot,shoe,toolbox,trash_bin,side_table,bookshelf,file_cabinet" \
    > "$OUT/detector.log" 2>&1 &
python3 "$T/object_box_fusion.py" > "$OUT/fusion.log" 2>&1 &
python3 "$T/gait_supervisor.py" --ros-args -p dry_run:=true > "$OUT/gait.log" 2>&1 &
python3 "$T/save_frames.py" "$OUT/frames" > "$OUT/frames.log" 2>&1 &
