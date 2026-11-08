# Sourced by run_trial.sh for traversability_semantic: the traversability stack
# plus the VLM labeller (feeding SemanticCostmapLayer in the local costmap) and
# its recorder. Ollama is on the Windows host, the WSL default gateway.
GAIT_ARGS="-p use_semantic:=true"
source "$T/start_extra_traversability.sh"
GW=$(ip route | awk '/default/{print $3; exit}')
bg python3 "$T/vlm_heuristic_node.py" --ros-args --params-file "$T/ollama.yaml" \
    -p ollama_url:="http://$GW:11434" -p dry_run:=false -p backend:=gazebo \
    > "$OUT/vlm.log" 2>&1
bg python3 "$T/record_semantic.py" "$OUT" > "$OUT/record_semantic.log" 2>&1
bg python3 "$T/save_costmap.py" "$OUT" 5 > "$OUT/save_costmap.log" 2>&1
