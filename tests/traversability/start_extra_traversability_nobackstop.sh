# Sourced by run_trial.sh: traversability with the gait backstop off (Phase 3 comparison).
# The controller is controller_traversability.yaml (run_all.sh copies it under this mode name).
GAIT_ARGS="-p backstop_enabled:=false"
source "$T/start_extra_traversability.sh"
