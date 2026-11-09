# Sourced by run_trial.sh: traversability with floor_estimation: histogram in HeightBandLayer
# and the fusion (flat-floor sanity check against fixed). The controller is
# controller_traversability.yaml plus floor_estimation, written by run_all.sh.
FUSION_ARGS="-p floor_estimation:=histogram"
source "$T/start_extra_traversability.sh"
