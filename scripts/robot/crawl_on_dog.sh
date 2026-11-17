#!/usr/bin/env bash
# Run step_over_crawl.py ON THE DOG with unitree_sdk2py and the CycloneDDS build its
# Python binding was compiled against (docs/hardware/step_over_crawl.md, "Setup on the dog").
#   ~/go2-agri-lowlevel/crawl.sh --check
#   ~/go2-agri-lowlevel/crawl.sh --stand
#   ~/go2-agri-lowlevel/crawl.sh --lift FR
# The dog has two CycloneDDS 0.10.2 builds (/usr/local and ~/cyclonedds_ws/install) with
# different features.h; the binding is built against ~/cyclonedds_ws/install, so that
# library must be the one loaded (the other one crashed every write).
export LD_LIBRARY_PATH="$HOME/cyclonedds_ws/install/cyclonedds/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$HOME/go2-agri-lowlevel/sdk:${PYTHONPATH:-}"
exec python3 "$HOME/go2-agri-lowlevel/step_over_crawl.py" --iface "${GO2_LOWLEVEL_IFACE:-eth0}" "$@"
