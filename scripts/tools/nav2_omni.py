#!/usr/bin/env python3
"""
nav2_omni.py: switch go2-nav2-wifi between forward-only and omnidirectional Nav2.

  nav2_omni.py --enable  NAV2_WIFI_DIR   # config/nav2/go2_omni.yaml + launch hooks
  nav2_omni.py --disable NAV2_WIFI_DIR   # remove both (no-op if absent)

--enable copies config/nav2/go2_omni.yaml to go2_nav2/config/go2_omni.yaml and
adds small hooks (every line tagged, see TAG) to:
  go2_nav2/launch/lidar3d_planning.launch.py    planner: omni lattice primitives
  go2_nav2/launch/lidar3d_controller.launch.py  controller: MPPI Omni instead of
                                                RPP + rotation shim; smoother vy
The hooks read /ws/src/go2_nav2/config/go2_omni.yaml (the bind-mounted source
tree), so editing that file and restarting the session is enough to retune.
--disable restores both launch files byte-for-byte and deletes the copy.
Run by wifi_nav.sh: --omni enables, every other session disables.
"""

import argparse
import os
import shutil
import sys

TAG = "  # go2-omni"
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SRC_CFG = os.path.join(REPO, "config", "nav2", "go2_omni.yaml")
CFG_IN_CONTAINER = "/ws/src/go2_nav2/config/go2_omni.yaml"

PLANNING = "go2_nav2/launch/lidar3d_planning.launch.py"
PLANNING_ANCHOR = "    data['planner_server'] = planner"
PLANNING_HOOK = f"""\
    _omni = '{CFG_IN_CONTAINER}'
    if os.path.exists(_omni) and 'lattice_filepath' in planner['ros__parameters']['GridBased']:
        with open(_omni) as stream:
            planner['ros__parameters']['GridBased'].update(yaml.safe_load(stream).get('planner', {{}}))
"""

CONTROLLER = "go2_nav2/launch/lidar3d_controller.launch.py"
CONTROLLER_ANCHOR = "    fd, path = tempfile.mkstemp(prefix='go2-lidar3d-controller-', suffix='.yaml')"
CONTROLLER_HOOK = f"""\
    _omni_smoother = {{}}
    _omni = '{CFG_IN_CONTAINER}'
    if os.path.exists(_omni):
        with open(_omni) as stream:
            _o = yaml.safe_load(stream)
        params['FollowPath'] = _o['controller']['FollowPath']
        _omni_smoother = _o.get('smoother', {{}})
"""
SMOOTHER_LINE = "                 'velocity_timeout': 1.0}]),"
SMOOTHER_NEW = "                 'velocity_timeout': 1.0, **_omni_smoother}]),"


def tagged(block):
    return [line + TAG for line in block.rstrip("\n").split("\n")]


def patch(path, anchor, hook, replace=None):
    with open(path) as f:
        lines = f.read().split("\n")
    if any(TAG in line for line in lines):
        return "already patched"
    try:
        i = lines.index(anchor)
    except ValueError:
        raise SystemExit(f"{path}: anchor not found (upstream changed?): {anchor.strip()}")
    lines[i:i] = tagged(hook)
    if replace:
        old, new = replace
        try:
            j = lines.index(old)
        except ValueError:
            raise SystemExit(f"{path}: line not found (upstream changed?): {old.strip()}")
        lines[j] = new + TAG + " was: " + old.strip()
    with open(path, "w") as f:
        f.write("\n".join(lines))
    return "patched"


def unpatch(path):
    with open(path) as f:
        lines = f.read().split("\n")
    if not any(TAG in line for line in lines):
        return "not patched"
    out = []
    for line in lines:
        if TAG not in line:
            out.append(line)
        elif " was: " in line:
            indent = line[: len(line) - len(line.lstrip())]
            out.append(indent + line.split(" was: ", 1)[1])
    with open(path, "w") as f:
        f.write("\n".join(out))
    return "restored"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--enable", action="store_true")
    mode.add_argument("--disable", action="store_true")
    ap.add_argument("nav2_wifi_dir")
    a = ap.parse_args()
    root = a.nav2_wifi_dir
    cfg = os.path.join(root, "go2_nav2", "config", "go2_omni.yaml")
    if a.enable:
        shutil.copyfile(SRC_CFG, cfg)
        p = patch(os.path.join(root, PLANNING), PLANNING_ANCHOR, PLANNING_HOOK)
        c = patch(os.path.join(root, CONTROLLER), CONTROLLER_ANCHOR, CONTROLLER_HOOK, (SMOOTHER_LINE, SMOOTHER_NEW))
        print(f"[omni] enabled: MPPI Omni + omni lattice (planning {p}, controller {c})")
    else:
        p = unpatch(os.path.join(root, PLANNING))
        c = unpatch(os.path.join(root, CONTROLLER))
        if os.path.exists(cfg):
            os.remove(cfg)
        print(f"[omni] disabled: forward-only RPP (planning {p}, controller {c})")


if __name__ == "__main__":
    sys.exit(main())
