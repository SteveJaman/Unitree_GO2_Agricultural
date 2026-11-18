"""scripts/tools/relay_lidar3d_raw.py on copies of go2-nav2-wifi's mapping.sh and relay."""
import os
import shutil
import subprocess
import sys

import pytest

HERE = os.path.dirname(__file__)
TOOL = os.path.join(HERE, "..", "scripts", "tools", "relay_lidar3d_raw.py")
SRC = os.path.expanduser(os.environ.get("NAV2_WIFI_DIR", "~/files/go2-nav2-wifi"))


@pytest.mark.skipif(not os.path.isfile(os.path.join(SRC, "mapping.sh.pre-raw")),
                    reason="needs a go2-nav2-wifi clone with the original mapping.sh.pre-raw")
def test_both_relays_get_the_switch(tmp_path):
    os.makedirs(tmp_path / "ws" / "scripts")
    shutil.copy(os.path.join(SRC, "mapping.sh.pre-raw"), tmp_path / "mapping.sh")
    shutil.copy(os.path.join(SRC, "ws", "scripts", "robot_relay_wifi.py.pre-raw"),
                tmp_path / "ws" / "scripts" / "robot_relay_wifi.py")
    out = subprocess.run([sys.executable, TOOL, str(tmp_path)], capture_output=True, text=True)
    assert out.returncode == 0, out.stdout + out.stderr
    m = (tmp_path / "mapping.sh").read_text()
    # The sensors relay, and the Jetson SLAM container whose relay streams Point-LIO.
    assert '"exec env GO2_LIDAR3D_RAW=${GO2_LIDAR3D_RAW:-0} GO2_HOST_IP=' in m
    assert '"exec docker exec -e GO2_LIDAR3D_RAW=${GO2_LIDAR3D_RAW:-0} -e GO2_LIDAR3D_BACKEND=' in m
    r = (tmp_path / "ws" / "scripts" / "robot_relay_wifi.py").read_text()
    assert 'topic = "/lidar3d/raw/" + topic.rsplit("/", 1)[1]' in r
    again = subprocess.run([sys.executable, TOOL, str(tmp_path)], capture_output=True, text=True)
    assert again.returncode == 0 and "edits" not in again.stdout          # idempotent
