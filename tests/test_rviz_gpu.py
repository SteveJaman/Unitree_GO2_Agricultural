"""scripts/tools/rviz_gpu.py on copies of go2-nav2-wifi's mapping.sh and lidar3d compose file."""
import os
import shutil
import subprocess
import sys

import pytest

HERE = os.path.dirname(__file__)
TOOL = os.path.join(HERE, "..", "scripts", "tools", "rviz_gpu.py")
SRC = os.path.expanduser(os.environ.get("NAV2_WIFI_DIR", "~/files/go2-nav2-wifi"))
ORIG = os.path.join(SRC, "docker", "docker-compose.lidar3d.yml.pre-gpu")


@pytest.mark.skipif(not os.path.isfile(ORIG), reason="needs a go2-nav2-wifi clone with the .pre-gpu originals")
def test_rviz_gets_the_gpu(tmp_path):
    os.makedirs(tmp_path / "docker")
    shutil.copy(ORIG, tmp_path / "docker" / "docker-compose.lidar3d.yml")
    shutil.copy(os.path.join(SRC, "mapping.sh.pre-gpu"), tmp_path / "mapping.sh")
    out = subprocess.run([sys.executable, TOOL, str(tmp_path)], capture_output=True, text=True)
    assert out.returncode == 0, out.stdout + out.stderr
    m = (tmp_path / "mapping.sh").read_text()
    assert "LIBGL_ALWAYS_SOFTWARE=1" not in m
    assert m.count("-e LIBGL_ALWAYS_SOFTWARE=${GO2_RVIZ_SOFTWARE_GL:-0} ") == 2      # both RViz launches
    c = (tmp_path / "docker" / "docker-compose.lidar3d.yml").read_text()
    assert "      - /dev/dri:/dev/dri\n" in c and '"${GO2_RENDER_GID:-110}"' in c
    assert "LIBGL_ALWAYS_SOFTWARE=${GO2_RVIZ_SOFTWARE_GL:-0}" in c
    again = subprocess.run([sys.executable, TOOL, str(tmp_path)], capture_output=True, text=True)
    assert again.returncode == 0 and "edits" not in again.stdout                     # idempotent
    check = subprocess.run([sys.executable, TOOL, "--check", str(tmp_path)], capture_output=True, text=True)
    assert check.returncode == 0
