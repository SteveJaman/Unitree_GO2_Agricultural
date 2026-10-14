@"
# CMU Stack — Legacy Archive

Everything in this folder belongs to the earlier iteration of the project,
based on the CMU ``autonomy_stack_go2`` running over Ethernet or the Unity
simulation.

**Not used by the current pipeline.** The active stack is:

    scripts/wifi_nav.sh  ->  go2-nav2-wifi  ->  Point-LIO + Nav2 over Wi-Fi

Kept here for reference in case we return to the Ethernet path or the
Unity simulation. Nothing in this folder is loaded by any active script.

## Contents

| Item | What |
|---|---|
| ``*.sh``, ``mock_sim.py`` | Legacy shell launchers and helpers |
| ``go2_integration_pkg/`` | ROS 2 package from the CMU era |
| ``simulation/`` | Unity scene helpers |
| ``cyclonedds_ethernet.xml`` | Ethernet DDS config |
| ``architecture.md``, ``ethernet.md``, ``wireless.md``, ``roadmap.md``, ``setup.md``, ``troubleshooting.md`` | CMU-era documentation |

## To restore any file

    git mv cmu-legacy/<file> scripts/

"@ | Out-File -Encoding utf8 cmu-legacy\README.md

git add cmu-legacy\README.md