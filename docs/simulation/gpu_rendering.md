# GPU rendering for the Gazebo sim

Gazebo draws the simulated lidar, the camera and its window with OpenGL. The sim image
sets `LIBGL_ALWAYS_SOFTWARE=1`, so until now all of that ran on the CPU (Mesa llvmpipe,
every core). Together with RViz, also drawn in software, and the VLM (ollama) using
6–7 GB of RAM, that froze the workstation.

`gazebo/run_sim.sh` now picks who draws with `SIM_GL`:

| `SIM_GL` | Draws on | Needs |
|---|---|---|
| `auto` (default) | `nvidia` if Docker has the nvidia runtime, else `intel`, else `software` | |
| `nvidia` | the Quadro RTX 4000, through PRIME offload (the desktop runs on the Intel graphics) | `nvidia-container-toolkit` |
| `intel` | the Intel UHD 750, through `/dev/dri` | nothing: works today |
| `software` | the CPU (the old behaviour) | |

The start line shows the choice, for example `[sim] starting go2-sim (GUI=1, GL=intel, ...)`.
If Gazebo misbehaves on a GPU, `SIM_GL=software ./gazebo/run_sim.sh up` brings the old
behaviour back.

Checked on the workstation (2026-11-18), `glxinfo -B` in `go2-sim:local`:
`llvmpipe (LLVM 15.0.7)` before, `Mesa Intel(R) Graphics (RKL GT1)`, OpenGL 4.6, with `SIM_GL=intel`.

## Using the NVIDIA card

Docker can't reach the Quadro until NVIDIA's container toolkit is installed (needs sudo):

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
```

After that, `SIM_GL=auto` picks `nvidia` by itself. Ollama already runs the VLM on the
Quadro (8 GB), so the two share its memory.

## RViz

RViz runs in the `go2-lidar3d` container (go2-nav2-wifi), which was created without
`/dev/dri` and starts RViz with `LIBGL_ALWAYS_SOFTWARE=1`. It still draws on the CPU.
Moving it to the GPU means recreating that container with the device added.
