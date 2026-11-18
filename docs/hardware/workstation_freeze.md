# Workstation freezes during a dog session

The workstation (16 cores, 15 GB RAM, Intel UHD 750 for the desktop, Quadro RTX 4000)
froze, and had to be restarted, during `wifi_3d_nav.sh` sessions and Gazebo runs. There
were three causes.

| Cause | Seen | Fix |
|---|---|---|
| The VLM's `llama-server` (Ollama, llava:7b) grows about 1 GB of RAM a minute while it gets camera images, even with the model fully on the GPU | 2026-11-17: killed by the kernel at 6.8 GB. 2026-11-18: 3.4 → 3.9 GB in 40 s, free RAM 7.2 → 3.2 GB in 3 min | `scripts/tools/ollama_mem_guard.py`, started by `wifi_3d_nav.sh` |
| RViz drew on the CPU: `go2-lidar3d` had no `/dev/dri` and RViz started with `LIBGL_ALWAYS_SOFTWARE=1` | 2026-11-18, during auto explore | `scripts/tools/rviz_gpu.py`; [GPU rendering](../simulation/gpu_rendering.md) |
| Gazebo drew its sensors and window on the CPU | sim runs | `SIM_GL` in `gazebo/run_sim.sh`; [GPU rendering](../simulation/gpu_rendering.md) |

## The VLM memory guard

When the VLM is on (the default, or `--semantic`) and Ollama runs on this computer,
`wifi_3d_nav.sh` starts `ollama_mem_guard.py`, which stops when the session stops. Every 5 s
it reads `llama-server`'s RAM, and it unloads the model through Ollama's API
(`keep_alive: 0`) when:

- `llama-server` holds more than `GO2_VLM_MAX_GB` (4 GB), or
- the computer has less than `GO2_VLM_MIN_FREE_GB` (1.5 GB) free while `llama-server`
  holds over 1 GB.

The VLM's next request loads the model again. That took 7.2 s on 2026-11-18, and the
fresh `llama-server` held 0.19 GB. Each unload is logged in `vlm_guard.log` in the session's
twin log directory, with a line a minute showing the current size. `GO2_VLM_GUARD=0` turns
the guard off.

Check by hand:

```bash
python3 scripts/tools/ollama_mem_guard.py --once
```

## RViz on the GPU (2026-11-18, auto explore)

`rviz2` loaded Mesa's `iris_dri.so` (the Intel graphics) and used 13–16% of one core for
the whole run. The system load stayed at 6–12 on 16 cores, mostly YOLO (250–320%) and
RTAB-Map.
