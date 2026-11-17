# Plan: per-leg step-over with a learned walking controller

[← Docs index](../README.md) · [Traversability handoff](traversability_handoff.md) · [Sim record](../simulation/traversability_sim.md)

Status: **plan, not started** (2026-11-16).

## Why

The project proposes a dog that lifts its feet to step over low clutter. In the sim it
does: the gait supervisor raises the foot lift to 0.12 m near low items, and foot
clearance over items ≤ 0.10 m rose from 26 % to 53 % of passes. On the real Go2 that
path is closed:

- **Sport mode has no foot-height control on this firmware.** `FootRaiseHeight`
  returns 3203. Unitree's ROS 2 release notes tell users to remove API 1014
  (`FOOTRAISEHEIGHT`).
- **The remote's L1 + ↑ does not raise the feet either** (measured 2026-11-16 with
  `scripts/robot/step_height_probe.py`). The median clearance was 5.4 cm before and
  5.1 cm after, with the lowest 10 % of steps at 1–2 cm both times.
- **The built-in gaits can't be tuned.** ClassicWalk lifts 4.1–6.5 cm, FreeWalk
  1.2–8.2 cm (2026-11-11).

Per-leg control means leaving sport mode for **low-level mode**: joint position or
torque commands at about 500 Hz, with Unitree's balance controller off. We then need
our own walking controller. The practical choice is a **learned policy that takes a
foot-swing height as an input**, so the traversability stack can ask for a high step
near an object.

## Candidate: Walk These Ways (Go2 port)

[Walk These Ways](https://arxiv.org/pdf/2212.03238) (Margolis & Agrawal) trains one
policy that takes, besides velocity, gait parameters including the **foot-swing
height**. It was deployed on a Go1. A community port for the Go2,
[Teddy-Liao/walk-these-ways-go2](https://www.github.com/Teddy-Liao/walk-these-ways-go2)
(MIT licence), provides:

- **Training** in Isaac Gym, and a **pretrained Go2 model**
  (`runs/gait-conditioned-agility/pretrain-go2`).
- **Deployment** over LCM: a C++ bridge `lcm_position_go2`, using unitree_sdk2, puts
  the dog in low-level mode, and `deploy_policy.py` runs the policy. It **runs on a
  PC over Ethernet**. Running it on the dog's Jetson is listed as unfinished.
- **Safety:** R2 starts the policy; L2 + B puts the dog in damping mode. The dog must
  be hung up or lying down when low-level mode starts.
- **Training needs** an NVIDIA GPU with at least 8 GB, PyTorch 1.10 with CUDA 11.3,
  and Isaac Gym.

The port is small (20 commits, last example run dated 2024-03) and research-grade.
Plan on fixing things.

Alternatives, if it stalls:

- [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) trains a Go2
  task, but its real-robot deployment covers only G1, H1 and H1_2.
- [go2-sim2real-deploy](https://backiee.wasmer.app/http_github_com/saifahmadgit/go2-sim2real-deploy)
  deploys walking and stair policies, with no foot-height input.
- One study found learned controllers had a large sim-to-real gap and kept Unitree's
  built-in controller instead
  ([multi-agent pushing](https://arxiv.org/pdf/2411.07104)). That is the main risk
  here.

## Hardware we have

| | |
|---|---|
| Workstation | Quadro RTX 4000 (**8 GB**, driver 615), 16 cores, **15 GB RAM** |
| Dog | Go2 EDU with its onboard Jetson |
| Network | Shared lab Wi-Fi (`icps-lab_5G`), with 1-second stalls on the dog's link |

8 GB of GPU memory and 15 GB of RAM are the stated minimum for training. Start from
the pretrained model and train only if needed, with fewer parallel environments.
**The low-level loop must not run over this Wi-Fi.** A one-second stall at 500 Hz
means the dog falls. Use an **Ethernet cable** to the laptop (tethered tests), or get
the policy running on the Jetson.

## Phases

Each phase has a gate. Stop and reassess if a gate fails.

**0. Safety kit (before anything moves).**

- A **hanging harness or gantry** for the dog. Low-level mode starts with Unitree's
  balance control off.
- An Ethernet cable from the laptop to the dog.
- One person on the remote for L2 + B (damping) at all times.
- A written stop procedure.
- Check the warranty and the lab's rules on low-level control.

**1. Pretrained policy in the sim (2–3 days).**

- Install Isaac Gym and the port in its own Python environment on the workstation.
- Play the pretrained Go2 model.
- Sweep the foot-swing height command and measure the swing in the sim.
- *Gate:* the policy walks, and the swing height follows the command over at least
  4–15 cm.

**2. A second simulator (2 days, recommended).**

- Run the same policy in MuJoCo (`unitree_mujoco`).
- *Gate:* it still walks and follows the swing height. A policy that only works in the
  simulator it was trained in rarely survives the real dog.

**3. Real dog, hung up (2–4 days).**

- Deploy over Ethernet with the legs in the air.
- Check joint directions, limits, the start-up posture, and that L2 + B damping works.
- *Gate:* smooth leg motion, no limit hits, and damping works every time.

**4. Real dog, on the ground, tethered (1–2 weeks).**

- Slow walking on flat floor first, with the harness slack but attached.
- Then sweep the swing height and measure each setting with `step_height_probe.py`
  (read-only).
- Then step over objects: 3, 5, 8, 10 cm, foam first, then the real items.
- *Gate:* the measured clearance follows the command (for example ≥ 10 cm when 12 cm
  is asked), and the dog crosses 8 cm foam without stumbling over at least 10 tries.

**5. Integration with navigation (1 week).**

- A new motion bridge replaces the sport-mode one (`robot_sport_bridge.py`):
  - Nav2's `/cmd_vel` becomes the policy's velocity command.
  - The gait supervisor's `/gait/command` (NORMAL / CAREFUL / HIGH_STEP) becomes its
    swing-height and speed commands. This is the `FootRaiseHeight` call the stack was
    built for.
- Raise the robot's step-over limit (`TRAV_STEP_OVER`) to the measured clearance
  minus a margin.
- SLAM is unaffected, since Point-LIO uses the LiDAR and its own IMU.
- *Gate:* a full `--semantic` run over a lane of items, tethered.

**6. Evaluation (1 week).** Use the sim protocol on the real dog:

- Built-in controller (avoid > 5 cm) against the learned policy (step over).
- The same lane, items and goals, with n ≥ 5 runs each.
- Measures: foot clearance from the probe, contacts from video, goals reached, falls.

Total: roughly **5–7 weeks** if the pretrained model transfers, and longer if it needs
retraining.

## Until then: the real dog avoids what it can't clear

With the built-in controller, the real dog should walk over ≤ 3 cm and avoid anything
taller than ~5 cm. Today it walks into 3–12 cm items: the route planner ignores
anything under 12 cm, and the local height band only marks 3–22 cm "caution", which
slows the dog but doesn't reroute it. See
[traversability_handoff.md](traversability_handoff.md).

## Risks

| Risk | Mitigation |
|---|---|
| The dog falls in low-level mode | Harness for phases 3–4; L2 + B damping; foam objects first |
| Sim-to-real gap: it walks worse than Unitree's controller | Phase 2 gate; keep sport mode as the fallback |
| Wi-Fi stalls | Ethernet or Jetson only for the control loop |
| The port is research-grade and unmaintained | Budget time to fix it; alternatives listed above |
| GPU is at the stated minimum | Pretrained model first; fewer parallel environments |
| Motor heat and battery during long tests | Short runs, check motor temperatures in `/lowstate` |
| Warranty or lab rules | Check before phase 3 |
