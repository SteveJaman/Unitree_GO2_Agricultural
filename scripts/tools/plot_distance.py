#!/usr/bin/env python3
"""
plot_distance.py: distance_over_time.png from distance_servo.py's CSV log (csv_path).

  python3 scripts/tools/plot_distance.py servo_log.csv            # -> distance_over_time.png next to it
  python3 scripts/tools/plot_distance.py servo_log.csv --out x.png --target 0.50 --deadband 0.02

Top: the measured camera-to-plant distance against time, with the servo's target and
deadband band; gaps where nothing was seen. Bottom: the tool's X in the arm frame (what
the servo moves) with the steps it commanded. Prints a short summary: start and final
distance, time to settle inside the deadband, steps taken, overshoot.
"""
import argparse
import csv
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def read(path):
    rows = list(csv.DictReader(open(path)))
    f = lambda v: float(v) if v not in ("", None) else math.nan  # noqa: E731
    return {
        "t": [f(r["time_s"]) for r in rows],
        "d": [f(r["current_distance_m"]) for r in rows],
        "x": [f(r["tool_x_m"]) for r in rows],
        "step": [f(r["step_m"]) for r in rows],
        "state": [r["state"] for r in rows],
    }


def summary(data, target, deadband):
    t, d = data["t"], data["d"]
    seen = [(ti, di) for ti, di in zip(t, d) if math.isfinite(di)]
    if not seen:
        return {"seen": 0}
    t_first, d_first = seen[0]
    d_last = seen[-1][1]
    settle = None
    for k, (ti, di) in enumerate(seen):            # first time it stays inside the band to the end
        if all(abs(dj - target) <= deadband for _, dj in seen[k:]):
            settle = ti - t_first
            break
    sign0 = math.copysign(1, d_first - target)
    over = max([sign0 * (target - di) for _, di in seen] + [0.0])
    steps = sum(1 for s in data["state"] if s in ("step", "dry_run"))
    return {"seen": len(seen), "start_m": d_first, "final_m": d_last, "settle_s": settle,
            "overshoot_m": over, "steps": steps, "duration_s": t[-1] - t[0]}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--out", default=None, help="default: distance_over_time.png next to the CSV")
    ap.add_argument("--target", type=float, default=0.50)
    ap.add_argument("--deadband", type=float, default=0.02)
    ap.add_argument("--title", default="Arm distance servo: camera to lettuce")
    a = ap.parse_args()
    data = read(a.csv)
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.csv)), "distance_over_time.png")
    s = summary(data, a.target, a.deadband)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True, gridspec_kw={"height_ratios": [2.2, 1]})
    t = data["t"]
    ax1.axhspan(a.target - a.deadband, a.target + a.deadband, color="#2a9d8f", alpha=0.15, label=f"deadband ±{a.deadband * 100:.0f} cm")
    ax1.axhline(a.target, color="#2a9d8f", lw=1.2, ls="--", label=f"target {a.target:.2f} m")
    ax1.plot(t, data["d"], color="#264653", lw=1.6, marker=".", ms=3, label="measured distance")
    st = [ti for ti, sv in zip(t, data["state"]) if sv in ("step", "dry_run")]
    sd = [di for di, sv in zip(data["d"], data["state"]) if sv in ("step", "dry_run")]
    ax1.plot(st, sd, "o", color="#e76f51", ms=5, label="step commanded")
    ax1.set_ylabel("camera to lettuce (m)")
    ax1.grid(alpha=0.3)
    ax1.legend(loc="upper right", fontsize=8)
    title = a.title
    if s.get("seen"):
        title += (f"\nstart {s['start_m']:.3f} m, final {s['final_m']:.3f} m, {s['steps']} steps"
                  + (f", settled in {s['settle_s']:.1f} s" if s["settle_s"] is not None else ", did not settle"))
    ax1.set_title(title, fontsize=11)

    ax2.plot(t, data["x"], color="#6d597a", lw=1.6, label="tool X (arm frame)")
    ax2.set_ylabel("tool X (m)")
    ax2.set_xlabel("time (s)")
    ax2.grid(alpha=0.3)
    ax2.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"[plot] {out}")
    if s.get("seen"):
        print(f"[plot] {s['seen']} distance samples over {s['duration_s']:.1f} s: start {s['start_m']:.3f} m, "
              f"final {s['final_m']:.3f} m, {s['steps']} steps, overshoot {s['overshoot_m'] * 100:.1f} cm, "
              + (f"settled in {s['settle_s']:.1f} s" if s["settle_s"] is not None else "did not settle in the band"))
    else:
        print("[plot] no distance in the log: the tracker saw nothing")


if __name__ == "__main__":
    main()
