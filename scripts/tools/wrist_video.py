#!/usr/bin/env python3
"""
wrist_video.py: V2, the wrist camera of one orbit per species and shell, with the YOLO box.

  python3 scripts/tools/wrist_video.py --runs SWEEP/runs --species basil --out-dir VIDEO_DIR

For each shell, the representative run is the one with the most executed waypoints of that
shell (ties: the first plant). Its logged wrist frames (5 per waypoint, captured after the arm
settled; orbit_waypoints.py) are played in capture order, each held --hold s, with the box of
~/.cache/go2-yolo/<species>_yolo11s.pt (conf >= --conf, 0.5) drawn and a caption (species, shell,
azimuth, confidence). Writes <out-dir>/wrist_<species>_<shell>.mp4 (H.264 when ffmpeg is
available, else mp4v). These are the frames the coverage and detection numbers come from,
not a continuous stream: between waypoints the arm moves and nothing is captured.
"""
import argparse
import glob
import os
import re
import shutil
import subprocess
import tempfile

import cv2

ORANGE = (52, 104, 235)  # BGR of #eb6834


def ffmpeg():
    if os.environ.get("FFMPEG"):
        return os.environ["FFMPEG"]
    if shutil.which("ffmpeg"):
        return shutil.which("ffmpeg")
    try:
        import imageio_ffmpeg

        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        return None


def frames_by_shell(run_dir):
    out = {}
    for p in sorted(glob.glob(os.path.join(run_dir, "frames", "k*_*.jpg"))):
        m = re.match(r"k(\d+)_(\w+?)_az([+-]\d+)_f(\d+)\.jpg", os.path.basename(p))
        if m:
            out.setdefault(m.group(2), []).append((int(m.group(1)), int(m.group(3)), int(m.group(4)), p))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", required=True)
    ap.add_argument("--species", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--hold", type=float, default=0.8, help="seconds per frame")
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--conf", type=float, default=0.5, help="boxes drawn at or above this confidence")
    a = ap.parse_args()
    from ultralytics import YOLO

    model = YOLO(os.path.expanduser(f"~/.cache/go2-yolo/{a.species}_yolo11s.pt"))
    runs = sorted(d for d in glob.glob(os.path.join(a.runs, "*")) if os.path.isdir(os.path.join(d, "frames")))
    per_run = {r: frames_by_shell(r) for r in runs}
    os.makedirs(a.out_dir, exist_ok=True)
    ff = ffmpeg()
    for shell in ("eq", "top", "under"):
        cand = [(len({k for k, *_ in fr.get(shell, [])}), r) for r, fr in per_run.items() if fr.get(shell)]
        if not cand:
            print(f"[wrist] {a.species} {shell}: no frames (shell not reached)")
            continue
        best = max(cand, key=lambda c: (c[0], -runs.index(c[1])))[1]
        frames = sorted(per_run[best][shell])
        tmp = tempfile.mkdtemp()
        n = 0
        for k, az, f, p in frames:
            img = cv2.imread(p)
            res = model.predict(img, conf=a.conf, verbose=False)[0]
            conf = 0.0
            for b in res.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                conf = max(conf, float(b.conf))
                cv2.rectangle(img, (x1, y1), (x2, y2), ORANGE, 3)
                cv2.putText(img, f"{a.species} {float(b.conf):.2f}", (x1 + 4, max(22, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, ORANGE, 2)
            cv2.rectangle(img, (0, 0), (img.shape[1], 30), (0, 0, 0), -1)
            cv2.putText(img, f"{a.species} | {shell} shell | az {az:+d} deg | waypoint {k} frame {f} | "
                        f"{'det %.2f' % conf if conf else 'no detection'}", (8, 21), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (255, 255, 255), 1, cv2.LINE_AA)
            for _ in range(max(1, round(a.hold * a.fps))):
                cv2.imwrite(os.path.join(tmp, f"{n:05d}.png"), img)
                n += 1
        out = os.path.join(a.out_dir, f"wrist_{a.species}_{shell}.mp4")
        if ff:
            subprocess.run([ff, "-y", "-loglevel", "error", "-framerate", str(a.fps), "-i", os.path.join(tmp, "%05d.png"),
                            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", out], check=True)
        else:
            first = cv2.imread(os.path.join(tmp, "00000.png"))
            vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (first.shape[1], first.shape[0]))
            for i in range(n):
                vw.write(cv2.imread(os.path.join(tmp, f"{i:05d}.png")))
            vw.release()
        shutil.rmtree(tmp, ignore_errors=True)
        print(f"[wrist] {out}: {os.path.basename(best)}, {len(frames)} frames, {n / a.fps:.1f} s")


if __name__ == "__main__":
    main()
