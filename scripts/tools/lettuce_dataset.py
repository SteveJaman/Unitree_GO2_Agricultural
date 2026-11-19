#!/usr/bin/env python3
"""
lettuce_dataset.py: labelled lettuce images from the Gazebo CropKing NFT 4-6 world, to
fine-tune YOLO for the simulation demo (COCO has no lettuce class: yolo11s.pt sees
nothing on the rack).

  python3 scripts/tools/lettuce_dataset.py --frames 600            # -> ~/.cache/go2-yolo/lettuce_dataset
  python3 scripts/tools/lettuce_dataset.py --frames 600 --train    # + fine-tune yolo11s -> lettuce_yolo11s.pt
  python3 scripts/tools/lettuce_dataset.py --world cropking_nft46_basil --label 2 --class-name basil \
      --out ~/.cache/go2-yolo/basil_dataset --frames 350 --prefix basil_ --wrist-frac 0.6 \
      --train --epochs 30 --from-weights ~/.cache/go2-yolo/lettuce_yolo11s.pt   # a species, from the lettuce model

How: Gazebo itself renders and labels. The world (gazebo/worlds/cropking_nft46.sdf; every
lettuce carries the gz Label system, label 1) gets one extra static camera model with a
boundingbox_camera and an rgbd_camera side by side (640x480, 69 deg: the D435i colour
camera). The script runs `gz sim -s` (server only; sensors render through the desktop's X
display, like render_world.sh: --headless-rendering crashes in Ogre here) in the go2-sim
image, moves that camera with the world's set_pose service to random poses around the rack,
and for each pose saves the rgbd_camera's image (the wrist camera's sensor type: the
boundingbox_camera's own image is much brighter, and a model trained on it missed the
lettuce on the arm) with the boundingbox_camera's 2D boxes of the same sim stamp (visible
part of each lettuce: occlusion handled by Gazebo), in YOLO format (class 0 = lettuce).
A --wrist-frac share of the poses (0.35) copy the claw camera at the rack: 0.62-0.82 m
high, level, 0.28-0.90 m in front of a planted site, mostly the target.

Training (--train) runs on the host's GPU with ultralytics (yolo11s.pt start, 640 px),
and copies the best weights to ~/.cache/go2-yolo/lettuce_yolo11s.pt, the model
run_sim_demo.sh gives yolo_depth_tracker.py.

Inside the container this file runs with --inside (started by the host side).
"""
import argparse
import json
import math
import os
import random
import shutil
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CACHE = os.path.expanduser("~/.cache/go2-yolo")
IMAGE = "go2-sim:local"
W, H, HFOV = 640, 480, 1.21

CAMERA_MODEL = f"""
    <model name="dataset_cam"><static>true</static><pose>1.8 0.1 0.7 0 0 0</pose>
      <link name="l"><sensor name="boxes" type="boundingbox_camera">
        <update_rate>10</update_rate><always_on>true</always_on><topic>dataset/boxes</topic>
        <box_type>2d</box_type>
        <camera><horizontal_fov>{HFOV}</horizontal_fov><image><width>{W}</width><height>{H}</height></image>
          <clip><near>0.05</near><far>12</far></clip></camera>
      </sensor>
      <sensor name="rgbd" type="rgbd_camera">
        <update_rate>10</update_rate><always_on>true</always_on><topic>dataset/rgbd</topic>
        <camera><horizontal_fov>{HFOV}</horizontal_fov><image><width>{W}</width><height>{H}</height></image>
          <clip><near>0.1</near><far>10</far></clip></camera>
      </sensor></link>
    </model>
"""


# ------------------------------------------------------------------ inside the container
def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def sample_pose(meta, rng, wrist_frac=0.35):
    """Camera (x, y, z, roll, pitch, yaw) looking at a lettuce, mostly from the front of the rack.

    A wrist_frac share copies the demo's claw camera at the rack: 0.62-0.82 m high, level,
    0.28-0.90 m in front of a planted site (the target most of the time), as the servo sees
    it from below the channel top."""
    cx = meta["rack"]["channel_x"]
    sy = meta["rack"]["site_y"]
    top = meta["rack"]["channel_top_z"]
    front = meta["rack"]["front_edge_x"]
    planted = meta["rack"].get("planted") or [[0, k] for k in range(len(sy))]
    if rng.random() < wrist_frac:                             # the demo's wrist view: low and level
        tp = meta.get("target_plant", {})
        i, k = (tp["channel"], tp["site"]) if tp and rng.random() < 0.6 else rng.choice(planted)
        tx, ty = cx[i], sy[k]
        d = rng.uniform(0.28, 0.90)
        x, y, z = tx - d, ty + rng.uniform(-0.15, 0.15), rng.uniform(0.62, 0.82)
        yaw = math.atan2(ty - y, tx - x) + rng.uniform(-0.15, 0.15)
        pitch = rng.uniform(-0.12, 0.08)                      # gz: negative pitch looks up
    else:
        i = 0 if rng.random() < 0.55 else rng.randrange(len(cx))
        tx, ty, tz = cx[i], rng.choice(sy), top + 0.06
        d = rng.uniform(0.35, 1.5)
        az = math.pi + rng.uniform(-1.1, 1.1)                 # around the front (-x) side
        x, y = tx + d * math.cos(az), ty + d * math.sin(az)
        x = min(x, front - 0.05)
        z = rng.uniform(0.55, 1.5)
        lx, ly, lz = tx + rng.uniform(-0.15, 0.15), ty + rng.uniform(-0.15, 0.15), tz + rng.uniform(-0.05, 0.08)
        yaw = math.atan2(ly - y, lx - x)
        pitch = math.atan2(z - lz, math.hypot(lx - x, ly - y))  # positive pitch looks down
    return x, y, z, rng.uniform(-0.06, 0.06), pitch, yaw


def inside(a):
    from gz.msgs10.annotated_axis_aligned_2d_box_v_pb2 import AnnotatedAxisAligned2DBox_V
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_pb2 import Pose
    from gz.transport13 import Node
    import numpy as np
    from PIL import Image as PILImage

    meta = json.load(open(f"/sim/worlds/{a.world}.json"))
    world = open(f"/sim/worlds/{a.world}.sdf").read().replace("@RENDER_ENGINE@", "ogre2")
    # the observer cameras only cost render time here
    out_lines, skip = [], False
    for line in world.splitlines(keepends=True):
        if '<model name="cam_' in line:
            continue
        out_lines.append(line)
    world = "".join(out_lines).replace("  </world>", CAMERA_MODEL + "  </world>")
    os.makedirs("/tmp/ds", exist_ok=True)
    open("/tmp/ds/world.sdf", "w").write(world)
    def start_gz():
        log = open("/tmp/ds/gz.log", "a")
        return subprocess.Popen(["gz", "sim", "-s", "-r", "/tmp/ds/world.sdf"], stdout=log, stderr=subprocess.STDOUT)
    gz = start_gz()

    node = Node()
    latest = {}

    def on_img(msg):
        latest["img"] = msg

    def on_boxes(msg):
        latest["boxes"] = msg

    # the image from an rgbd_camera, the sensor type of the wrist camera (its colours differ
    # from the boundingbox_camera's own image), the boxes from the boundingbox_camera at
    # the same pose, rate and sim stamp
    node.subscribe(Image, "/dataset/rgbd/image", on_img)
    node.subscribe(AnnotatedAxisAligned2DBox_V, "/dataset/boxes", on_boxes)
    t0 = time.time()
    while "img" not in latest:
        if time.time() - t0 > 180 or gz.poll() is not None:
            gz.kill()
            sys.exit("no image from the dataset camera; see /tmp/ds/gz.log:\n" + open("/tmp/ds/gz.log").read()[-2000:])
        time.sleep(0.5)
    print(f"[dataset] Gazebo up after {time.time() - t0:.0f} s", flush=True)

    out = "/out"
    for split in ("train", "val"):
        os.makedirs(f"{out}/images/{split}", exist_ok=True)
        os.makedirs(f"{out}/labels/{split}", exist_ok=True)
    rng = random.Random(a.seed)
    stamp = lambda m: m.header.stamp.sec + m.header.stamp.nsec * 1e-9  # noqa: E731
    saved = boxes_total = 0
    fails, restarts, n = 0, 0, -1

    def restart_gz(why):
        # Gazebo stops answering set_pose after a few hundred frames (seen 2026-11-18/19):
        # restart the server and carry on with the next frame number
        nonlocal gz, restarts
        restarts += 1
        print(f"[dataset] Gazebo not answering ({why}): restart {restarts}", flush=True)
        gz.kill()
        gz.wait(timeout=30)
        latest.clear()
        gz = start_gz()
        t_up = time.time()
        while "img" not in latest and time.time() - t_up < 180:
            time.sleep(0.5)
        if "img" not in latest:
            sys.exit("Gazebo did not come back after a restart")
    while saved < a.frames:
        if restarts > 20:
            print("[dataset] more than 20 Gazebo restarts: stopping", flush=True)
            break
        n += 1
        x, y, z, r, p, yw = sample_pose(meta, rng, a.wrist_frac)
        q = quat(r, p, yw)
        req = Pose()
        req.name = "dataset_cam"
        req.position.x, req.position.y, req.position.z = x, y, z
        req.orientation.w, req.orientation.x, req.orientation.y, req.orientation.z = q
        ok, _ = node.request(f"/world/{meta['world']}/set_pose", req, Pose, Boolean, 2000)
        if not ok:
            fails += 1
            if fails >= 5:
                restart_gz("5 set_pose failures in a row")
                fails = 0
            continue
        fails = 0
        # wait for a frame rendered after the move: both topics with the same, newer stamp
        before = stamp(latest["img"])
        t1 = time.time()
        pair = None
        while time.time() - t1 < 5:
            time.sleep(0.05)
            im, bx = latest.get("img"), latest.get("boxes")
            if im is None or bx is None:
                continue
            if stamp(im) > before + 0.25 and abs(stamp(im) - stamp(bx)) < 1e-6:
                pair = (im, bx)
                break
        if pair is None:
            fails += 1                                # no new frame for 5 s
            if fails >= 3:
                restart_gz("no new frame for 3 poses")
                fails = 0
            continue
        im, bx = pair
        arr = np.frombuffer(im.data, np.uint8).reshape(im.height, im.width, -1)[:, :, :3]
        rows = []
        for b in bx.annotated_box:
            if b.label != a.label:
                continue
            x0, y0 = b.box.min_corner.x, b.box.min_corner.y
            x1, y1 = b.box.max_corner.x, b.box.max_corner.y
            x0, y0, x1, y1 = max(0, x0), max(0, y0), min(W - 1, x1), min(H - 1, y1)
            if x1 - x0 < 10 or y1 - y0 < 8:
                continue
            patch = arr[int(y0):int(y1) + 1, int(x0):int(x1) + 1]
            if patch.size and float(patch.std()) < 4.0:
                continue                      # a blank patch: the camera is inside or behind an occluder
            rows.append(f"0 {(x0 + x1) / 2 / W:.5f} {(y0 + y1) / 2 / H:.5f} {(x1 - x0) / W:.5f} {(y1 - y0) / H:.5f}")
        split = "val" if n % 8 == 0 else "train"
        PILImage.fromarray(arr).save(f"{out}/images/{split}/{a.prefix}{n:05d}.jpg", quality=92)
        open(f"{out}/labels/{split}/{a.prefix}{n:05d}.txt", "w").write("\n".join(rows) + ("\n" if rows else ""))
        saved += 1
        boxes_total += len(rows)
        if saved % 50 == 0:
            print(f"[dataset] {saved}/{a.frames} frames, {boxes_total} boxes", flush=True)
    gz.terminate()
    try:
        gz.wait(timeout=20)
    except subprocess.TimeoutExpired:
        gz.kill()
    open(f"{out}/data.yaml", "w").write(f"path: {out}\ntrain: images/train\nval: images/val\nnames:\n  0: {a.class_name}\n")
    print(f"[dataset] done: {saved} frames, {boxes_total} {a.class_name} boxes", flush=True)
    os._exit(0)


# ------------------------------------------------------------------ host side
def host(a):
    out = os.path.abspath(a.out)
    if os.path.isdir(out) and not a.keep:
        shutil.rmtree(out)
    os.makedirs(out, exist_ok=True)
    dev = ["--device", "/dev/dri"] if os.path.exists("/dev/dri") else []
    cmd = ["docker", "run", "--rm", "--network", "host", "--ipc", "host", *dev,
           "-e", f"GZ_PARTITION=lettuce{os.getpid()}", "-e", "GZ_IP=127.0.0.1", "-e", "HOME=/tmp",
           "-e", "GZ_SIM_RESOURCE_PATH=/sim/worlds/models:/sim/models",
           "-e", f"DISPLAY={os.environ.get('DISPLAY', ':0')}", "-e", "QT_X11_NO_MITSHM=1", "-v", "/tmp/.X11-unix:/tmp/.X11-unix:rw",
           "-v", f"{REPO}/gazebo:/sim:ro", "-v", f"{REPO}/scripts/tools:/tools:ro", "-v", f"{out}:/out",
           "--user", f"{os.getuid()}:{os.getgid()}", "--group-add", "video", IMAGE,
           "bash", "-c", f"source /opt/ros/humble/setup.bash 2>/dev/null; python3 -u /tools/lettuce_dataset.py --inside "
                         f"--frames {a.frames} --seed {a.seed} --wrist-frac {a.wrist_frac} --prefix {a.prefix} "
                         f"--world {a.world} --label {a.label} --class-name {a.class_name}"]
    if subprocess.call(cmd) != 0:
        sys.exit("dataset generation failed")
    if a.train:
        train(out, a)


def train(out, a):
    from ultralytics import YOLO
    # data.yaml is written inside the container (path /out): point it at the host folder
    open(os.path.join(out, "data.yaml"), "w").write(
        f"path: {out}\ntrain: images/train\nval: images/val\nnames:\n  0: {a.class_name}\n")
    os.makedirs(CACHE, exist_ok=True)
    base = os.path.join(CACHE, "yolo11s.pt") if os.path.exists(os.path.join(CACHE, "yolo11s.pt")) else "yolo11s.pt"
    if a.from_weights:
        base = os.path.expanduser(a.from_weights)   # fine-tune an earlier lettuce model
    model = YOLO(base)
    model.train(data=os.path.join(out, "data.yaml"), epochs=a.epochs, imgsz=640, batch=16, device=0 if a.device == "" else a.device,
                project=os.path.join(CACHE, "runs"), name=a.class_name, exist_ok=True, patience=15, plots=False,
                hsv_h=0.02, hsv_s=0.5, hsv_v=0.4, degrees=5, fliplr=0.5, mosaic=1.0, verbose=False)
    best = os.path.join(CACHE, "runs", a.class_name, "weights", "best.pt")
    dst = os.path.join(CACHE, f"{a.class_name}_yolo11s.pt")
    shutil.copy(best, dst)
    print(f"[dataset] weights: {dst}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=int, default=600)
    ap.add_argument("--seed", type=int, default=46)
    ap.add_argument("--out", default=os.path.join(CACHE, "lettuce_dataset"))
    ap.add_argument("--keep", action="store_true", help="do not clear OUT first")
    ap.add_argument("--train", action="store_true", help="then fine-tune yolo11s on it (host GPU)")
    ap.add_argument("--train-only", action="store_true", help="skip generation: train on OUT")
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--device", default="")
    ap.add_argument("--wrist-frac", type=float, default=0.35, help="share of frames copying the claw camera at the rack")
    ap.add_argument("--prefix", default="f", help="image file prefix (several runs into one --keep dataset)")
    ap.add_argument("--from-weights", default="", help="start training from these weights instead of yolo11s.pt")
    ap.add_argument("--world", default="cropking_nft46", help="gazebo/worlds/<world>.sdf/.json (species: cropking_nft46_<species>)")
    ap.add_argument("--label", type=int, default=1, help="gz Label of the class (config/species_config.yaml)")
    ap.add_argument("--class-name", default="lettuce", help="class name; the weights go to ~/.cache/go2-yolo/<class>_yolo11s.pt")
    ap.add_argument("--inside", action="store_true", help=argparse.SUPPRESS)
    a = ap.parse_args()
    if a.inside:
        inside(a)
    elif a.train_only:
        train(os.path.abspath(a.out), a)
    else:
        host(a)


if __name__ == "__main__":
    main()
