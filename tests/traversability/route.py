#!/usr/bin/env python3
"""Seeded random spawn and goals for a world, from its truth JSON. Standard library only.

  route.py WORLD_JSON SEED     -> prints {"seed", "spawn": [x, y, yaw], "goals": [...]}

Points keep CLEAR_M from every object footprint and from the walls, and each
goal is at least MIN_LEG_M from the point before it. The fixed sequence is what
nav_trial_runner.py uses without --randomize.
"""
import json
import math
import random
import sys

CLEAR_M = 0.9
MIN_LEG_M = 3.0
N_GOALS = 3
FIXED = {"spawn": [0.0, 0.0, 0.0],
         "goals": [{"name": "goal_1_mid", "x": 5.0, "y": 0.0, "yaw": 0.0},
                   {"name": "goal_2_end", "x": 10.0, "y": 0.0, "yaw": 0.0},
                   {"name": "goal_3_return", "x": 0.0, "y": 0.0, "yaw": 3.14}]}


def clear_of(x, y, objects, margin):
    for o in objects:
        c, s = math.cos(o["yaw"]), math.sin(o["yaw"])
        dx, dy = x - o["x"], y - o["y"]
        ex = max(0.0, abs(c * dx + s * dy) - o["sx"] / 2)
        ey = max(0.0, abs(-s * dx + c * dy) - o["sy"] / 2)
        if math.hypot(ex, ey) < margin:
            return False
    return True


def bounds(world):
    xs = [w["x"] for w in world["walls"]]
    ys = [w["y"] for w in world["walls"]]
    return min(xs), max(xs), min(ys), max(ys)


def sample(world, seed):
    rng = random.Random(seed)
    x0, x1, y0, y1 = bounds(world)
    objects = world["objects"]

    def point():
        for _ in range(10000):
            x = rng.uniform(x0 + CLEAR_M, x1 - CLEAR_M)
            y = rng.uniform(y0 + CLEAR_M, y1 - CLEAR_M)
            if clear_of(x, y, objects, CLEAR_M):
                return x, y
        raise SystemExit(f"seed {seed}: no free point found")

    sx, sy = point()
    route = {"seed": seed, "spawn": [round(sx, 3), round(sy, 3), round(rng.uniform(-math.pi, math.pi), 3)], "goals": []}
    px, py = sx, sy
    for k in range(N_GOALS):
        for _ in range(1000):
            gx, gy = point()
            if math.hypot(gx - px, gy - py) >= MIN_LEG_M:
                break
        route["goals"].append({"name": f"goal_{k + 1}", "x": round(gx, 3), "y": round(gy, 3), "yaw": 0.0})
        px, py = gx, gy
    return route


if __name__ == "__main__":
    print(json.dumps(sample(json.load(open(sys.argv[1])), int(sys.argv[2]))))
