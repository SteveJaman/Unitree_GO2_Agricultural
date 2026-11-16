import re, sys
p = "scripts/nodes/traversability_node.py"
s = open(p).read()
done = []
if "min_range" not in s:
    s, a = re.subn(r"(p\('max_range', 5\.0\)[^\n]*\n)", r"\1        p('min_range', 0.6)                 # m: ignore points this close (the robot's own body)\n", s, count=1)
    s, b = re.subn(r"(self\.base_height, self\.max_range = [^\n]*\n)", r"\1        self.min_range = float(gp('min_range'))\n", s, count=1)
    s, c = re.subn(r"near = np\.hypot\(pts\[:, 0\] - bx, pts\[:, 1\] - by\) < self\.max_range",
                   "dist = np.hypot(pts[:, 0] - bx, pts[:, 1] - by)\n            near = (dist < self.max_range) & (dist > self.min_range)", s, count=1)
    if (a, b, c) != (1, 1, 1):
        sys.exit(f"ERROR min_range: matches {(a, b, c)}")
    done.append("min_range")
if "cv2 = None" not in s:
    s, a = re.subn(r"^import cv2\n", "try:\n    import cv2\nexcept ImportError:   # only needed for VLM crops\n    cv2 = None\n", s, count=1, flags=re.M)
    s, b = re.subn(r"(    def make_crop\(self, t, floor_z\):\n)", r"\1        if cv2 is None:\n            return None\n", s, count=1)
    if (a, b) != (1, 1):
        sys.exit(f"ERROR cv2: matches {(a, b)}")
    done.append("cv2 optional")
open(p, "w").write(s)
print("applied:", ", ".join(done) or "nothing (already up to date)")
