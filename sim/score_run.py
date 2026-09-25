"""Score an `amr run --sim --log run.csv` in the arena against the true geometry.

    python score_run.py output/run.csv          # prints a summary, writes run.png beside it

The log's x, y, yaw columns are the true pose from Gazebo (the controller never
uses them). For every logged pose the robot's footprint is checked against
every wall and obstacle in worlds/amr_arena.json:

- clearance: the smallest gap between the footprint and anything, over the run;
- contacts: poses where that gap was under 5 mm, i.e. touching or pushing.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).parent
# Footprint in base_link: the wheels are the widest and the furthest forward.
HALF_L, HALF_W = 0.0945, 0.096
CONTACT_M = 0.005


def footprint_outline(step: float = 0.01) -> np.ndarray:
    xs = np.arange(-HALF_L, HALF_L + 1e-9, step)
    ys = np.arange(-HALF_W, HALF_W + 1e-9, step)
    return np.vstack([np.c_[xs, np.full_like(xs, HALF_W)], np.c_[xs, np.full_like(xs, -HALF_W)],
                      np.c_[np.full_like(ys, HALF_L), ys], np.c_[np.full_like(ys, -HALF_L), ys]])


def distance_to(points: np.ndarray, shape: dict) -> np.ndarray:
    """Distance from each point to the shape (negative inside)."""
    d = points - [shape["x"], shape["y"]]
    if shape["kind"] == "cylinder":
        return np.hypot(d[:, 0], d[:, 1]) - shape["r"]
    c, s = math.cos(shape["yaw"]), math.sin(shape["yaw"])
    local = np.c_[c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]]
    q = np.abs(local) - [shape["sx"] / 2, shape["sy"] / 2]
    outside = np.hypot(np.maximum(q[:, 0], 0), np.maximum(q[:, 1], 0))
    return outside + np.minimum(np.maximum(q[:, 0], q[:, 1]), 0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("log", type=Path)
    parser.add_argument("--world", type=Path, default=HERE / "worlds" / "amr_arena.json")
    args = parser.parse_args()
    world = json.loads(args.world.read_text(encoding="utf-8"))
    rows = [r for r in csv.DictReader(open(args.log, newline="")) if r["x"]]
    poses = np.array([[float(r["x"]), float(r["y"]), float(r["yaw"])] for r in rows])
    outline = footprint_outline()

    gaps = np.empty(len(poses))
    nearest = []
    for i, (x, y, yaw) in enumerate(poses):
        c, s = math.cos(yaw), math.sin(yaw)
        pts = outline @ np.array([[c, s], [-s, c]]) + [x, y]
        best, name = math.inf, ""
        for shape in world["obstacles"]:
            d = float(distance_to(pts, shape).min())
            if d < best:
                best, name = d, shape["name"]
        gaps[i] = best
        nearest.append(name)
    contact = gaps < CONTACT_M
    # count separate touches, not frames
    touches = int(np.sum(contact[1:] & ~contact[:-1]) + (1 if len(contact) and contact[0] else 0))
    path = float(np.sum(np.hypot(*np.diff(poses[:, :2], axis=0).T))) if len(poses) > 1 else 0.0
    states = {}
    for r in rows:
        states[r["state"]] = states.get(r["state"], 0) + 1
    summary = {
        "frames": len(rows), "sim_s": float(rows[-1]["t"]) if rows else 0.0, "path_m": round(path, 2),
        "min_clearance_m": round(float(gaps.min()), 3) if len(gaps) else None,
        "min_clearance_to": nearest[int(np.argmin(gaps))] if len(gaps) else None,
        "contacts": touches, "contact_frames": int(contact.sum()),
        "touched": sorted({nearest[i] for i in np.where(contact)[0]}),
        "states": states,
    }
    print(json.dumps(summary, indent=2))

    # top-down picture: obstacles, path, touches
    size, arena = 800, world["arena"]
    scale = size / (arena + 0.4)

    def px(x, y):
        return int(round(size / 2 + x * scale)), int(round(size / 2 - y * scale))
    image = np.full((size, size, 3), 245, np.uint8)
    for shape in world["obstacles"]:
        if shape["kind"] == "cylinder":
            cv2.circle(image, px(shape["x"], shape["y"]), max(1, round(shape["r"] * scale)), (90, 90, 200), -1)
        else:
            corners = cv2.boxPoints(((shape["x"], shape["y"]), (shape["sx"], shape["sy"]), math.degrees(shape["yaw"])))
            cv2.fillPoly(image, [np.array([px(*p) for p in corners], np.int32)], (90, 90, 90) if
                         shape["name"].startswith("wall") else (90, 90, 200))
    if len(poses) > 1:
        cv2.polylines(image, [np.array([px(x, y) for x, y, _ in poses], np.int32)], False, (200, 120, 40), 2)
        for i in np.where(contact)[0]:
            cv2.circle(image, px(*poses[i, :2]), 6, (0, 0, 255), 2)
        cv2.circle(image, px(*poses[0, :2]), 7, (0, 160, 0), -1)
    cv2.putText(image, f"{summary['sim_s']:.0f} s, {path:.1f} m, min clearance {summary['min_clearance_m']} m, "
                f"contacts {touches}", (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
    out = args.log.with_suffix(".png")
    cv2.imwrite(str(out), image)
    print(f"picture: {out}")


if __name__ == "__main__":
    main()
