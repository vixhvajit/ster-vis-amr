"""Check the Ster-Vis test AMR in Gazebo: it loads, sits level, drives, and its
stereo pair has the right geometry.

Start the server first (.\\start-gz.ps1), then, in the gz-env shell:

    python check_model.py            # writes output/check/

It saves pictures from the fixed view cameras and both robot cameras, then:

- rest: the robot has settled on its wheels, level, at floor height;
- stereo: the red target's front face is 0.95 m ahead; its disparity between
  the left and right images gives its depth, which must match;
- drive: 0.3 m/s forward for 2 s, then 1.5 rad/s in place for 2 s, measured
  against the true pose from the world.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from gz.msgs10.camera_info_pb2 import CameraInfo
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node

HERE = Path(__file__).parent
WORLD, ROBOT = "amr_test", "amr"
TARGET_FACE_X = 0.95          # red target: centre 1.0 m, 0.10 m deep
VIEWS = ("view_iso", "view_side", "view_scene")


class Mailbox:
    """Latest message on each topic; gz-transport callbacks run on their own threads."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.images: dict[str, np.ndarray] = {}
        self.info: CameraInfo | None = None
        self.pose: tuple[float, ...] | None = None     # t, x, y, z, roll, pitch, yaw
        self.poses: list[tuple[float, ...]] = []

    def on_image(self, name: str):
        def callback(msg: Image) -> None:
            image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            with self.lock:
                self.images[name] = image
        return callback

    def on_info(self, msg: CameraInfo) -> None:
        with self.lock:
            self.info = msg

    def on_poses(self, msg: Pose_V) -> None:
        for p in msg.pose:
            if p.name == ROBOT:
                q = p.orientation
                roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
                pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
                pose = (t, p.position.x, p.position.y, p.position.z, roll, pitch, yaw)
                with self.lock:
                    self.pose = pose
                    self.poses.append(pose)
                return


def camera_poses() -> list[tuple[float, float, float]]:
    """Left and right camera positions in the robot frame, from the model."""
    sdf = (HERE / "models" / "ster_vis_test_amr" / "model.sdf").read_text(encoding="utf-8")
    return [tuple(float(v) for v in re.search(
        rf'<sensor name="{side}_camera".*?<pose>(\S+) (\S+) (\S+)', sdf, re.S).groups())
        for side in ("left", "right")]


def wait_for(what, timeout: float, label: str):
    end = time.time() + timeout
    while time.time() < end:
        value = what()
        if value:
            return value
        time.sleep(0.05)
    raise SystemExit(f"timed out waiting for {label}: is the server running (.\\start-gz.ps1)?")


def sim_now(box: Mailbox) -> float:
    with box.lock:
        return box.pose[0]


def drive(box: Mailbox, pub, linear: float, angular: float, seconds: float) -> list[tuple[float, ...]]:
    """Command a twist for `seconds` of sim time; return the poses seen meanwhile."""
    with box.lock:
        box.poses.clear()
    start = sim_now(box)
    twist = Twist()
    twist.linear.x, twist.angular.z = linear, angular
    while sim_now(box) - start < seconds:
        pub.publish(twist)
        time.sleep(0.05)
    pub.publish(Twist())
    stop = sim_now(box)
    while sim_now(box) - stop < 1.0:       # let it come to rest
        pub.publish(Twist())
        time.sleep(0.05)
    with box.lock:
        return list(box.poses)


def red_mask_edges(image_rgb: np.ndarray) -> tuple[float, float, np.ndarray]:
    """Left and right image columns of the red target, from a colour mask."""
    r, g, b = (image_rgb[..., i].astype(np.float32) for i in range(3))
    mask = ((r > 25) & (r > 1.8 * g) & (r > 1.8 * b)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count < 2:
        raise SystemExit("red target not found in the camera image")
    biggest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    blob = labels == biggest
    rows = np.where(blob.any(axis=1))[0]
    middle = blob[rows[len(rows) // 4]: rows[3 * len(rows) // 4]]      # skip the ragged top and bottom
    cols = [np.where(row)[0] for row in middle if row.any()]
    return float(np.median([c[0] for c in cols])), float(np.median([c[-1] for c in cols])), blob


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out", type=Path, default=HERE / "output" / "check")
    parser.add_argument("--no-drive", action="store_true", help="pictures and stereo check only")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    box = Mailbox()
    node = Node()
    for side in ("left", "right"):
        node.subscribe(Image, f"/amr/stereo/{side}/image", box.on_image(side))
    node.subscribe(CameraInfo, "/amr/stereo/left/camera_info", box.on_info)
    for view in VIEWS:
        node.subscribe(Image, f"/view/{view}/image", box.on_image(view))
    node.subscribe(Pose_V, f"/world/{WORLD}/pose/info", box.on_poses)
    cmd = node.advertise("/amr/cmd_vel", Twist)

    wait_for(lambda: box.pose, 60, "the robot's pose")
    wait_for(lambda: all(k in box.images for k in ("left", "right", *VIEWS)) and box.info, 90, "camera images")
    # settle: 2 s of sim time after the first pose
    t0 = sim_now(box)
    wait_for(lambda: sim_now(box) - t0 > 2.0, 120, "2 s of sim time")
    time.sleep(0.5)
    results: dict = {}

    with box.lock:
        _, x, y, z, roll, pitch, yaw = box.pose
        images = {k: v.copy() for k, v in box.images.items()}
        info = box.info
    results["rest"] = {"x": round(x, 4), "y": round(y, 4), "z_m": round(z, 4),
                       "roll_deg": round(math.degrees(roll), 2), "pitch_deg": round(math.degrees(pitch), 2)}
    for name, image in images.items():
        cv2.imwrite(str(args.out / f"{name}.png"), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))

    # Stereo geometry: depth of the red target's face from its disparity.
    fx = info.intrinsics.k[0]
    left_l, left_r, left_blob = red_mask_edges(images["left"])
    right_l, right_r, _ = red_mask_edges(images["right"])
    disparity = ((left_l - right_l) + (left_r - right_r)) / 2
    (cam_x, cam_y, _), (_, right_y, _) = camera_poses()
    baseline = cam_y - right_y
    true_depth = TARGET_FACE_X - (x + cam_x)
    depth = fx * baseline / disparity
    results["stereo"] = {"fx_px": round(fx, 1), "image": [info.width, info.height],
                         "disparity_px": round(disparity, 2), "depth_m": round(depth, 4),
                         "true_depth_m": round(true_depth, 4), "error_mm": round((depth - true_depth) * 1000, 1)}
    pair = np.hstack([images["left"], images["right"]])
    cv2.imwrite(str(args.out / "stereo_pair.png"), cv2.cvtColor(pair, cv2.COLOR_RGB2BGR))

    if not args.no_drive:
        poses = drive(box, cmd, 0.3, 0.0, 2.0)
        a, b = poses[0], poses[-1]
        results["drive_forward"] = {
            "commanded_m": 0.6, "moved_m": round(math.hypot(b[1] - a[1], b[2] - a[2]), 3),
            "sideways_drift_m": round(abs(b[2] - a[2]), 3),
            "max_abs_pitch_deg": round(max(abs(math.degrees(p[5])) for p in poses), 2),
            "max_z_m": round(max(p[3] for p in poses), 4)}
        poses = drive(box, cmd, 0.0, 1.5, 2.0)
        yaw_turned = sum((math.remainder(q[6] - p[6], math.tau) for p, q in zip(poses, poses[1:])))
        a, b = poses[0], poses[-1]
        results["turn_in_place"] = {
            "commanded_rad": 3.0, "turned_rad": round(yaw_turned, 3),
            "centre_moved_m": round(math.hypot(b[1] - a[1], b[2] - a[2]), 3),
            "max_abs_roll_deg": round(max(abs(math.degrees(p[4])) for p in poses), 2)}
        time.sleep(0.6)
        with box.lock:
            after = box.images["view_scene"].copy()
        cv2.imwrite(str(args.out / "view_scene_after_drive.png"), cv2.cvtColor(after, cv2.COLOR_RGB2BGR))

    (args.out / "summary.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
    print(json.dumps(results, indent=2))
    print(f"pictures in {args.out}")


if __name__ == "__main__":
    main()
