"""The same controller on the Gazebo model (sim/), through gz-transport.

GazeboSource runs Ster-Vis's depth pipeline on the simulated camera pair, with
the settings the Pi service uses (deploy/pi/ster-vis.env), so the controller
sees what it would see on the robot. The cameras are ideal, so the calibration
is built from their CameraInfo instead of a chessboard. GazeboDrive sends
cmd_vel to the model's DiffDrive. TruthPose reads the robot's true pose, for
scoring runs only; the controller never uses it.
"""

from __future__ import annotations

import math
import threading
import time

import cv2
import numpy as np

from .config import Config
from .scan import Scan, scan_from_depth

# The Ster-Vis service settings this mirrors (deploy/pi/ster-vis.env).
PRESET = "pi5"
MIN_DISTANCE_MM = 300.0


class GazeboSource:
    def __init__(self, config: Config, topic: str = "/amr/stereo", rate_hz: float = 0.0) -> None:
        from gz.msgs10.camera_info_pb2 import CameraInfo
        from gz.msgs10.image_pb2 import Image
        from gz.transport13 import Node
        from stereo_vision.live import DepthPipeline
        from stereo_vision.outputs import CameraModel
        from stereo_vision.presets import get_preset

        self.config = config
        self.lock = threading.Lock()
        self.images: dict[float, dict[str, np.ndarray]] = {}
        self.info = None
        self.last_stamp = -1.0
        # Process at most this many pairs per sim second, like a Pi that
        # cannot keep up with the cameras (0 = every pair).
        self.period = 1.0 / rate_hz if rate_hz > 0 else 0.0
        self.last_processed = -1e9
        self.node = Node()

        def on_image(side):
            def callback(msg) -> None:
                image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
                stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
                with self.lock:
                    self.images.setdefault(stamp, {})[side] = image
            return callback

        def on_info(msg) -> None:
            self.info = msg

        self.node.subscribe(Image, f"{topic}/left/image", on_image("left"))
        self.node.subscribe(Image, f"{topic}/right/image", on_image("right"))
        self.node.subscribe(CameraInfo, f"{topic}/left/camera_info", on_info)
        end = time.time() + 60
        while self.info is None:
            if time.time() > end:
                raise SystemExit(f"no {topic}/left/camera_info: is Gazebo running (sim/start-gz.ps1)?")
            time.sleep(0.1)
        baseline_mm = 2000.0 * config.camera_y_m
        self.pipeline = DepthPipeline(ideal_calibration(self.info, baseline_mm), get_preset(PRESET), MIN_DISTANCE_MM)
        self.camera = CameraModel.from_calibration(self.pipeline.calibration)
        self.closest_m = self.pipeline.closest_mm / 1000.0

    def _newest_pair(self):
        with self.lock:
            complete = [s for s, v in self.images.items() if len(v) == 2 and s > self.last_stamp]
            if not complete:
                return None
            stamp = max(complete)
            pair = self.images[stamp]
            for s in [s for s in self.images if s <= stamp]:
                del self.images[s]
            return stamp, pair["left"], pair["right"]

    def next(self, timeout: float) -> Scan | None:
        from stereo_vision.sources import StereoFrame

        end = time.monotonic() + timeout
        while time.monotonic() < end:
            pair = self._newest_pair()
            if pair is None:
                time.sleep(0.005)
                continue
            stamp, left, right = pair
            self.last_stamp = stamp
            if stamp - self.last_processed < self.period - 1e-6:
                # skipped for the rate cap, but the cameras are alive: that is
                # not what the watchdog is for, so restart its clock
                end = time.monotonic() + timeout
                continue
            self.last_processed = stamp
            frame = StereoFrame(cv2.cvtColor(left, cv2.COLOR_RGB2GRAY), cv2.cvtColor(right, cv2.COLOR_RGB2GRAY),
                                timestamp=stamp)
            result = self.pipeline.process(frame)
            depth_m = (result.depth_mm / 1000.0).astype(np.float32)
            return scan_from_depth(depth_m, self.camera, self.config, stamp)
        return None

    def close(self) -> None:
        pass


def ideal_calibration(info, baseline_mm: float):
    """What a perfect chessboard calibration of the simulated pair would return."""
    from stereo_vision.calibration import StereoCalibration

    K = np.array(info.intrinsics.k, np.float64).reshape(3, 3)
    size = (int(info.width), int(info.height))
    dist = np.zeros(5)
    R = np.eye(3)
    # OpenCV's T maps left-camera points into the right camera, which sits the
    # baseline to the right (+x in the optical frame).
    T = np.array([[-baseline_mm], [0.0], [0.0]])
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K, dist, K, dist, size, R, T, flags=cv2.CALIB_ZERO_DISPARITY,
                                                alpha=0)
    return StereoCalibration(size, K, dist, K.copy(), dist.copy(), R, T, R1, R2, P1, P2, Q, rms=0.0,
                             coverage_pct=100.0)


class GazeboDrive:
    def __init__(self, config: Config, topic: str = "/amr/cmd_vel") -> None:
        from gz.msgs10.twist_pb2 import Twist
        from gz.transport13 import Node

        self.Twist = Twist
        self.node = Node()
        self.pub = self.node.advertise(topic, Twist)
        time.sleep(0.3)     # let the publisher connect

    def send(self, v: float, w: float) -> None:
        twist = self.Twist()
        twist.linear.x, twist.angular.z = v, w
        self.pub.publish(twist)

    def stop(self) -> None:
        self.send(0.0, 0.0)

    def close(self) -> None:
        # DiffDrive keeps the last command, so say stop more than once.
        for _ in range(3):
            self.stop()
            time.sleep(0.05)


class TruthPose:
    """The robot's true pose from the world, for logging and scoring only."""

    def __init__(self, world: str, model: str = "amr") -> None:
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node

        self.model = model
        self.pose: tuple[float, float, float] | None = None
        self.node = Node()
        self.node.subscribe(Pose_V, f"/world/{world}/pose/info", self._on_poses)

    def _on_poses(self, msg) -> None:
        for p in msg.pose:
            if p.name == self.model:
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                self.pose = (p.position.x, p.position.y, yaw)
                return
