"""Depth from the Ster-Vis service running on the same Pi, over its HTTP API.

Ster-Vis runs as its own service (`ster-vis depth --live --headless --stream
8080`); this reads its latest frame. Two requests per frame: /api/v1/frame for
the sequence number and timestamp, then /api/v1/depth.npy.
"""

from __future__ import annotations

import io
import json
import time
import urllib.error
import urllib.request

import numpy as np

from .config import Config
from .scan import Scan, scan_from_depth


class StervisSource:
    def __init__(self, config: Config) -> None:
        from stereo_vision.outputs import CameraModel

        self.config = config
        self.base = config.stervis_url.rstrip("/") + "/api/v1/"
        self.last_sequence = None
        info = self._wait_for_info()
        cam = info["camera"]
        self.camera = CameraModel(cam["width"], cam["height"], cam["fx"], cam["fy"], cam["cx"], cam["cy"],
                                  cam["baseline_m"])
        self.closest_m = info.get("closest_m")

    def _get(self, name: str, timeout: float = 1.0) -> bytes:
        with urllib.request.urlopen(self.base + name, timeout=timeout) as reply:
            return reply.read()

    def _wait_for_info(self, seconds: float = 60.0) -> dict:
        end = time.time() + seconds
        while True:
            try:
                return json.loads(self._get("info"))
            except (urllib.error.URLError, OSError, ValueError) as error:
                if time.time() > end:
                    raise SystemExit(f"no Ster-Vis at {self.config.stervis_url} ({error}); is the ster-vis "
                                     "service running with --stream?") from None
                time.sleep(1.0)

    def next(self, timeout: float) -> Scan | None:
        """The next new frame's scan, or None if none arrives in time (or it is stale)."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                frame = json.loads(self._get("frame", timeout=timeout))
                if frame["sequence"] != self.last_sequence:
                    depth = np.load(io.BytesIO(self._get("depth.npy", timeout=timeout)))
                    self.last_sequence = frame["sequence"]
                    # Ster-Vis stamps frames with the Pi's clock at capture.
                    if time.time() - frame["timestamp"] > self.config.scan_timeout_s:
                        return None
                    return scan_from_depth(depth, self.camera, self.config, frame["timestamp"])
            except (urllib.error.URLError, OSError, ValueError, KeyError):
                pass    # 503 before the first frame, or the service restarting
            time.sleep(0.01)
        return None

    def close(self) -> None:
        pass
