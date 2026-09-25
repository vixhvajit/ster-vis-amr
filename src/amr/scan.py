"""The laser scan the controller steers by, from a Ster-Vis depth map.

The scan is built here, with Ster-Vis's own ``laser_scan``, rather than read
from Ster-Vis's JSON ``/api/v1/scan``: that JSON writes both "nothing in range"
and "no depth" as null, and the avoider must tell them apart (clear ahead vs
blind ahead). The depth map keeps the difference.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import Config


@dataclass
class Scan:
    """Ranges from the left camera, bearing 0 ahead, positive to the left.

    inf = nothing within range_max; nan = no depth on that bearing.
    """

    angle_min: float
    angle_increment: float
    range_max: float
    ranges: np.ndarray
    stamp: float = 0.0

    @property
    def angles(self) -> np.ndarray:
        return self.angle_min + np.arange(len(self.ranges)) * self.angle_increment


def scan_from_depth(depth_m: np.ndarray, camera, config: Config, stamp: float = 0.0) -> Scan:
    """``camera`` is a Ster-Vis ``CameraModel`` for the (rectified) depth map."""
    from stereo_vision.outputs import laser_scan

    scan = laser_scan(depth_m, camera, config.scan_beams, config.scan_min_height_m, config.scan_max_height_m,
                      0.2, config.scan_range_max_m)
    return Scan(scan.angle_min, scan.angle_increment, scan.range_max, np.asarray(scan.ranges, np.float32), stamp)
