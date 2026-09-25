"""Reactive obstacle avoidance on the stereo laser scan.

Adapted from the avoider Ster-Vis validated in Gazebo (sim/gazebo/avoid.py in
the Ster-Vis repo), resized for this 0.2 m robot and made cautious about what
the cameras cannot see:

- Look down a corridor as wide as the robot plus a margin; the nearest hit in
  it, measured from the robot's front, sets the speed.
- Obstacles to either side, within ``influence_m``, push the heading away.
- Too close ahead: stop and turn in place towards the clearer side, and keep
  turning that way until the corridor is open (committing stops dithering).
- Mostly no depth straight ahead (a blank wall, a lens covered, the camera
  too close to something to match it): treat it as blocked. Ster-Vis cannot
  match nearer than its ``closest_m``, so something that close reads as
  unknown, not clear.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import Config
from .scan import Scan

AHEAD_RAD = 0.25    # the "straight ahead" cone used for the blind check, +-14 deg


@dataclass
class Command:
    v: float            # m/s, forward
    w: float            # rad/s, positive = turn left
    state: str          # cruise | turn | blind
    front_m: float      # clear distance ahead of the robot's front


class Avoider:
    def __init__(self, config: Config) -> None:
        self.c = config
        self.turn_dir = 0

    def step(self, scan: Scan) -> Command:
        c = self.c
        angles = scan.angles
        seen = ~np.isnan(scan.ranges)
        r = np.where(np.isinf(scan.ranges), scan.range_max, scan.ranges)
        # scan points in base_link
        x = c.camera_x_m + r * np.cos(angles)
        y = c.camera_y_m + r * np.sin(angles)
        corridor = seen & (np.abs(y) < c.half_width_m + c.side_margin_m) & (x > 0)
        front = float(x[corridor].min() - c.front_m) if corridor.any() else scan.range_max

        def clearance(mask) -> float:
            return float(np.mean(r[mask])) if mask.any() else 0.0
        left, right = clearance(seen & (angles > 0.1)), clearance(seen & (angles < -0.1))

        ahead = np.abs(angles) < AHEAD_RAD
        blind = ahead.any() and float(np.mean(~seen[ahead])) > c.blind_fraction

        if self.turn_dir:
            if front > c.go_m and not blind:
                self.turn_dir = 0
            else:
                return Command(0.0, self.turn_dir * c.max_turn, "turn", front)
        if front < c.stop_m or blind:
            self.turn_dir = 1 if left >= right else -1
            return Command(0.0, self.turn_dir * c.max_turn, "blind" if blind else "turn", front)

        v = c.max_speed * float(np.clip((front - c.stop_m) / (c.slow_m - c.stop_m), 0.3, 1.0))
        # Left obstacles (positive bearing) push the heading right (negative w).
        near = seen & (r < c.influence_m)
        push = float(np.sum(-np.sign(angles[near]) * (1.0 / r[near] - 1.0 / c.influence_m)))
        w = 0.05 * push
        if front < c.slow_m:
            w += 0.4 * np.sign(left - right)
        return Command(v, float(np.clip(w, -c.max_turn, c.max_turn)), "cruise", front)


class Ramp:
    """Acceleration limits, so the motors do not brown out the Pi or tip the robot."""

    def __init__(self, config: Config) -> None:
        self.c = config
        self.v = self.w = 0.0

    def step(self, v: float, w: float, dt: float) -> tuple[float, float]:
        dt = max(0.0, min(dt, 0.5))
        self.v += float(np.clip(v - self.v, -self.c.max_accel * dt, self.c.max_accel * dt))
        self.w += float(np.clip(w - self.w, -self.c.max_turn_accel * dt, self.c.max_turn_accel * dt))
        return self.v, self.w

    def reset(self) -> None:
        self.v = self.w = 0.0
