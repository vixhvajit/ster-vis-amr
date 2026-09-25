"""Reactive obstacle avoidance on the stereo laser scan, with a short memory.

Adapted from the avoider Ster-Vis validated in Gazebo (sim/gazebo/avoid.py in
the Ster-Vis repo), resized for this 0.2 m robot and made cautious about what
the cameras cannot see:

- Look down a corridor as wide as the robot plus a margin; the nearest hit in
  it, measured from the robot's front, sets the speed.
- Obstacles to either side push the heading away, harder the closer they are
  to the robot's outline. Arcs stay gentle (radius 0.8 m or more): a tight arc
  would sweep the inner front corner through space the cameras never saw, so
  bigger heading changes are made as turns in place.
- Too close ahead: stop and turn in place, and keep turning that way until the
  corridor is open (committing stops dithering). The turn goes the way the
  robot can rotate furthest without its corners touching anything it
  remembers, and otherwise towards the clearer side.
- Mostly no depth straight ahead (a blank wall, a lens covered, the camera
  too close to something to match it): treat it as blocked. Ster-Vis cannot
  match nearer than its ``closest_m``, so something that close reads as
  unknown, not clear.

The memory is what lets it pass things safely. The cameras see 66 degrees
ahead, and Ster-Vis has no depth in the leftmost ~17% of the left image (the
right image does not reach that far), so an obstacle leaves the view well
before the robot has passed it. Points seen in the last few seconds (up to 8 s, while within 1 m) are kept,
carried along by dead reckoning on the commanded motion, and used wherever the
cameras cannot currently see. Where they can, the current scan replaces them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .config import Config
from .scan import Scan

AHEAD_RAD = 0.25       # the "straight ahead" cone used for the blind check, +-14 deg
MEMORY_S = 8.0         # an obstacle point out of view is remembered this long (while within reach)
MEMORY_REACH = 1.0     # only points this close to the robot are kept
PUSH_GAIN = 0.06       # rad/s per 1/m of nearness, for obstacles beside the robot
SWING_STEP = math.radians(5)
MIN_ARC_R = 0.8        # m; tighter arcs sweep the inner front corner through space the cameras never saw
SWING_CLEAR = 0.02     # a turn in place may bring nothing nearer the outline than this


@dataclass
class Command:
    v: float            # m/s, forward
    w: float            # rad/s, positive = turn left
    state: str          # cruise | turn | blind
    front_m: float      # clear distance ahead of the robot's front


class Memory:
    """Obstacle points (x, y in base_link) and how long ago each was seen."""

    def __init__(self) -> None:
        self.xy = np.empty((0, 2))
        self.age = np.empty(0)

    def advance(self, v: float, w: float, dt: float) -> None:
        """The robot drove at (v, w) for dt: move the points the other way."""
        if dt <= 0 or not len(self.xy):
            return
        th = w * dt
        p = self.xy - [v * dt * math.cos(th / 2), v * dt * math.sin(th / 2)]
        c, s = math.cos(th), math.sin(th)
        self.xy = np.column_stack([c * p[:, 0] + s * p[:, 1], -s * p[:, 0] + c * p[:, 1]])
        self.age = self.age + dt

    def update(self, scan: Scan, config: Config, now_xy: np.ndarray) -> None:
        """Drop what the current scan can see (it is the authority there), what
        is old or far, and add what the scan sees now."""
        keep = self.age < MEMORY_S
        if len(self.xy):
            rel = self.xy - [config.camera_x_m, config.camera_y_m]
            bearing = np.arctan2(rel[:, 1], rel[:, 0])
            beam = np.rint((bearing - scan.angle_min) / scan.angle_increment).astype(int)
            in_view = (beam >= 0) & (beam < len(scan.ranges)) & (rel[:, 0] > 0)
            beam = np.clip(beam, 0, len(scan.ranges) - 1)
            seen_there = in_view & ~np.isnan(scan.ranges[beam])
            reach = np.hypot(np.maximum(np.abs(self.xy[:, 0]) - config.front_m, 0),
                             np.maximum(np.abs(self.xy[:, 1]) - config.half_width_m, 0))
            keep &= ~seen_there & (reach < MEMORY_REACH)
        self.xy = np.vstack([self.xy[keep], now_xy])
        self.age = np.concatenate([self.age[keep], np.zeros(len(now_xy))])

    def clear(self) -> None:
        self.__init__()


class Avoider:
    def __init__(self, config: Config) -> None:
        self.c = config
        self.turn_dir = 0
        self.memory = Memory()

    def _gap(self, pts: np.ndarray) -> np.ndarray:
        """Distance from each point to the robot's outline (0 if inside it)."""
        c = self.c
        return np.hypot(np.maximum(np.abs(pts[:, 0]) - c.front_m, 0),
                        np.maximum(np.abs(pts[:, 1]) - c.half_width_m, 0))

    def _free_turn(self, pts: np.ndarray, direction: int, limit: float = math.pi / 2) -> float:
        """How far the robot can turn in place in ``direction`` before its
        outline comes within SWING_CLEAR of one of ``pts`` (up to ``limit``).
        Something already that close only counts if the turn brings it closer.
        Skid steer drifts as it turns, so the reach checked is generous."""
        c = self.c
        near = pts[np.hypot(pts[:, 0], pts[:, 1]) < math.hypot(c.front_m, c.half_width_m) + 0.12]
        if not len(near):
            return limit
        floor = min(SWING_CLEAR, float(self._gap(near).min()) - 0.005)
        a = 0.0
        while a < limit:
            a += SWING_STEP
            # turning the robot by +a is turning the points by -a
            ca, sa = math.cos(direction * a), math.sin(direction * a)
            rotated = np.column_stack([ca * near[:, 0] + sa * near[:, 1], -sa * near[:, 0] + ca * near[:, 1]])
            if float(self._gap(rotated).min()) < floor:
                return a - SWING_STEP
        return limit

    def step(self, scan: Scan, dt: float = 0.0, v_prev: float = 0.0, w_prev: float = 0.0) -> Command:
        """``dt``, ``v_prev`` and ``w_prev``: the time since the last scan and
        the motion commanded over it, for carrying the memory along."""
        c = self.c
        angles = scan.angles
        seen = ~np.isnan(scan.ranges)
        hit = seen & np.isfinite(scan.ranges)
        r = np.where(np.isinf(scan.ranges), scan.range_max, scan.ranges)
        now = np.column_stack([c.camera_x_m + r[hit] * np.cos(angles[hit]),
                               c.camera_y_m + r[hit] * np.sin(angles[hit])])
        self.memory.advance(v_prev, w_prev, dt)
        self.memory.update(scan, c, now)
        pts = self.memory.xy

        corridor = (np.abs(pts[:, 1]) < c.half_width_m + c.side_margin_m) & (pts[:, 0] > c.front_m - 0.01)
        front = float(pts[corridor, 0].min() - c.front_m) if corridor.any() else scan.range_max

        def clearance(mask) -> float:
            return float(np.mean(r[mask])) if mask.any() else 0.0
        left, right = clearance(seen & (angles > 0.1)), clearance(seen & (angles < -0.1))

        ahead = np.abs(angles) < AHEAD_RAD
        blind = ahead.any() and float(np.mean(~seen[ahead])) > c.blind_fraction

        if self.turn_dir:
            if front > c.go_m and not blind:
                self.turn_dir = 0
            else:
                if self._free_turn(pts, self.turn_dir, 2 * SWING_STEP) < 2 * SWING_STEP:
                    self.turn_dir = -self.turn_dir        # a corner is about to touch: go the other way
                return Command(0.0, self.turn_dir * c.max_turn, "turn", front)
        if front < c.stop_m or blind:
            free_l, free_r = self._free_turn(pts, 1), self._free_turn(pts, -1)
            if abs(free_l - free_r) > 1e-6:
                self.turn_dir = 1 if free_l > free_r else -1
            else:
                self.turn_dir = 1 if left >= right else -1
            return Command(0.0, self.turn_dir * c.max_turn, "blind" if blind else "turn", front)

        v = c.max_speed * float(np.clip((front - c.stop_m) / (c.slow_m - c.stop_m), 0.3, 1.0))
        # Steer away from the nearest thing on each side (left pushes right).
        gap = self._gap(pts)
        side = pts[:, 0] > -c.front_m - 0.05

        def nearness(mask) -> float:
            d = gap[mask & side]
            return max(0.0, 1.0 / max(float(d.min()), 0.02) - 1.0 / c.influence_m) if d.size else 0.0
        w = PUSH_GAIN * (nearness(pts[:, 1] < 0) - nearness(pts[:, 1] > 0))
        if front < c.slow_m and abs(left - right) > 0.3:
            w += 0.25 * np.sign(left - right)
        beside = side & (pts[:, 0] < c.front_m + 0.02) & (gap < c.side_margin_m)
        if beside.any():
            v = min(v, 0.5 * c.max_speed)                # something alongside: ease off while steering away
        # Only gentle arcs while driving: bigger heading changes happen as turns
        # in place, which sweep the view round before the robot moves into it.
        w_cap = min(c.max_turn, max(0.15, v / MIN_ARC_R))
        return Command(v, float(np.clip(w, -w_cap, w_cap)), "cruise", front)


class Ramp:
    """Acceleration limits, so the motors do not brown out the Pi or tip the robot."""

    def __init__(self, config: Config) -> None:
        self.c = config
        self.v = self.w = 0.0

    def step(self, v: float, w: float, dt: float) -> tuple[float, float]:
        dt = max(0.0, min(dt, 0.5))
        # braking may be harder than speeding up: stopping short is the safe side
        self.v += float(np.clip(v - self.v, -self.c.max_decel * dt, self.c.max_accel * dt))
        self.w += float(np.clip(w - self.w, -self.c.max_turn_accel * dt, self.c.max_turn_accel * dt))
        return self.v, self.w

    def reset(self) -> None:
        self.v = self.w = 0.0
