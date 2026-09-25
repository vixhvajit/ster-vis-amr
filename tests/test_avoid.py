import math

import numpy as np
import pytest

from amr.avoid import Avoider, Ramp
from amr.config import Config
from amr.scan import Scan

HALF_FOV = math.radians(33)
BEAMS = 121


def scan(ranges) -> Scan:
    return Scan(-HALF_FOV, 2 * HALF_FOV / (BEAMS - 1), 2.5, np.asarray(ranges, np.float32))


def angles():
    return -HALF_FOV + np.arange(BEAMS) * 2 * HALF_FOV / (BEAMS - 1)


def test_open_space_cruises_at_full_speed():
    config = Config()
    command = Avoider(config).step(scan(np.full(BEAMS, np.inf)))
    assert command.state == "cruise"
    assert command.v == pytest.approx(config.max_speed)
    assert command.w == pytest.approx(0.0)


def test_wall_close_ahead_stops_and_turns():
    config = Config()
    wall = 0.25 / np.cos(angles())          # flat wall 0.25 m ahead of the camera
    command = Avoider(config).step(scan(wall))
    assert command.state == "turn"
    assert command.v == 0.0
    assert abs(command.w) == pytest.approx(config.max_turn)


def test_turn_commits_until_clear():
    config = Config()
    avoider = Avoider(config)
    ranges = np.full(BEAMS, np.inf)
    ranges[angles() < 0] = 0.3                # blocked ahead and right, open left
    ranges[np.abs(angles()) < 0.3] = 0.3
    first = avoider.step(scan(ranges))
    assert first.state == "turn" and first.w > 0      # towards the open left
    # a little less blocked but still under go_m: keeps turning the same way
    ranges[np.abs(angles()) < 0.3] = config.go_m
    assert avoider.step(scan(ranges)).w > 0
    assert avoider.step(scan(np.full(BEAMS, np.inf))).state == "cruise"


def test_unknown_ahead_is_not_clear():
    config = Config()
    ranges = np.full(BEAMS, np.inf)
    ranges[np.abs(angles()) < 0.3] = np.nan   # no depth straight ahead
    command = Avoider(config).step(scan(ranges))
    assert command.state == "blind"
    assert command.v == 0.0


def test_obstacle_on_left_steers_right():
    config = Config()
    ranges = np.full(BEAMS, np.inf)
    ranges[(angles() > 0.4) & (angles() < 0.55)] = 0.35
    command = Avoider(config).step(scan(ranges))
    assert command.state == "cruise"
    assert command.w < 0


def test_ramp_limits_acceleration():
    config = Config()
    ramp = Ramp(config)
    v, w = ramp.step(1.0, 10.0, 0.1)
    assert v == pytest.approx(config.max_accel * 0.1)
    assert w == pytest.approx(config.max_turn_accel * 0.1)
