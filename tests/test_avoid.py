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
    ramp.v = 0.3
    v, _ = ramp.step(0.0, 0.0, 0.1)
    assert v == pytest.approx(0.3 - config.max_decel * 0.1)


def test_remembers_an_obstacle_after_it_leaves_the_view():
    config = Config()
    avoider = Avoider(config)
    # a post ahead and to the left, clear of the corridor, 0.5 m from the camera
    ranges = np.full(BEAMS, np.inf)
    ranges[np.abs(angles() - 0.35) < 0.02] = 0.5
    avoider.step(scan(ranges))
    # drive 0.4 m forward: the post is now beside the robot, out of the view,
    # which (like Ster-Vis's left edge) has no depth on that side
    ranges = np.full(BEAMS, np.inf)
    ranges[angles() > 0.45] = np.nan
    command = avoider.step(scan(ranges), dt=1.0, v_prev=0.4, w_prev=0.0)
    assert len(avoider.memory.xy) > 0
    assert command.w < 0                       # still steering away from it


def test_current_view_replaces_memory():
    config = Config()
    avoider = Avoider(config)
    ranges = np.full(BEAMS, np.inf)
    ranges[np.abs(angles()) < 0.05] = 0.6
    avoider.step(scan(ranges))
    assert len(avoider.memory.xy) > 0
    # the same bearings now see nothing in range: the old points go
    avoider.step(scan(np.full(BEAMS, np.inf)), dt=0.1)
    assert len(avoider.memory.xy) == 0


def test_turns_the_way_its_corners_are_free():
    config = Config()
    avoider = Avoider(config)
    avoider.memory.xy = np.array([[-0.06, 0.104]])     # just beside the left side, towards the rear
    avoider.memory.age = np.array([0.5])
    wall = 0.25 / np.cos(angles())
    command = avoider.step(scan(wall))
    # turning right swings that side into it; turning left swings it away
    assert command.state == "turn" and command.w > 0
