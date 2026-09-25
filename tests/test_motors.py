import math

import pytest

from amr.config import Config
from amr.motors import duty, mix


def test_duty_cap_protects_6v_motors_on_3s():
    config = Config()
    # 6 V motor, 12.6 V pack, 2 V driver drop
    assert config.duty_cap == pytest.approx(6.0 / 10.6)
    assert duty(100.0, config) == pytest.approx(config.duty_cap)
    assert duty(-100.0, config) == pytest.approx(-config.duty_cap)


def test_duty_zero_and_minimum():
    config = Config()
    assert duty(0.0, config) == 0.0
    tiny = duty(0.01, config)
    assert tiny >= config.min_duty_fraction * config.duty_cap   # enough to overcome stall
    assert duty(0.2, config) < duty(0.4, config)


def test_mix_straight_and_turn():
    config = Config()
    assert mix(0.3, 0.0, config) == pytest.approx((0.3, 0.3))
    left, right = mix(0.0, 1.0, config)
    assert left == pytest.approx(-right)
    assert right == pytest.approx(config.track_m / 2 * config.turn_gain)
    assert right > 0      # positive w turns left: right wheel forward


def test_mix_keeps_turn_radius_when_saturated():
    config = Config()
    left, right = mix(2.0, 3.0, config)
    assert max(abs(left), abs(right)) == pytest.approx(config.wheel_max_speed)
    l0, r0 = 2.0 - 3.0 * config.track_m / 2 * config.turn_gain, 2.0 + 3.0 * config.track_m / 2 * config.turn_gain
    assert left / right == pytest.approx(l0 / r0)


def test_wheel_max_speed():
    config = Config()
    assert config.wheel_max_speed == pytest.approx(200 * 2 * math.pi * 0.0325 / 60)
