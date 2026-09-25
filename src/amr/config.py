"""Robot settings: geometry, motors and wiring, avoidance.

Defaults match the minimal Ster-Vis test AMR (Ster-Vis repo, hardware/test-amr)
as designed: one printed deck, two L298Ns, the lenses about 7 cm up. Override any of them in a TOML file, by the same names:

    max_speed = 0.25
    stop_m = 0.4

    [motors.front_left]
    forward = 17
    backward = 27
    invert = true

Frame: base_link on the floor under the centre of the chassis, x forward,
y left, z up.
"""

from __future__ import annotations

import math
import tomllib
from dataclasses import dataclass, field, fields, replace
from pathlib import Path

MOTOR_NAMES = ("front_left", "rear_left", "front_right", "rear_right")


@dataclass(frozen=True)
class MotorPins:
    """One BO motor on one L298N channel (BCM GPIO numbers).

    ``forward`` and ``backward`` go to the channel's IN pins. ``enable`` is the
    channel's EN pin, or None when the board's EN jumper is left on. ``invert``
    flips the motor if it turns the wrong way (swap its leads, or set this).
    """

    forward: int
    backward: int
    enable: int | None = None
    invert: bool = False


def default_motors() -> dict[str, MotorPins]:
    # Left L298N: OUT1/OUT2 front-left, OUT3/OUT4 rear-left. Right L298N the same.
    return {
        "front_left": MotorPins(17, 27),
        "rear_left": MotorPins(22, 23),
        "front_right": MotorPins(5, 6),
        "rear_right": MotorPins(13, 19),
    }


@dataclass(frozen=True)
class Config:
    # -- geometry (hardware/test-amr/chassis.py) --------------------------------
    track_m: float = 0.166            # wheel centre to wheel centre
    wheel_radius_m: float = 0.0325    # 65 mm BO wheel
    half_width_m: float = 0.096       # centre line to the outside of a wheel
    front_m: float = 0.0945           # centre to the front of the front wheels
    camera_x_m: float = 0.0816        # left camera in base_link; scan origin
    camera_y_m: float = 0.030

    # -- power and motors ------------------------------------------------------
    # BO motors are rated 3-6 V. A full 3S pack (12.6 V) less the L298N's ~2 V
    # drop would put ~10.5 V on them, so duty is capped at motor_max_v / that.
    battery_full_v: float = 12.6
    driver_drop_v: float = 2.0
    motor_max_v: float = 6.0
    no_load_rpm: float = 200.0        # BO 1:48 at 6 V, typical; measure yours
    # Below about a third of their voltage BO motors stall; any non-zero
    # command gets at least this fraction of the capped duty.
    min_duty_fraction: float = 0.35
    # Skid steer scrubs sideways in turns, so it turns less than the wheel
    # speeds say. 1.3 is from the Gazebo model; tune on the real floor.
    turn_gain: float = 1.3
    pwm_hz: float = 1000.0
    motors: dict[str, MotorPins] = field(default_factory=default_motors)

    # -- motion limits ---------------------------------------------------------
    max_speed: float = 0.30           # m/s
    max_turn: float = 1.5             # rad/s
    max_accel: float = 0.8            # m/s^2
    max_decel: float = 2.0            # m/s^2; braking
    max_turn_accel: float = 4.0       # rad/s^2

    # -- avoidance (distances from the robot's front) ----------------------------
    stop_m: float = 0.30              # closer than this ahead: stop and turn
    slow_m: float = 0.90              # full speed beyond this
    go_m: float = 0.55                # after a turn, drive on once this is clear
    side_margin_m: float = 0.05       # added to half_width for the corridor
    influence_m: float = 0.6          # side obstacles nearer than this steer away
    blind_fraction: float = 0.6       # this much of the view ahead unknown: treat as blocked

    # -- Ster-Vis ---------------------------------------------------------------
    stervis_url: str = "http://127.0.0.1:8080"
    scan_timeout_s: float = 0.5       # no fresh depth for this long: stop
    # The scan band, relative to the camera (lens ~0.07 m up): from 3 cm above
    # the floor to 5 cm above the lens, which covers everything the robot can hit
    # (the Pi's cooler tops out about 9 cm up).
    scan_min_height_m: float = -0.04
    scan_max_height_m: float = 0.05
    scan_range_max_m: float = 2.5     # what a 60 mm baseline measures usefully
    scan_beams: int = 121

    @property
    def duty_cap(self) -> float:
        return min(1.0, self.motor_max_v / (self.battery_full_v - self.driver_drop_v))

    @property
    def wheel_max_speed(self) -> float:
        """Wheel rim speed at the duty cap, open loop."""
        return self.no_load_rpm * 2 * math.pi * self.wheel_radius_m / 60.0


def load(path: Path | None) -> Config:
    config = Config()
    if path is None:
        return config
    data = tomllib.loads(Path(path).read_text(encoding="utf-8"))
    known = {f.name for f in fields(Config)}
    unknown = set(data) - known
    if unknown:
        raise ValueError(f"{path}: unknown settings {', '.join(sorted(unknown))}")
    motors = dict(config.motors)
    for name, pins in data.pop("motors", {}).items():
        if name not in MOTOR_NAMES:
            raise ValueError(f"{path}: unknown motor {name!r}; use {', '.join(MOTOR_NAMES)}")
        motors[name] = replace(motors[name], **pins)
    return replace(config, motors=motors, **data)
