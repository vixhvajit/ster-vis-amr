"""Four BO motors on two L298N boards, driven as a skid steer.

There are no encoders, so this is open loop: a wheel speed becomes a PWM duty
in proportion to the motor's no-load speed, under the voltage cap. Real speed
depends on the battery, the load and the floor.
"""

from __future__ import annotations

from .config import Config

EPS = 1e-3


def mix(v: float, w: float, config: Config) -> tuple[float, float]:
    """Body twist -> (left, right) wheel speeds in m/s, scaled down together if
    either is beyond what the motors can do, so the turn radius is kept."""
    half = w * config.track_m / 2 * config.turn_gain
    left, right = v - half, v + half
    peak = max(abs(left), abs(right))
    limit = config.wheel_max_speed
    if peak > limit:
        left, right = left * limit / peak, right * limit / peak
    return left, right


def duty(speed: float, config: Config) -> float:
    """Wheel speed in m/s -> signed PWM duty, -duty_cap..duty_cap."""
    if abs(speed) < EPS:
        return 0.0
    fraction = min(1.0, abs(speed) / config.wheel_max_speed)
    fraction = config.min_duty_fraction + (1.0 - config.min_duty_fraction) * fraction
    return (1.0 if speed > 0 else -1.0) * config.duty_cap * fraction


class L298NDrive:
    """The real motors, through gpiozero (lgpio on the Pi 5)."""

    def __init__(self, config: Config) -> None:
        from gpiozero import Motor   # Pi only; imported here so the rest runs anywhere

        self.config = config
        self.motors = {}
        for name, pins in config.motors.items():
            motor = Motor(pins.forward, pins.backward, enable=pins.enable, pwm=True)
            for device in (motor.forward_device, motor.backward_device):
                device.frequency = config.pwm_hz
            self.motors[name] = motor

    def set_duty(self, name: str, value: float) -> None:
        pins = self.config.motors[name]
        value = max(-self.config.duty_cap, min(self.config.duty_cap, value))
        self.motors[name].value = -value if pins.invert else value

    def send(self, v: float, w: float) -> None:
        left, right = mix(v, w, self.config)
        dl, dr = duty(left, self.config), duty(right, self.config)
        for name in self.motors:
            self.set_duty(name, dl if name.endswith("left") else dr)

    def stop(self) -> None:
        for motor in self.motors.values():
            motor.stop()

    def close(self) -> None:
        self.stop()
        for motor in self.motors.values():
            motor.close()


class PrintDrive:
    """No motors: print what would be sent. For trying the loop on the Pi safely."""

    def __init__(self, config: Config) -> None:
        self.config = config

    def send(self, v: float, w: float) -> None:
        left, right = mix(v, w, self.config)
        print(f"  left {duty(left, self.config):+.2f}  right {duty(right, self.config):+.2f}")

    def stop(self) -> None:
        pass

    def close(self) -> None:
        pass
