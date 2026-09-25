"""amr: drive the Ster-Vis test AMR.

    amr run                     on the Pi: Ster-Vis depth -> avoidance -> motors
    amr run --dry-run           same, but print motor duties instead of driving
    amr run --sim               on the Gazebo model (sim/), from this PC
    amr motor-test              spin each motor in turn, to check the wiring
    amr drive V W SECONDS       open-loop drive, then stop
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from . import __version__
from .config import MOTOR_NAMES, load

DEFAULT_CONFIG = Path("/etc/amr/amr.toml")


def config_from(args):
    path = args.config
    if path is None and DEFAULT_CONFIG.exists():
        path = DEFAULT_CONFIG
    return load(path)


def cmd_run(args) -> None:
    from .control import run

    config = config_from(args)
    truth = None
    if args.sim:
        from .sim import GazeboDrive, GazeboSource, TruthPose

        source, drive = GazeboSource(config, rate_hz=args.rate), GazeboDrive(config)
        truth = TruthPose(args.world)
    else:
        from .motors import L298NDrive, PrintDrive
        from .stervis import StervisSource

        source = StervisSource(config)
        drive = PrintDrive(config) if args.dry_run else L298NDrive(config)
    print(f"amr {__version__}: {'sim' if args.sim else 'dry run' if args.dry_run else 'motors live'}; "
          f"Ster-Vis sees from {source.closest_m:.2f} m; stop at {config.stop_m:.2f} m; "
          f"max {config.max_speed:.2f} m/s; duty cap {config.duty_cap:.2f}", flush=True)
    stats = run(source, drive, config, args.duration, args.log, truth, quiet=args.quiet)
    print(json.dumps(stats), flush=True)


def cmd_motor_test(args) -> None:
    from .motors import L298NDrive

    config = config_from(args)
    drive = L298NDrive(config)
    duty = args.duty * config.duty_cap
    try:
        for name in MOTOR_NAMES:
            for label, sign in (("forward", 1), ("backward", -1)):
                print(f"{name}: {label} (duty {sign * duty:+.2f})", flush=True)
                drive.set_duty(name, sign * duty)
                time.sleep(args.seconds)
                drive.stop()
                time.sleep(0.5)
        print("Each wheel should have turned forward (robot would roll ahead), then back. "
              "If one turned the wrong way, set invert = true for it in the config.")
    finally:
        drive.close()


def cmd_drive(args) -> None:
    from .motors import L298NDrive

    config = config_from(args)
    drive = L298NDrive(config)
    try:
        drive.send(args.v, args.w)
        time.sleep(args.seconds)
    finally:
        drive.close()


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(prog="amr", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--version", action="version", version=f"amr {__version__}")
    parser.add_argument("--config", type=Path, default=None, help=f"TOML settings (default {DEFAULT_CONFIG} if present)")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("run", help="drive and avoid obstacles")
    p.add_argument("--sim", action="store_true", help="drive the Gazebo model instead of the motors")
    p.add_argument("--world", default="amr_arena", help="Gazebo world name, for the true pose (--sim)")
    p.add_argument("--rate", type=float, default=0.0,
                   help="--sim: process at most this many frames per second, like a slower Pi (default: all)")
    p.add_argument("--dry-run", action="store_true", help="print motor duties, do not drive")
    p.add_argument("--duration", type=float, default=None, help="seconds to drive (default: until stopped)")
    p.add_argument("--log", type=Path, default=None, help="write a CSV log here")
    p.add_argument("--quiet", action="store_true")
    p.set_defaults(func=cmd_run)

    p = sub.add_parser("motor-test", help="spin each motor forward then backward")
    p.add_argument("--duty", type=float, default=0.6, help="fraction of the duty cap (default 0.6)")
    p.add_argument("--seconds", type=float, default=1.0)
    p.set_defaults(func=cmd_motor_test)

    p = sub.add_parser("drive", help="open-loop drive for a while, then stop")
    p.add_argument("v", type=float, help="m/s forward")
    p.add_argument("w", type=float, help="rad/s, positive = left")
    p.add_argument("seconds", type=float)
    p.set_defaults(func=cmd_drive)

    args = parser.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
