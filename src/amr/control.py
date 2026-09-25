"""The drive loop: depth -> scan -> avoider -> ramp -> motors, with a watchdog."""

from __future__ import annotations

import csv
import math
import signal
import time
from pathlib import Path

from .avoid import Avoider, Ramp
from .config import Config


def run(source, drive, config: Config, duration: float | None = None, log_path: Path | None = None,
        truth=None, quiet: bool = False) -> dict:
    """Drive until ``duration`` seconds of scan time pass, Ctrl+C or SIGTERM.

    The motors stop whenever no fresh scan arrives within scan_timeout_s,
    and always on the way out.
    """
    avoider, ramp = Avoider(config), Ramp(config)
    stopping = False

    def on_term(signum, frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, on_term)

    log_file = open(log_path, "w", newline="") if log_path else None
    log = csv.writer(log_file) if log_file else None
    if log:
        log.writerow(["t", "state", "front_m", "v_cmd", "w_cmd", "v", "w", "x", "y", "yaw"])
    stats = {"frames": 0, "timeouts": 0, "states": {}, "distance_m": 0.0}
    start = last = None
    last_xy = None
    try:
        while not stopping:
            scan = source.next(config.scan_timeout_s)
            if scan is None:
                # watchdog: no fresh depth, so do not keep driving on old data
                drive.stop()
                ramp.reset()
                stats["timeouts"] += 1
                if not quiet:
                    print("no fresh depth: stopped", flush=True)
                continue
            start = scan.stamp if start is None else start
            dt = 0.1 if last is None else scan.stamp - last
            last = scan.stamp
            if duration is not None and scan.stamp - start > duration:
                break
            command = avoider.step(scan)
            v, w = ramp.step(command.v, command.w, dt)
            drive.send(v, w)
            stats["frames"] += 1
            stats["states"][command.state] = stats["states"].get(command.state, 0) + 1
            pose = truth.pose if truth is not None else None
            if pose is not None:
                if last_xy is not None:
                    stats["distance_m"] += math.hypot(pose[0] - last_xy[0], pose[1] - last_xy[1])
                last_xy = pose[:2]
            if log:
                log.writerow([round(scan.stamp - start, 3), command.state, round(command.front_m, 3),
                              round(command.v, 3), round(command.w, 3), round(v, 3), round(w, 3),
                              *([round(p, 4) for p in pose] if pose else ["", "", ""])])
            if not quiet and stats["frames"] % 10 == 0:
                print(f"t {scan.stamp - start:6.1f}s  {command.state:6s}  front {command.front_m:4.2f} m  "
                      f"v {v:+.2f}  w {w:+.2f}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        drive.close()
        if log_file:
            log_file.close()
    stats["distance_m"] = round(stats["distance_m"], 2)
    stats["duration_s"] = round((last - start) if start is not None else 0.0, 1)
    return stats
