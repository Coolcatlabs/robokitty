import argparse

from ._constants import DEFAULT_PORT, DEFAULT_BAUD
from . import __version__


def _cli_parser() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=f"RoboKitty IK Walking Controller: {__version__}"
    )
    parser.add_argument(
        "--port", default=DEFAULT_PORT, help=f"Serial port (default: {DEFAULT_PORT})"
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Baudrate (default: {DEFAULT_BAUD})",
    )
    parser.add_argument(
        "--dir-pin",
        type=int,
        default=None,
        help="GPIO BCM pin for half-duplex direction",
    )
    parser.add_argument(
        "--stand", action="store_true", help="Stand only (calibration mode)"
    )
    parser.add_argument(
        "--diag", action="store_true", help="Print IK diagnostics and exit"
    )
    parser.add_argument(
        "--identify", action="store_true", help="Flash each servo LED to verify wiring"
    )
    parser.add_argument(
        "--read-pose",
        action="store_true",
        help="Read current servo positions (torque off, pose manually)",
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Pose legs by hand, read positions, compute offsets",
    )
    return parser.parse_args()
