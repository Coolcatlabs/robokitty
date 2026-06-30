import argparse

from .config import DEFAULT_PORT, DEFAULT_BAUD
from . import __version__


def _cli_parser() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=f"RoboKitty IK Walking Controller: {__version__}"
    )
    parser.add_argument(
        "--version",
        action="version",
        version=f"{__package__} {__version__}",
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"Serial port (default: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Baudrate (default: {DEFAULT_BAUD})",
    )

    subparsers = parser.add_subparsers(dest="command", metavar="command")

    gait = subparsers.add_parser("gait", help="Run a gait pattern")
    gait.add_argument(
        "pattern",
        choices=["crawl", "walk", "run"],
        default="crawl",
        nargs="?",
        help="Leg movement pattern (default: crawl)",
    )

    servo = subparsers.add_parser("servo", help="AX-12A servo tools")
    servo_sub = servo.add_subparsers(dest="servo_command", metavar="servo_command")

    diagnose = servo_sub.add_parser(
        "diagnose", help="Print full register diagnostic for a servo"
    )
    diagnose.add_argument("id", type=int, help="Servo ID (0–253)")

    repair = servo_sub.add_parser("repair", help="Attempt software repair of a servo")
    repair.add_argument("id", type=int, help="Servo ID (0–253)")

    servo_sub.add_parser("set-id", help="Safely change a servo's ID (interactive)")

    return parser.parse_args()