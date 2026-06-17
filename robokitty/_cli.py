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

    parser.add_argument(
        "-g",
        "--gait",
        choices=["crawl", "walk", "run"],
        default="crawl",
        help="Set the quadruped leg movement pattern (default: walk)",
    )
    return parser.parse_args()
