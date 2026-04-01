import time
import signal
import sys

from . import _log
from ._cli import _cli_parser
from ._robokitty import QuadrupedWalker, run_keyboard_control, print_diagnostics


def main():
    args = _cli_parser()

    walker = QuadrupedWalker(
        port=args.port,
        baudrate=args.baud,
        direction_pin=args.dir_pin,
    )

    if args.diag:
        print_diagnostics(walker)
        return

    if not walker.connect():
        _log.error("Failed to connect. Check port and power.")
        return

    def signal_handler(sig, frame):
        _log.info("\nShutting down...")
        walker.disconnect()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    if args.identify:
        _log.info("\nIdentifying all servos by flashing LEDs...")
        walker.identify_all_servos()
        walker.disconnect()
        return

    if args.read_pose:
        # Disable torque so servos can be moved by hand
        _log.info("\nDisabling torque on all servos for manual posing...")
        all_ids = walker._all_servo_ids()
        for sid in all_ids:
            walker.servos.enable_torque(sid, False)
            time.sleep(0.003)
        _log.info("Torque OFF. Pose the robot into the desired standing position.")
        _log.info("Press Enter when ready to read positions...")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            walker.disconnect()
            return
        walker.read_all_positions()
        walker.disconnect()
        return

    if args.calibrate:
        border = "=" * 62
        calibration_msg = (
            f"\n{border}\n"
            "  ROBOKITTY3 STANDING CALIBRATION\n"
            f"{border}\n"
            "  1. All servo torque will be [bold red]DISABLED[/bold red]\n"
            "  2. Manually pose ALL legs into your desired standing position\n"
            "  3. Press Enter to read positions\n"
            "  4. Offsets will be calculated and displayed\n"
            f"{border}"
        )

        _log.info(calibration_msg)
        # Disable torque
        all_ids = walker._all_servo_ids()
        for sid in all_ids:
            walker.servos.enable_torque(sid, False)
            time.sleep(0.003)
        _log.info("\nTorque OFF on all servos. Pose the legs now.")
        _log.info("Press Enter when the robot is in the desired standing position...")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            walker.disconnect()
            return

        walker.calibrate_standing()
        walker.disconnect()
        return

    if args.stand:
        print_diagnostics(walker)
        _log.info("Moving to standing pose (slowly)...")
        walker.smooth_stand(2.0)
        _log.info("Standing. Adjust offsets/inversions as needed. Ctrl+C to exit.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        walker.disconnect()
        return

    # Normal operation
    print_diagnostics(walker)
    walker.smooth_stand(1.5)
    walker.start()

    try:
        run_keyboard_control(walker)
    finally:
        walker.disconnect()
        _log.info("\nRoboKitty3 shutting down. Bye!")


if __name__ == "__main__":
    raise SystemExit(main())
