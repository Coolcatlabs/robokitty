import sys
import threading
import time
from typing import TYPE_CHECKING

from .constants import AX12_CENTER, AX12_DEG_TO_UNITS, LEG_SERVO_CONFIG, LegID, GaitType

if TYPE_CHECKING:
    from .config import ServoJointConfig
    from .walker import QuadrupedWalker


from .. import _log


def angle_to_raw(angle_deg: float, config: "ServoJointConfig") -> int:
    """Convert joint angle in degrees to AX-12A raw position (0-1023)."""
    clamped = max(config.min_deg, min(config.max_deg, angle_deg))
    effective = clamped + config.offset_deg
    if config.inverted:
        effective = -effective
    raw = int(AX12_CENTER + effective * AX12_DEG_TO_UNITS)
    return max(0, min(1023, raw))


def print_diagnostics(walker: "QuadrupedWalker"):
    """Print IK solution for standing pose."""
    _log.info("\n--- RoboKitty3 Standing Pose Diagnostics ---")
    _log.info(
        f"Leg dims: coxa={walker.leg_dims.coxa_length}mm "
        f"femur={walker.leg_dims.femur_length}mm "
        f"tibia={walker.leg_dims.tibia_length}mm"
    )
    _log.info(
        f"Body: half_length={walker.body_dims.half_length}mm "
        f"half_width={walker.body_dims.half_width}mm"
    )
    _log.info(f"Standing height: {walker.gait_cfg.body_height}mm")
    _log.info(
        f"Max leg reach: {walker.leg_dims.femur_length + walker.leg_dims.tibia_length}mm "
        f"(using {walker.gait_cfg.body_height / (walker.leg_dims.femur_length + walker.leg_dims.tibia_length) * 100:.0f}%)"
    )

    for leg_id in LegID:
        foot = walker.neutral_feet[leg_id]
        is_front = walker.leg_is_front[leg_id]
        angles = walker.ik.solve(*foot, is_front=is_front)
        configs = LEG_SERVO_CONFIG[leg_id]
        joint_names = ["Shoulder", "Femur   ", "Leg     "]

        _log.info(
            f"  {leg_id.value:12s}  foot=({foot[0]:6.1f}, {foot[1]:6.1f}, {foot[2]:6.1f})"
            f"  {'FRONT' if is_front else 'REAR'}"
        )
        for i, (cfg, name) in enumerate(zip(configs, joint_names)):
            raw = angle_to_raw(angles[i], cfg)
            inv = " INV" if cfg.inverted else ""
            _log.info(
                f"    {name} ID{cfg.servo_id:2d}: {angles[i]:+7.1f}deg -> raw={raw:4d}"
                f"  (offset={cfg.offset_deg:+.1f}deg{inv})"
            )


def run_keyboard_control(walker: "QuadrupedWalker"):
    """Terminal keyboard control for testing."""
    border = "=" * 50
    menu_content = (
        f"\n{border}\n"
        "  ROBOKITTY3 KEYBOARD CONTROL\n"
        f"{border}\n"
        "  w/s     = forward / backward\n"
        "  a/d     = turn left / right\n"
        "  SPACE   = stop movement\n"
        "  1       = trot gait\n"
        "  2       = walk gait (slow, stable)\n"
        "  3       = pace gait\n"
        "  i       = identify servos (flash LEDs)\n"
        "  t       = test RR shoulder servo (sweep in/out)\n"
        "  r       = diagnose RR shoulder + scan all servos\n"
        "  p       = read current servo positions\n"
        "  q       = quit\n"
        f"{border}\n"
    )

    _log.info(menu_content)

    speed = 0.0
    turn = 0.0
    speed_inc = 0.2

    try:
        import tty
        import termios
        import select

        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        try:
            while True:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == "q":
                        break
                    elif key == "w":
                        speed = min(1.0, speed + speed_inc)
                    elif key == "s":
                        speed = max(-1.0, speed - speed_inc)
                    elif key == "a":
                        turn = max(-1.0, turn - 0.3)
                    elif key == "d":
                        turn = min(1.0, turn + 0.3)
                    elif key == " ":
                        speed = 0.0
                        turn = 0.0
                    elif key == "1":
                        walker.set_gait(GaitType.TROT)
                    elif key == "2":
                        walker.set_gait(GaitType.WALK)
                    elif key == "3":
                        walker.set_gait(GaitType.PACE)
                    elif key == "i":
                        _log.error("\n  Identifying servos...")
                        walker.identify_all_servos()
                        _log.error("  Done!\n")
                    elif key == "t":
                        was_running = walker._running
                        if was_running:
                            walker._running = False
                            if walker._walk_thread:
                                walker._walk_thread.join(timeout=2.0)
                            time.sleep(0.1)
                        _log.error(
                            "\n  Testing RR Shoulder (ID 2) - sweeping in/out..."
                        )
                        walker.test_rr_shoulder()
                        _log.error("  Done!\n")
                        if was_running:
                            walker._running = True
                            walker._walk_thread = threading.Thread(
                                target=walker._control_loop, daemon=True
                            )
                            walker._walk_thread.start()
                    elif key == "r":
                        # Must stop walk loop so bus is free for reads
                        was_running = walker._running
                        if was_running:
                            walker._running = False
                            if walker._walk_thread:
                                walker._walk_thread.join(timeout=2.0)
                            time.sleep(0.1)
                        _log.info("\n  Diagnosing RR Shoulder (ID 2)...")
                        walker.diagnose_servo(2)
                        # Also scan all servos for errors
                        _log.info("\n  Scanning ALL servos for errors...")
                        walker.scan_all_errors()
                        if was_running:
                            walker._running = True
                            walker._walk_thread = threading.Thread(
                                target=walker._control_loop, daemon=True
                            )
                            walker._walk_thread.start()
                        _log.info("  Done!\n")
                    elif key == "p":
                        was_running = walker._running
                        if was_running:
                            walker._running = False
                            if walker._walk_thread:
                                walker._walk_thread.join(timeout=2.0)
                            time.sleep(0.1)
                        _log.info("\n  Reading current positions...")
                        walker.read_all_positions()
                        if was_running:
                            walker._running = True
                            walker._walk_thread = threading.Thread(
                                target=walker._control_loop, daemon=True
                            )
                            walker._walk_thread.start()

                    walker.set_speed(speed)
                    walker.set_turn(turn)
                    sys.stdout.write(
                        f"\r  Speed: {speed:+.1f}  Turn: {turn:+.1f}  "
                        f"Gait: {walker.gait_type.value}       "
                    )
                    sys.stdout.flush()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    except ImportError:
        _log.info("(Line-input mode - type command and press Enter)")
        while True:
            try:
                cmd = input(f"[spd={speed:+.1f} trn={turn:+.1f}] > ").strip().lower()
                if cmd == "q":
                    break
                elif cmd == "w":
                    speed = min(1.0, speed + speed_inc)
                elif cmd == "s":
                    speed = max(-1.0, speed - speed_inc)
                elif cmd == "a":
                    turn = max(-1.0, turn - 0.3)
                elif cmd == "d":
                    turn = min(1.0, turn + 0.3)
                elif cmd in ("x", ""):
                    speed = 0.0
                    turn = 0.0
                elif cmd == "1":
                    walker.set_gait(GaitType.TROT)
                elif cmd == "2":
                    walker.set_gait(GaitType.WALK)
                elif cmd == "i":
                    walker.identify_all_servos()
                walker.set_speed(speed)
                walker.set_turn(turn)
            except (EOFError, KeyboardInterrupt):
                break
