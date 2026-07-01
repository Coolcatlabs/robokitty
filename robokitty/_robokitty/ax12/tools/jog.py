"""
tools/jog.py – AX-12A single-servo jog utility.

Move a servo to a user-specified angle, display a mini diagnostic
at each position, then return to the original position.
"""

import time

from ..ax12 import AX12Interface
from ..utils import position_to_degrees, _MIN_DEGREES, _MAX_DEGREES
from .utils import header, row, safe_read

# Speed for jog movements (moderate pace, not jarring)
JOG_SPEED = 200

# Settle time after the servo reports it has stopped,
# to let mechanical vibration die out before reading position.
SETTLE_DELAY_S = 0.2


def _mini_diagnostic(ax: AX12Interface, servo_id: int, label: str) -> None:
    """Print a compact snapshot of position, voltage, temperature, and load."""
    header(label)

    pos = safe_read(ax.get_position, servo_id, label="Position")
    if pos is not None:
        row("Position", f"{pos}  ({position_to_degrees(pos):.1f}°)")

    voltage = safe_read(ax.get_voltage, servo_id, label="Voltage")
    if voltage is not None:
        row("Voltage", f"{voltage:.1f} V")

    temp = safe_read(ax.get_temperature, servo_id, label="Temperature")
    if temp is not None:
        row("Temperature", f"{temp} °C")

    load = safe_read(ax.get_load, servo_id, label="Load")
    if load is not None:
        load_val = load & 0x3FF
        direction = "CCW" if load & 0x400 else "CW"
        row("Load", f"{load_val}  ({direction}, {load_val / 1023 * 100:.0f}%)")


def _step_ping_and_read(ax: AX12Interface, servo_id: int) -> int | None:
    """Ping the servo and return its current position, or None on failure."""
    if not ax.ping(servo_id):
        print(f"\n  ❌ No response from servo {servo_id}.")
        print("     Check wiring, power supply, and baud rate.")
        return None

    start_pos = safe_read(ax.get_position, servo_id, label="Start Position")
    if start_pos is None:
        print("\n  ❌ Could not read starting position.")
        return None

    return start_pos


def _step_move(
    ax: AX12Interface,
    servo_id: int,
    target_degrees: float,
    target_pos: int | None = None,
) -> bool:
    """Move the servo and wait for it to stop. Returns True on success."""
    try:
        if target_pos is not None:
            ax.move(servo_id, target_pos, speed=JOG_SPEED)
        else:
            ax.move_degrees(servo_id, target_degrees, speed=JOG_SPEED)
    except (TimeoutError, ValueError) as e:
        print(f"\n  ❌ Move command failed: {e}")
        return False

    if not ax.wait_for_stop(servo_id, timeout=5.0):
        print("  ⚠️  Servo did not stop within timeout.")

    time.sleep(SETTLE_DELAY_S)
    return True


def _step_verify_return(ax: AX12Interface, servo_id: int, start_pos: int) -> None:
    """Check how close the servo is to its starting position."""
    final_pos = safe_read(ax.get_position, servo_id, label="Final Position")
    if final_pos is not None:
        error = abs(start_pos - final_pos)
        if error <= 5:
            print(f"\n  ✅ Servo returned to start (error: {error} steps).")
        else:
            print(f"\n  ⚠️  Servo did not fully return (error: {error} steps).")


def jog_servo(ax: AX12Interface, servo_id: int, target_degrees: float) -> bool:
    """
    Jog a single servo to a target angle and back.

    Returns True on success, False on failure.
    """
    if not _MIN_DEGREES <= target_degrees <= _MAX_DEGREES:
        print(
            f"\n  ❌ Angle must be between {_MIN_DEGREES:.0f}° and {_MAX_DEGREES:.0f}°."
        )
        return False

    print(f"\n  Jogging servo {servo_id} → {target_degrees:.1f}° → back")

    start_pos = _step_ping_and_read(ax, servo_id)
    if start_pos is None:
        return False

    start_degrees = position_to_degrees(start_pos)

    _mini_diagnostic(ax, servo_id, f"Before jog — {start_degrees:.1f}°")

    header(f"Jogging to {target_degrees:.1f}°")
    print(f"\n  Moving {start_degrees:.1f}° → {target_degrees:.1f}°...")
    if not _step_move(ax, servo_id, target_degrees):
        return False

    _mini_diagnostic(ax, servo_id, f"At target — {target_degrees:.1f}°")

    header(f"Returning to {start_degrees:.1f}°")
    print(f"\n  Moving {target_degrees:.1f}° → {start_degrees:.1f}°...")
    if not _step_move(ax, servo_id, start_degrees, target_pos=start_pos):
        print(f"     Servo may be stuck at {target_degrees:.1f}°.")
        return False

    _mini_diagnostic(ax, servo_id, "After jog — returned")
    _step_verify_return(ax, servo_id, start_pos)

    return True
