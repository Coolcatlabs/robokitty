"""
tools/jog.py – AX-12A single-servo jog utility.

Move a servo to a user-specified angle, display a mini diagnostic
at each position, then return to the original position.
"""

import time

from ..ax12 import AX12Interface
from ..utils import position_to_degrees
from .utils import header, row, safe_read

# AX-12A joint range is 0-300 degrees
MIN_ANGLE = 0.0
MAX_ANGLE = 300.0

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


def jog_servo(ax: AX12Interface, servo_id: int, target_degrees: float) -> bool:
    """
    Jog a single servo to a target angle and back.

    Steps:
      1. Ping and read the starting position
      2. Show mini diagnostic at starting position
      3. Move to the target angle
      4. Show mini diagnostic at target position
      5. Return to the original position
      6. Show final diagnostic to confirm return

    Returns True on success, False on failure.
    """
    if not MIN_ANGLE <= target_degrees <= MAX_ANGLE:
        print(f"\n  ❌ Angle must be between {MIN_ANGLE:.0f}° and {MAX_ANGLE:.0f}°.")
        return False

    # Step 1: Ping
    print(f"\n  Jogging servo {servo_id} → {target_degrees:.1f}° → back")

    if not ax.ping(servo_id):
        print(f"\n  ❌ No response from servo {servo_id}.")
        print("     Check wiring, power supply, and baud rate.")
        return False

    start_pos = safe_read(ax.get_position, servo_id, label="Start Position")
    if start_pos is None:
        print("\n  ❌ Could not read starting position.")
        return False

    start_degrees = position_to_degrees(start_pos)

    # Step 2: Starting diagnostic
    _mini_diagnostic(ax, servo_id, f"Before jog — {start_degrees:.1f}°")

    # Step 3: Move to target
    header(f"Jogging to {target_degrees:.1f}°")
    print(f"\n  Moving {start_degrees:.1f}° → {target_degrees:.1f}°...")

    try:
        ax.move_degrees(servo_id, target_degrees, speed=JOG_SPEED)
    except (TimeoutError, ValueError) as e:
        print(f"\n  ❌ Move command failed: {e}")
        return False

    if not ax.wait_for_stop(servo_id, timeout=5.0):
        print("  ⚠️  Servo did not stop within timeout.")

    time.sleep(SETTLE_DELAY_S)

    # Step 4: Diagnostic at target
    _mini_diagnostic(ax, servo_id, f"At target — {target_degrees:.1f}°")

    # Step 5: Return to start
    header(f"Returning to {start_degrees:.1f}°")
    print(f"\n  Moving {target_degrees:.1f}° → {start_degrees:.1f}°...")

    try:
        ax.move(servo_id, start_pos, speed=JOG_SPEED)
    except (TimeoutError, ValueError) as e:
        print(f"\n  ❌ Return move failed: {e}")
        print(f"     Servo may be stuck at {target_degrees:.1f}°.")
        return False

    if not ax.wait_for_stop(servo_id, timeout=5.0):
        print("  ⚠️  Servo did not stop within timeout.")

    time.sleep(SETTLE_DELAY_S)

    # Step 6: Final diagnostic
    _mini_diagnostic(ax, servo_id, "After jog — returned")

    final_pos = safe_read(ax.get_position, servo_id, label="Final Position")
    if final_pos is not None:
        error = abs(start_pos - final_pos)
        if error <= 5:
            print(f"\n  ✅ Servo returned to start (error: {error} steps).")
        else:
            print(f"\n  ⚠️  Servo did not fully return (error: {error} steps).")

    return True
