"""
tools/repair.py – AX-12A servo repair tool.

Attempts to recover a servo from common software-recoverable faults:
  - EEPROM lock preventing writes
  - Status Return Level muted or modified
  - Wheel mode (angle limits both 0)
  - Torque disabled or torque limit zeroed
  - Compliance values outside sensible range
  - Servo not reaching goal position

Hardware faults (burnt coil, stripped gears, dead driver IC) cannot be
repaired here. If the servo does not respond to a ping, this function
returns immediately without attempting any writes.
"""

import time

from ..ax12 import AX12Interface
from ..registers import Register
from ..utils import position_to_degrees

from .utils import W, header, row, safe_read, safe_write

# Repair targets
_CENTER_POSITION = 512
_MOVE_SPEED = 200
_POSITION_TOL = 20  # steps; goal vs present position tolerance
_MOVE_TIMEOUT = 4.0  # seconds to wait for the servo to reach centre

# Timing
_TORQUE_SETTLE = 0.1  # seconds after disabling torque before EEPROM writes
_DRIVER_ENERGISE = 0.15  # seconds after enabling torque before commanding movement


def repair_servo(ax: AX12Interface, servo_id: int) -> bool:
    """
    Attempt to repair common software-recoverable faults on a single servo.

    Steps
    -----
    1.  Ping              — abort immediately if no response.
    2.  EEPROM lock       — clear LOCK bit if set (RAM register, safe before torque-off).
    3.  Disable torque    — required before any EEPROM writes; abort if this fails.
    4.  Status Return Level — restore to 2 (respond to all) if muted.
    5.  Angle limits      — restore joint mode if servo is in wheel mode.
    6.  Torque limits     — restore MAX_TORQUE (EEPROM) and TORQUE_LIMIT (RAM) to 1023.
    7.  Compliance        — restore margins and slopes to safe defaults.
    8.  Enable torque     — re-energise the motor.
    9.  Verify            — drive to centre (512) and confirm position is reached.

    Returns True if the servo is responding to position commands, False otherwise.
    """
    print()
    print(f"  {'═' * (W - 4)}")
    print(f"  {'AX-12A Repair':^{W - 4}}")
    print(f"  {'Servo ID ' + str(servo_id):^{W - 4}}")
    print(f"  {'═' * (W - 4)}")

    # ── Step 1: ping ─────────────────────────────────────────────────────────
    header("Step 1  —  Ping")
    if not ax.ping(servo_id):
        print(f"  !!  No response from servo {servo_id}.")
        print("      Check wiring, power, and baud rate.")
        print("      Cannot continue — hardware must be inspected.\n")
        return False
    print(f"  {'Ping':.<32} OK")

    applied: list[str] = []
    failed: list[str] = []

    def _apply(label: str, write_fn, *args) -> bool:
        ok = safe_write(write_fn, *args, label=label)
        (applied if ok else failed).append(label)
        return ok

    # ── Step 2: unlock EEPROM ────────────────────────────────────────────────
    # LOCK is a RAM register — safe to write before torque is disabled.
    header("Step 2  —  EEPROM lock")
    lock = safe_read(ax.read_byte, servo_id, Register.LOCK, label="Lock")
    if lock:
        row("Lock", "LOCKED — clearing")
        _apply("Unlock EEPROM", ax.write_byte, servo_id, Register.LOCK, 0)
    else:
        row("Lock", "unlocked  (no action needed)")

    # ── Step 3: disable torque ───────────────────────────────────────────────
    # EEPROM writes while torque is active are unreliable on AX-12A.
    # Abort the entire sequence if this fails — proceeding risks corruption.
    header("Step 3  —  Disable torque")
    if not _apply("Torque off", ax.torque_enable, servo_id, False):
        print("\n  !! CRITICAL: Failed to disable torque.")
        print("     EEPROM writes while torque is active are unstable on AX-12A.")
        print("     Aborting repair sequence to prevent data corruption.\n")
        return False
    time.sleep(_TORQUE_SETTLE)

    # ── Step 4: Status Return Level (EEPROM) ─────────────────────────────────
    header("Step 4  —  Status Return Level")
    srl = safe_read(ax.read_byte, servo_id, Register.STATUS_RETURN_LEVEL, label="SRL")
    if srl is None:
        row("SRL", "READ FAILED — skipping")
    elif srl != 2:
        row("SRL", f"{srl} — restoring to 2 (respond to all)")
        _apply("SRL → 2", ax.write_byte, servo_id, Register.STATUS_RETURN_LEVEL, 2)
    else:
        row("SRL", "2  (ok)")

    # ── Step 5: restore joint mode (EEPROM) ──────────────────────────────────
    header("Step 5  —  Angle limits / mode")
    cw_limit = safe_read(
        ax.read_word, servo_id, Register.CW_ANGLE_LIMIT_L, label="CW Limit"
    )
    ccw_limit = safe_read(
        ax.read_word, servo_id, Register.CCW_ANGLE_LIMIT_L, label="CCW Limit"
    )

    if cw_limit is None or ccw_limit is None:
        row("Mode", "READ FAILED — forcing joint mode defaults")
        _apply("CW limit → 0", ax.write_word, servo_id, Register.CW_ANGLE_LIMIT_L, 0)
        _apply(
            "CCW limit → 1023",
            ax.write_word,
            servo_id,
            Register.CCW_ANGLE_LIMIT_L,
            1023,
        )
    elif cw_limit == 0 and ccw_limit == 0:
        row("Mode", "WHEEL MODE — restoring joint mode")
        # CW is already 0; only CCW needs to change to re-enable joint mode.
        _apply(
            "CCW limit → 1023",
            ax.write_word,
            servo_id,
            Register.CCW_ANGLE_LIMIT_L,
            1023,
        )
    else:
        row("Mode", f"Joint  ({cw_limit}–{ccw_limit})  (no action needed)")

    # ── Step 6: restore torque limits (EEPROM & RAM) ─────────────────────────
    header("Step 6  —  Torque limits")
    max_torque = safe_read(
        ax.read_word, servo_id, Register.MAX_TORQUE_L, label="Max Torque EEPROM"
    )
    torque_limit = safe_read(
        ax.read_word, servo_id, Register.TORQUE_LIMIT_L, label="Torque Limit RAM"
    )

    if max_torque is None:
        row("Max Torque (EEPROM)", "READ FAILED — skipping")
    elif max_torque < 1023:
        row("Max Torque (EEPROM)", f"{max_torque} — restoring to 1023")
        _apply(
            "Max torque → 1023", ax.write_word, servo_id, Register.MAX_TORQUE_L, 1023
        )
    else:
        row("Max Torque (EEPROM)", f"{max_torque}  (ok)")

    if torque_limit is None:
        row("Torque Limit (RAM)", "READ FAILED — skipping")
    elif torque_limit < 1023:
        row("Torque Limit (RAM)", f"{torque_limit} — restoring to 1023")
        _apply(
            "Torque limit → 1023",
            ax.write_word,
            servo_id,
            Register.TORQUE_LIMIT_L,
            1023,
        )
    else:
        row("Torque Limit (RAM)", f"{torque_limit}  (ok)")

    # ── Step 7: compliance defaults (RAM) ────────────────────────────────────
    header("Step 7  —  Compliance")
    _apply("CW margin → 1", ax.write_byte, servo_id, Register.CW_COMPLIANCE_MARGIN, 1)
    _apply("CCW margin → 1", ax.write_byte, servo_id, Register.CCW_COMPLIANCE_MARGIN, 1)
    _apply("CW slope → 32", ax.write_byte, servo_id, Register.CW_COMPLIANCE_SLOPE, 32)
    _apply("CCW slope → 32", ax.write_byte, servo_id, Register.CCW_COMPLIANCE_SLOPE, 32)

    # ── Step 8: re-enable torque ─────────────────────────────────────────────
    header("Step 8  —  Enable torque")
    _apply("Torque on", ax.torque_enable, servo_id, True)
    time.sleep(_DRIVER_ENERGISE)

    # ── Step 9: drive to centre and verify ───────────────────────────────────
    header("Step 9  —  Move to centre and verify")
    _apply("Speed → 200", ax.set_moving_speed, servo_id, _MOVE_SPEED)
    _apply("Goal → 512", ax.set_goal_position, servo_id, _CENTER_POSITION)

    stopped = ax.wait_for_stop(servo_id, poll_interval=0.05, timeout=_MOVE_TIMEOUT)
    if not stopped:
        row("Movement", f"WARNING: still moving after {_MOVE_TIMEOUT:.0f} s timeout")

    pos = safe_read(ax.get_position, servo_id, label="Present Position")
    if pos is not None:
        diff = abs(_CENTER_POSITION - pos)
        row(
            "Goal Position",
            f"{_CENTER_POSITION}  ({position_to_degrees(_CENTER_POSITION):.1f}°)",
        )
        row("Present Position", f"{pos}  ({position_to_degrees(pos):.1f}°)")
        row("Error", f"{diff} steps")
        success = diff < _POSITION_TOL
    else:
        success = False

    # ── Summary ──────────────────────────────────────────────────────────────
    header("Summary")

    if applied:
        print("  Applied:")
        for a in applied:
            print(f"    ✓  {a}")

    if failed:
        print("  Failed:")
        for f in failed:
            print(f"    ✗  {f}")

    print()
    if success:
        print("  ✓  Servo is responding to position commands.")
    else:
        print("  ✗  Servo is still not moving.")
        print("     Software repair exhausted — inspect hardware:")
        print("       • Check supply voltage under load")
        print("       • Listen for stall noise or heat")
        print("       • Verify no mechanical obstruction")
        print("       • Driver IC or winding may be damaged")

    print(f"\n  {'─' * (W - 4)}\n")
    return success
