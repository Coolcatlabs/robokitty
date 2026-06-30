"""
tools/set_id.py – AX-12A servo ID change utility.

Implements Issue #19: safely change a servo's ID with a multi-step
confirmation flow, ensuring only one servo is on the bus at a time.
"""

import time

from ..ax12 import AX12Interface
from ..registers import Register
from .utils import header


# Robokitty uses IDs 0-11 (12 servos, 3 per leg)
MIN_ID = 0
MAX_ID = 11

# AX-12A EEPROM write latency. The datasheet does not specify an exact
# duration, but Dynamixel application notes recommend at least 250 ms
# after an EEPROM write before issuing the next command.
EEPROM_WRITE_DELAY_S = 0.3


def _confirm(prompt: str) -> bool:
    """
    Display a prompt and wait for explicit y/n confirmation.
    Re-prompts on invalid input. Returns True only on 'y'/'yes',
    False on 'n'/'no' or Ctrl+C.
    """
    try:
        response = input(f"\n  {prompt} [y/N] ").strip().lower()
    except (KeyboardInterrupt, EOFError):
        print("\n\nAborted.")
        return False
    if response in ("y", "yes"):
        return True
    if response in ("n", "no"):
        print("  Cancelled.")
        return False
    print("  Please enter 'y' or 'n'.")
    return _confirm(prompt)


def _step_safety_gate() -> bool:
    """Step 1: confirm only one servo is physically connected."""
    header("Step 1 — Hardware safety gate")
    print(
        "\n  ⚠️  WARNING: DISCONNECT ALL SERVOS EXCEPT THE ONE YOU WANT TO CHANGE.\n"
        "\n"
        "  Only ONE AX-12A servo must be connected to the bus.\n"
        "  Having multiple servos connected during an ID change can\n"
        "  cause bus conflicts and assign duplicate IDs."
    )
    return _confirm("All other servos are disconnected?")


def _step_scan(ax: AX12Interface) -> int | None:
    """Step 2: scan the bus and return the single servo's ID, or None."""
    header("Step 2 — Scanning bus")
    print("\n  🔍 Scanning bus for connected servos...")

    found = ax.scan(range(0, 254))

    if len(found) == 0:
        print("\n  ❌ No servos found on the bus.")
        print("     Check wiring, power supply, and baud rate.")
        return None

    if len(found) > 1:
        print(f"\n  ❌ Found {len(found)} servos on the bus (IDs: {found}).")
        print("     Only ONE servo must be connected for an ID change.")
        print("     Disconnect all extras and try again.")
        return None

    current_id = found[0]
    print("\n  Found 1 servo.")
    print(f"  Current ID: {current_id}")
    return current_id


def _step_get_new_id(current_id: int) -> int | None:
    """Step 3: prompt user for the new ID, validate, and confirm."""
    header("Step 3 — ID change")

    while True:
        try:
            raw = input(f"\n  Enter new ID ({MIN_ID}-{MAX_ID}): ")
        except (KeyboardInterrupt, EOFError):
            print("\n\nAborted.")
            return None

        try:
            new_id = int(raw.strip())
        except ValueError:
            print(f"  → Please enter a number between {MIN_ID} and {MAX_ID}.")
            continue

        if not MIN_ID <= new_id <= MAX_ID:
            print(f"  → ID must be between {MIN_ID} and {MAX_ID}.")
            continue

        if new_id == current_id:
            print(f"  → Servo is already ID {current_id}. Nothing to change.")
            return None

        break

    if not _confirm(f"Change servo ID from {current_id} → {new_id}?"):
        return None

    return new_id


def _step_write_and_verify(ax: AX12Interface, current_id: int, new_id: int) -> bool:
    """Step 4: write the new ID to EEPROM and verify by pinging."""
    header("Step 4 — Writing new ID")

    try:
        ax.write_byte(current_id, Register.ID, new_id)
    except (TimeoutError, ValueError) as e:
        print(f"\n  ❌ Write failed: {e}")
        print("     Check power and wiring, then try again.")
        return False

    time.sleep(EEPROM_WRITE_DELAY_S)

    print("\n  Verifying...")
    if ax.ping(new_id):
        print("\n  ✅ ID changed successfully.")
        print(f"\n  Previous ID : {current_id}")
        print(f"  New ID      : {new_id} (verified)")
        print("\n  You may now disconnect this servo and connect the next one.")
        return True

    print(f"\n  ❌ Verification failed — no response at new ID {new_id}.")
    print(f"     The servo may still be at ID {current_id}.")
    print("     Try power-cycling the servo and running diagnostics.")
    return False


def set_servo_id(ax: AX12Interface) -> bool:
    """
    Interactive CLI flow to safely change an AX-12A servo ID.

    Returns True on success, False on failure or abort.
    """
    if not _step_safety_gate():
        return False

    current_id = _step_scan(ax)
    if current_id is None:
        return False

    new_id = _step_get_new_id(current_id)
    if new_id is None:
        return False

    return _step_write_and_verify(ax, current_id, new_id)
