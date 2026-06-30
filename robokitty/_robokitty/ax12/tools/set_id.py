"""
tools/set_id.py – AX-12A servo ID change utility.

Implements Issue #19: a multi-step safety flow to change a servo's
default ID, ensuring only one servo is on the bus at a time.
"""

import time

from ..ax12 import AX12Interface
from .utils import header


# Robokitty uses IDs 0-11 (12 servos, 3 per leg)
MIN_ID = 0
MAX_ID = 11


def _prompt_continue(message: str) -> bool:
    """
    Print a message and wait for the user to type 'continue'.
    Returns True if they typed it, False on Ctrl+C or EOF.
    """
    print(message)
    while True:
        try:
            response = input(
                '\nType "continue" and press Enter to proceed, '
                             "or Ctrl+C to abort: "
              )
        except (KeyboardInterrupt, EOFError):
            print("\n\nAborted.")
            return False

        if response.strip().lower() == "continue":
            return True
        print('  → Please type exactly "continue" to proceed.')


def set_servo_id(ax: AX12Interface) -> bool:
    """
    Interactive CLI flow to safely change an AX-12A servo ID.

    Walks through:
      1. Hardware safety gate (confirm only one servo connected)
      2. Bus scan to find the current servo
      3. New ID input and confirmation
      4. Write + verify

    Returns True on success, False on failure or abort.
    """

    # ── Step 1: Hardware safety gate ──────────────────────────────────────
    header("Step 1 — Hardware safety gate")
    if not _prompt_continue(
        "\n  ⚠️  WARNING: DISCONNECT ALL SERVOS EXCEPT THE ONE YOU WANT TO CHANGE.\n"
        "\n"
        "  Only ONE AX-12A servo must be connected to the bus.\n"
        "  Having multiple servos connected during an ID change can\n"
        "  cause bus conflicts and assign duplicate IDs.\n"
        "\n"
        "  Confirm all other servos are physically disconnected."
    ):
        return False

    # ── Step 2: Scan the bus ──────────────────────────────────────────────
    header("Step 2 — Scanning bus")
    print("\n  🔍 Scanning bus for connected servos...")

    # Scan the full valid range (0-253) to find whatever is out there
    found = ax.scan(range(0, 254))

    if len(found) == 0:
        print("\n  ❌ No servos found on the bus.")
        print("     Check wiring, power supply, and baud rate.")
        return False

    if len(found) > 1:
        print(f"\n  ❌ Found {len(found)} servos on the bus (IDs: {found}).")
        print("     Only ONE servo must be connected for an ID change.")
        print("     Disconnect all extras and try again.")
        return False

    current_id = found[0]
    print("\n  Found 1 servo.")
    print(f"  Current ID: {current_id}")

    # ── Step 3: Get new ID and confirm ────────────────────────────────────
    header("Step 3 — ID change")

    # Get the new ID from the user
    while True:
        try:
            raw = input(f"\n  Enter new ID ({MIN_ID}-{MAX_ID}): ")
        except (KeyboardInterrupt, EOFError):
            print("\n\nAborted.")
            return False

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
            return True

        break

    # Confirm the change
    if not _prompt_continue(
        f"\n  ⚠️  You are about to change servo ID from {current_id} → {new_id}.\n"
        "\n"
        "  This cannot be undone from this tool without repeating the process."
    ):
        return False

    # ── Step 4: Write and verify ──────────────────────────────────────────
    header("Step 4 — Writing new ID")

    try:
        ax.write_byte(current_id, 0x03, new_id)  # Register.ID = 0x03
    except Exception as e:
        print(f"\n  ❌ Write failed: {e}")
        print("     Check power and wiring, then try again.")
        return False

    # Small delay for the EEPROM write to complete
    time.sleep(0.3)

    # Verify by pinging the new ID
    print("\n  Verifying...")
    if ax.ping(new_id):
        print("\n  ✅ ID changed successfully.")
        print(f"\n  Previous ID : {current_id}")
        print(f"  New ID      : {new_id} (verified)")
        print("\n  You may now disconnect this servo and connect the next one.")
        return True
    else:
        print(f"\n  ❌ Verification failed — no response at new ID {new_id}.")
        print(f"     The servo may still be at ID {current_id}.")
        print("     Try power-cycling the servo and running diagnostics.")
        return False
