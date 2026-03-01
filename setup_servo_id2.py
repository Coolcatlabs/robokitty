#!/usr/bin/env python3
"""
AX-12A Servo Configuration Tool
================================
Sets up a new/spare servo as ID 2 (RR Shoulder) for RoboKitty.

New AX-12A servos default to ID 1 at baudrate 1000000.
This script:
  1. Pings the servo at its current ID (default 1)
  2. Changes its ID to 2
  3. Sets joint mode (position control)
  4. Sets appropriate speed and torque limits

IMPORTANT: Only connect the ONE new servo to the bus when running this.
           Disconnect all other servos first or you'll get ID conflicts.

Usage:
    python3 setup_servo_id2.py
    python3 setup_servo_id2.py --current-id 1    (if servo is already ID 1)
    python3 setup_servo_id2.py --current-id 3    (if servo has a different ID)
"""

import time
import sys
import argparse

# AX-12A Protocol 1.0
HEADER = bytes([0xFF, 0xFF])
INST_WRITE = 0x03
INST_READ = 0x02
INST_PING = 0x01

# Control table addresses
ADDR_ID = 3
ADDR_BAUD = 4
ADDR_RETURN_DELAY = 5
ADDR_CW_LIMIT = 6       # 2 bytes
ADDR_CCW_LIMIT = 8      # 2 bytes
ADDR_MAX_TORQUE = 14     # 2 bytes
ADDR_STATUS_RETURN = 16
ADDR_TORQUE_ENABLE = 24
ADDR_LED = 25
ADDR_CW_MARGIN = 26
ADDR_CCW_MARGIN = 27
ADDR_CW_SLOPE = 28
ADDR_CCW_SLOPE = 29
ADDR_GOAL_POSITION = 30  # 2 bytes
ADDR_MOVING_SPEED = 32   # 2 bytes
ADDR_TORQUE_LIMIT = 34   # 2 bytes


def checksum(data):
    return (~sum(data)) & 0xFF


def build_packet(servo_id, instruction, params=b''):
    length = len(params) + 2
    body = bytes([servo_id, length, instruction]) + params
    chk = checksum(body)
    return HEADER + body + bytes([chk])


def main():
    parser = argparse.ArgumentParser(description="Configure new AX-12A as ID 2")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baud", type=int, default=1000000, help="Baudrate")
    parser.add_argument("--current-id", type=int, default=1,
                        help="Current ID of the new servo (default: 1)")
    parser.add_argument("--target-id", type=int, default=2,
                        help="Target ID to set (default: 2)")
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        print("ERROR: pyserial not installed. Run: pip install pyserial")
        return

    print("=" * 50)
    print("  AX-12A Servo Setup for RoboKitty")
    print("  RR Shoulder = ID 2")
    print("=" * 50)
    print()
    print(f"  Port:       {args.port}")
    print(f"  Baudrate:   {args.baud}")
    print(f"  Current ID: {args.current_id}")
    print(f"  Target ID:  {args.target_id}")
    print()
    print("  WARNING: Only the NEW servo should be connected!")
    print("  Disconnect all other servos from the bus first.")
    print()

    input("  Press Enter to continue (Ctrl+C to cancel)...")
    print()

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=0.1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
    except Exception as e:
        print(f"ERROR: Cannot open {args.port}: {e}")
        return

    def send(servo_id, instruction, params=b''):
        packet = build_packet(servo_id, instruction, params)
        ser.write(packet)
        ser.flush()
        time.sleep(0.05)
        # Read any response
        resp = ser.read(256)
        return resp

    def write_byte(servo_id, addr, value):
        send(servo_id, INST_WRITE, bytes([addr, value]))

    def write_word(servo_id, addr, value):
        send(servo_id, INST_WRITE, bytes([addr, value & 0xFF, (value >> 8) & 0xFF]))

    current_id = args.current_id
    target_id = args.target_id

    # Step 1: Ping current ID
    print(f"  1. Pinging servo at ID {current_id}...")
    resp = send(current_id, INST_PING)
    if len(resp) >= 6:
        error = resp[4] if len(resp) > 4 else 0
        print(f"     Response received! Error byte: {error}")
        if error == 0:
            print(f"     Servo ID {current_id} is alive and healthy.")
        else:
            print(f"     Servo responded but has error flags: {error:#04x}")
    else:
        print(f"     No response from ID {current_id}.")
        print(f"     Trying broadcast ping (ID 254)...")
        resp = send(254, INST_PING)
        if len(resp) >= 6:
            found_id = resp[2]
            print(f"     Found servo at ID {found_id}!")
            current_id = found_id
        else:
            print(f"     No servo found. Check wiring and power.")
            ser.close()
            return

    # Step 2: Disable torque before changing settings
    print(f"  2. Disabling torque...")
    write_byte(current_id, ADDR_TORQUE_ENABLE, 0)
    time.sleep(0.1)

    # Step 3: Change ID
    if current_id != target_id:
        print(f"  3. Changing ID from {current_id} to {target_id}...")
        write_byte(current_id, ADDR_ID, target_id)
        time.sleep(0.3)

        # Verify new ID
        resp = send(target_id, INST_PING)
        if len(resp) >= 6:
            print(f"     ID change successful! Servo now responds as ID {target_id}.")
        else:
            print(f"     WARNING: No response at new ID {target_id}.")
            print(f"     Trying to ping new ID again...")
            time.sleep(0.5)
            resp = send(target_id, INST_PING)
            if len(resp) >= 6:
                print(f"     OK, servo responds as ID {target_id}.")
            else:
                print(f"     FAILED. Servo may still be at old ID.")
                ser.close()
                return
    else:
        print(f"  3. ID already {target_id}, skipping.")

    sid = target_id

    # Step 4: Set joint mode (CW limit=0, CCW limit=1023)
    print(f"  4. Setting joint mode (position control)...")
    write_word(sid, ADDR_CW_LIMIT, 0)
    time.sleep(0.05)
    write_word(sid, ADDR_CCW_LIMIT, 1023)
    time.sleep(0.05)

    # Step 5: Set max torque
    print(f"  5. Setting max torque to 1023 (100%)...")
    write_word(sid, ADDR_MAX_TORQUE, 1023)
    time.sleep(0.05)
    write_word(sid, ADDR_TORQUE_LIMIT, 1023)
    time.sleep(0.05)

    # Step 6: Set compliance margins and slopes
    print(f"  6. Setting compliance (margins=1, slopes=32)...")
    write_byte(sid, ADDR_CW_MARGIN, 1)
    write_byte(sid, ADDR_CCW_MARGIN, 1)
    write_byte(sid, ADDR_CW_SLOPE, 32)
    write_byte(sid, ADDR_CCW_SLOPE, 32)
    time.sleep(0.05)

    # Step 7: Set return delay time
    print(f"  7. Setting return delay to 50us...")
    write_byte(sid, ADDR_RETURN_DELAY, 25)  # 25 * 2us = 50us
    time.sleep(0.05)

    # Step 8: Enable torque
    print(f"  8. Enabling torque...")
    write_byte(sid, ADDR_TORQUE_ENABLE, 1)
    time.sleep(0.1)

    # Step 9: Test movement
    print(f"  9. Testing movement...")
    print(f"     Moving to center (512)...")
    write_word(sid, ADDR_MOVING_SPEED, 200)
    time.sleep(0.05)
    write_word(sid, ADDR_GOAL_POSITION, 512)
    time.sleep(1.5)

    print(f"     Moving out (300)...")
    write_word(sid, ADDR_GOAL_POSITION, 300)
    time.sleep(1.5)

    print(f"     Moving in (700)...")
    write_word(sid, ADDR_GOAL_POSITION, 700)
    time.sleep(1.5)

    print(f"     Back to center (512)...")
    write_word(sid, ADDR_GOAL_POSITION, 512)
    time.sleep(1.5)

    # Step 10: Flash LED to confirm
    print(f"  10. Flashing LED to confirm...")
    for _ in range(5):
        write_byte(sid, ADDR_LED, 1)
        time.sleep(0.2)
        write_byte(sid, ADDR_LED, 0)
        time.sleep(0.2)

    ser.close()

    print()
    print("=" * 50)
    print(f"  DONE! Servo is now configured as ID {target_id}")
    print(f"  Settings:")
    print(f"    ID:         {target_id}")
    print(f"    Mode:       Joint (position control)")
    print(f"    CW Limit:   0")
    print(f"    CCW Limit:  1023")
    print(f"    Max Torque:  100%")
    print(f"    Baudrate:   {args.baud} (unchanged)")
    print()
    print(f"  You can now reconnect all servos and run")
    print(f"  the walking controller.")
    print("=" * 50)


if __name__ == "__main__":
    main()