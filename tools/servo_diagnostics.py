#!/usr/bin/env python3
"""
AX-12A Servo Diagnostic Reader
================================
Reads ALL registers from a servo and reports its full state.
Includes error flags, position, load, temperature, mode, limits.

Usage:
    python3 servo_diag.py              (reads servo ID 2)
    python3 servo_diag.py --id 2
    python3 servo_diag.py --id 2 --fix (attempt to fix common issues)
"""

import time
import argparse

try:
    import serial
except ImportError:
    print("ERROR: pip install pyserial")
    exit(1)

HEADER = bytes([0xFF, 0xFF])
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03


def checksum(data):
    return (~sum(data)) & 0xFF


def build_packet(servo_id, instruction, params=b""):
    length = len(params) + 2
    body = bytes([servo_id, length, instruction]) + params
    return HEADER + body + bytes([checksum(body)])


class AX12Diag:
    def __init__(self, port="/dev/ttyUSB0", baud=1000000):
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        time.sleep(0.1)
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def send_recv(self, servo_id, instruction, params=b""):
        self.ser.reset_input_buffer()
        packet = build_packet(servo_id, instruction, params)
        self.ser.write(packet)
        self.ser.flush()
        time.sleep(0.05)
        resp = self.ser.read(256)
        return resp

    def ping(self, servo_id):
        resp = self.send_recv(servo_id, INST_PING)
        if len(resp) >= 6:
            error = resp[4]
            return True, error
        return False, None

    def read_byte(self, servo_id, addr):
        resp = self.send_recv(servo_id, INST_READ, bytes([addr, 1]))
        if len(resp) >= 7:
            error = resp[4]
            value = resp[5]
            return value, error
        return None, None

    def read_word(self, servo_id, addr):
        resp = self.send_recv(servo_id, INST_READ, bytes([addr, 2]))
        if len(resp) >= 8:
            error = resp[4]
            value = resp[5] | (resp[6] << 8)
            return value, error
        return None, None

    def write_byte(self, servo_id, addr, value):
        self.send_recv(servo_id, INST_WRITE, bytes([addr, value]))

    def write_word(self, servo_id, addr, value):
        self.send_recv(
            servo_id, INST_WRITE, bytes([addr, value & 0xFF, (value >> 8) & 0xFF])
        )

    def read_all(self, servo_id):
        """Read and display all important registers."""
        print(f"\n{'=' * 55}")
        print(f"  AX-12A Full Diagnostic - Servo ID {servo_id}")
        print(f"{'=' * 55}\n")

        # Ping first
        alive, ping_err = self.ping(servo_id)
        if not alive:
            print(f"  !! NO RESPONSE from servo ID {servo_id}")
            print("     Check wiring, power, and ID")
            return False
        print(f"  Ping: OK (error byte: {ping_err})")
        if ping_err:
            self.decode_error(ping_err)

        # --- EEPROM (persistent settings) ---
        print("\n  --- EEPROM (Persistent Settings) ---")

        v, _ = self.read_word(servo_id, 0)
        if v is not None:
            print(f"  Model Number:        {v} ({'AX-12A' if v == 12 else 'UNKNOWN'})")

        v, _ = self.read_byte(servo_id, 2)
        print(f"  Firmware Version:    {v}")

        v, _ = self.read_byte(servo_id, 3)
        print(f"  ID:                  {v}")

        v, _ = self.read_byte(servo_id, 4)
        baud_actual = 2000000 / (v + 1) if v is not None else None
        print(
            f"  Baud Rate:           {v} ({int(baud_actual) if baud_actual else '?'} bps)"
        )

        v, _ = self.read_byte(servo_id, 5)
        print(f"  Return Delay:        {v} ({v * 2 if v else '?'} us)")

        cw_limit, _ = self.read_word(servo_id, 6)
        ccw_limit, _ = self.read_word(servo_id, 8)
        print(f"  CW Angle Limit:      {cw_limit}")
        print(f"  CCW Angle Limit:     {ccw_limit}")

        # Determine mode
        if cw_limit == 0 and ccw_limit == 0:
            mode = "WHEEL MODE (no position control!)"
        elif cw_limit == 0 and ccw_limit == 1023:
            mode = "Joint Mode (full range)"
        else:
            mode = f"Joint Mode (limited: {cw_limit}-{ccw_limit})"
        print(f"  >> MODE:             {mode}")

        v, _ = self.read_byte(servo_id, 11)
        print(f"  Temp Limit:          {v} C")

        v, _ = self.read_byte(servo_id, 12)
        print(f"  Min Voltage:         {v / 10:.1f} V")

        v, _ = self.read_byte(servo_id, 13)
        print(f"  Max Voltage:         {v / 10:.1f} V")

        v, _ = self.read_word(servo_id, 14)
        print(
            f"  Max Torque:          {v} ({v / 1023 * 100:.0f}%)"
            if v
            else "  Max Torque:          ?"
        )

        v, _ = self.read_byte(servo_id, 16)
        status_labels = {0: "None", 1: "Read only", 2: "All"}
        print(f"  Status Return Level: {v} ({status_labels.get(v, '?')})")

        v, _ = self.read_byte(servo_id, 17)
        print(f"  Alarm LED:           {v:#04x}")

        v, _ = self.read_byte(servo_id, 18)
        print(f"  Alarm Shutdown:      {v:#04x}")
        if v:
            self.decode_error(v, prefix="     Shutdown on: ")

        # --- RAM (runtime state) ---
        print("\n  --- RAM (Runtime State) ---")

        v, _ = self.read_byte(servo_id, 24)
        print(f"  Torque Enable:       {v} ({'ON' if v else 'OFF'})")

        v, _ = self.read_byte(servo_id, 25)
        print(f"  LED:                 {v} ({'ON' if v else 'OFF'})")

        v, _ = self.read_byte(servo_id, 26)
        print(f"  CW Compliance Margin:  {v}")
        v, _ = self.read_byte(servo_id, 27)
        print(f"  CCW Compliance Margin: {v}")
        v, _ = self.read_byte(servo_id, 28)
        print(f"  CW Compliance Slope:   {v}")
        v, _ = self.read_byte(servo_id, 29)
        print(f"  CCW Compliance Slope:  {v}")

        goal, _ = self.read_word(servo_id, 30)
        print(f"  Goal Position:       {goal} ({self.raw_to_deg(goal):+.1f} deg)")

        speed, _ = self.read_word(servo_id, 32)
        print(f"  Moving Speed:        {speed}")

        tl, _ = self.read_word(servo_id, 34)
        print(
            f"  Torque Limit:        {tl} ({tl / 1023 * 100:.0f}%)"
            if tl is not None
            else "  Torque Limit:        ?"
        )

        pos, _ = self.read_word(servo_id, 36)
        print(f"  Present Position:    {pos} ({self.raw_to_deg(pos):+.1f} deg)")

        spd, _ = self.read_word(servo_id, 38)
        # Speed bit 10 is direction
        if spd is not None:
            direction = "CCW" if spd & 0x400 else "CW"
            spd_val = spd & 0x3FF
            print(f"  Present Speed:       {spd_val} ({direction})")

        load, _ = self.read_word(servo_id, 40)
        if load is not None:
            load_dir = "CCW" if load & 0x400 else "CW"
            load_val = load & 0x3FF
            print(
                f"  Present Load:        {load_val} ({load_dir}, {load_val / 1023 * 100:.0f}%)"
            )

        volt, _ = self.read_byte(servo_id, 42)
        print(
            f"  Present Voltage:     {volt / 10:.1f} V"
            if volt
            else "  Present Voltage:     ?"
        )

        temp, _ = self.read_byte(servo_id, 43)
        print(f"  Present Temperature: {temp} C")

        v, _ = self.read_byte(servo_id, 44)
        print(f"  Registered:          {v}")

        v, _ = self.read_byte(servo_id, 46)
        print(f"  Moving:              {v} ({'YES' if v else 'no'})")

        v, _ = self.read_byte(servo_id, 47)
        print(f"  Lock:                {v} ({'LOCKED' if v else 'unlocked'})")

        punch, _ = self.read_word(servo_id, 48)
        print(f"  Punch:               {punch}")

        # --- Summary ---
        print("\n  --- SUMMARY ---")
        if cw_limit == 0 and ccw_limit == 0:
            print("  !! WHEEL MODE - Servo ignores position commands!")
            print("     Fix: Set CW limit=0, CCW limit=1023")

        v, _ = self.read_byte(servo_id, 24)
        if not v:
            print("  !! TORQUE DISABLED - Servo will not move!")

        tl, _ = self.read_word(servo_id, 34)
        if tl is not None and tl == 0:
            print("  !! TORQUE LIMIT = 0 - Servo has no power!")

        if goal is not None and pos is not None:
            diff = abs(goal - pos)
            if diff > 10:
                print(f"  !! Position error: goal={goal} actual={pos} diff={diff}")
                print("     Servo is not reaching its target!")

        if temp and temp > 60:
            print(f"  !! HIGH TEMPERATURE: {temp}C")

        if volt:
            if volt < 95:
                print(f"  !! LOW VOLTAGE: {volt / 10:.1f}V")
            elif volt > 140:
                print(f"  !! HIGH VOLTAGE: {volt / 10:.1f}V")

        print()
        return True

    def decode_error(self, error_byte, prefix="     "):
        errors = []
        if error_byte & 0x01:
            errors.append("Input Voltage Error")
        if error_byte & 0x02:
            errors.append("Angle Limit Error")
        if error_byte & 0x04:
            errors.append("Overheating Error")
        if error_byte & 0x08:
            errors.append("Range Error")
        if error_byte & 0x10:
            errors.append("Checksum Error")
        if error_byte & 0x20:
            errors.append("Overload Error")
        if error_byte & 0x40:
            errors.append("Instruction Error")
        if errors:
            print(f"{prefix}Errors: {', '.join(errors)}")

    def raw_to_deg(self, raw):
        if raw is None:
            return 0
        return (raw - 512) / 3.41

    def fix_servo(self, servo_id):
        """Attempt to fix common issues."""
        print(f"\n  --- Attempting fixes for servo ID {servo_id} ---\n")

        # 1. Clear errors by toggling torque
        print("  1. Disabling torque to clear errors...")
        self.write_byte(servo_id, 24, 0)  # torque off
        self.write_byte(servo_id, 25, 0)  # LED off
        time.sleep(0.3)

        # 2. Set joint mode
        print("  2. Setting joint mode (CW=0, CCW=1023)...")
        self.write_word(servo_id, 6, 0)  # CW limit
        time.sleep(0.05)
        self.write_word(servo_id, 8, 1023)  # CCW limit
        time.sleep(0.05)

        # 3. Set max torque
        print("  3. Setting max torque (1023)...")
        self.write_word(servo_id, 14, 1023)  # max torque EEPROM
        time.sleep(0.05)
        self.write_word(servo_id, 34, 1023)  # torque limit RAM
        time.sleep(0.05)

        # 4. Set compliance
        print("  4. Setting compliance margins=1, slopes=32...")
        self.write_byte(servo_id, 26, 1)  # CW margin
        self.write_byte(servo_id, 27, 1)  # CCW margin
        self.write_byte(servo_id, 28, 32)  # CW slope
        self.write_byte(servo_id, 29, 32)  # CCW slope
        time.sleep(0.05)

        # 5. Unlock EEPROM
        print("  5. Unlocking EEPROM...")
        self.write_byte(servo_id, 47, 0)
        time.sleep(0.05)

        # 6. Enable torque
        print("  6. Enabling torque...")
        self.write_byte(servo_id, 24, 1)
        time.sleep(0.3)

        # 7. Set speed and move to center
        print("  7. Setting speed=200, moving to center (512)...")
        self.write_word(servo_id, 32, 200)
        time.sleep(0.05)
        self.write_word(servo_id, 30, 512)
        time.sleep(2.0)

        # 8. Read back position
        pos, _ = self.read_word(servo_id, 36)
        goal, _ = self.read_word(servo_id, 30)
        print(
            f"  8. Goal={goal}  Present={pos}  Diff={abs(goal - pos) if goal and pos else '?'}"
        )

        if pos is not None and goal is not None and abs(goal - pos) < 20:
            print("\n  SUCCESS! Servo is responding to position commands.")
        else:
            print("\n  STILL NOT MOVING. Possible hardware fault.")

        print()


def main():
    parser = argparse.ArgumentParser(description="AX-12A Servo Diagnostic")
    parser.add_argument("--id", type=int, default=2, help="Servo ID to diagnose")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baud", type=int, default=1000000, help="Baudrate")
    parser.add_argument("--fix", action="store_true", help="Attempt to fix issues")
    parser.add_argument(
        "--scan", action="store_true", help="Scan all IDs 0-20 to find servos"
    )
    args = parser.parse_args()

    diag = AX12Diag(args.port, args.baud)

    if args.scan:
        print("\n  Scanning servo IDs 0-20...")
        for sid in range(21):
            alive, err = diag.ping(sid)
            if alive:
                print(f"    ID {sid:3d}: FOUND (error={err})")
        print()
    else:
        diag.read_all(args.id)
        if args.fix:
            diag.fix_servo(args.id)
            print("  Re-reading after fix:")
            diag.read_all(args.id)

    diag.close()


if __name__ == "__main__":
    main()
