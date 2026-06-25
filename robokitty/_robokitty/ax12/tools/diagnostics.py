"""
tools/diagnostics.py – AX-12A full register diagnostic.
"""

from ..ax12 import AX12Interface, Register
from ..status import ErrorFlag
from ..utils import position_to_degrees
from .utils import W, header, row, safe_read


def get_servo_metrics(ax: AX12Interface, servo_id: int) -> bool:
    """
    Print a full diagnostic report for a single AX-12A servo.

    Returns True if the servo responded, False otherwise.
    """
    print()
    print(f"  {'═' * (W - 4)}")
    print(f"  {'AX-12A Diagnostic':^{W - 4}}")
    print(f"  {'Servo ID ' + str(servo_id):^{W - 4}}")
    print(f"  {'═' * (W - 4)}")

    if not ax.ping(servo_id):
        print(f"\n  !!  NO RESPONSE from servo ID {servo_id}")
        print("      Check wiring, power, and baud rate.")
        return False
    print(f"\n  {'Ping':.<32} OK")

    warnings: list[str] = []

    # ── EEPROM ───────────────────────────────────────────────────────────────
    header("EEPROM  (persistent settings)")

    model = safe_read(
        ax.read_word, servo_id, Register.MODEL_NUMBER_L, label="Model Number"
    )
    if model is not None:
        _label = "AX-12A" if model == 12 else "UNKNOWN"
        row("Model Number", f"{model}  ({_label})")

    fw = safe_read(ax.read_byte, servo_id, Register.VERSION, label="Firmware")
    if fw is not None:
        row("Firmware Version", str(fw))

    id_val = safe_read(ax.read_byte, servo_id, Register.ID, label="ID")
    if id_val is not None:
        row("ID", str(id_val))

    baud_raw = safe_read(ax.read_byte, servo_id, Register.BAUD_RATE, label="Baud Rate")
    if baud_raw is not None:
        baud_bps = int(2_000_000 / (baud_raw + 1))
        row("Baud Rate", f"{baud_raw}  ({baud_bps:,} bps)")

    delay = safe_read(
        ax.read_byte, servo_id, Register.RETURN_DELAY_TIME, label="Return Delay"
    )
    if delay is not None:
        row("Return Delay", f"{delay}  ({delay * 2} µs)")

    cw_limit = safe_read(
        ax.read_word, servo_id, Register.CW_ANGLE_LIMIT_L, label="CW Limit"
    )
    ccw_limit = safe_read(
        ax.read_word, servo_id, Register.CCW_ANGLE_LIMIT_L, label="CCW Limit"
    )
    if cw_limit is not None and ccw_limit is not None:
        if cw_limit == 0 and ccw_limit == 0:
            mode = "WHEEL MODE"
            warnings.append(
                "Wheel mode active — position commands are ignored. "
                "Set CCW limit to 1023 to restore joint mode."
            )
        elif cw_limit == 0 and ccw_limit == 1023:
            mode = "Joint  (full range)"
        else:
            mode = f"Joint  (limited {cw_limit}–{ccw_limit})"
        row("CW / CCW Limits", f"{cw_limit} / {ccw_limit}  →  {mode}")

    temp_limit = safe_read(
        ax.read_byte, servo_id, Register.TEMPERATURE_LIMIT, label="Temp Limit"
    )
    if temp_limit is not None:
        row("Temp Limit", f"{temp_limit} °C")

    min_v = safe_read(
        ax.read_byte, servo_id, Register.MIN_VOLTAGE_LIMIT, label="Min Voltage"
    )
    max_v = safe_read(
        ax.read_byte, servo_id, Register.MAX_VOLTAGE_LIMIT, label="Max Voltage"
    )
    if min_v is not None and max_v is not None:
        row("Voltage Range", f"{min_v / 10:.1f} V – {max_v / 10:.1f} V")

    max_torque = safe_read(
        ax.read_word, servo_id, Register.MAX_TORQUE_L, label="Max Torque"
    )
    if max_torque is not None:
        row("Max Torque", f"{max_torque}  ({max_torque / 1023 * 100:.0f}%)")

    srl = safe_read(
        ax.read_byte, servo_id, Register.STATUS_RETURN_LEVEL, label="Status Return"
    )
    if srl is not None:
        srl_label = {0: "None", 1: "Read only", 2: "All"}.get(srl, "?")
        row("Status Return Level", f"{srl}  ({srl_label})")

    alarm_led = safe_read(ax.read_byte, servo_id, Register.ALARM_LED, label="Alarm LED")
    if alarm_led is not None:
        row("Alarm LED", f"{alarm_led:#04x}")

    alarm_shutdown = safe_read(
        ax.read_byte, servo_id, Register.ALARM_SHUTDOWN, label="Alarm Shutdown"
    )
    if alarm_shutdown is not None:
        flags = ErrorFlag.decode_to_list(alarm_shutdown)
        row(
            "Alarm Shutdown",
            f"{alarm_shutdown:#04x}  ({', '.join(flags) if flags else 'none'})",
        )

    # ── RAM ──────────────────────────────────────────────────────────────────
    header("RAM  (runtime state)")

    torque_en = safe_read(
        ax.read_byte, servo_id, Register.TORQUE_ENABLE, label="Torque Enable"
    )
    if torque_en is not None:
        warn = "Torque is disabled — servo will not move." if not torque_en else ""
        if warn:
            warnings.append(warn)
        row("Torque Enable", "ON" if torque_en else "OFF", warn=warn)

    led = safe_read(ax.read_byte, servo_id, Register.LED, label="LED")
    if led is not None:
        row("LED", "ON" if led else "off")

    cw_margin = safe_read(
        ax.read_byte, servo_id, Register.CW_COMPLIANCE_MARGIN, label="CW Margin"
    )
    ccw_margin = safe_read(
        ax.read_byte, servo_id, Register.CCW_COMPLIANCE_MARGIN, label="CCW Margin"
    )
    cw_slope = safe_read(
        ax.read_byte, servo_id, Register.CW_COMPLIANCE_SLOPE, label="CW Slope"
    )
    ccw_slope = safe_read(
        ax.read_byte, servo_id, Register.CCW_COMPLIANCE_SLOPE, label="CCW Slope"
    )
    if None not in (cw_margin, ccw_margin, cw_slope, ccw_slope):
        row("Compliance Margin", f"CW {cw_margin}  /  CCW {ccw_margin}")
        row("Compliance Slope", f"CW {cw_slope}  /  CCW {ccw_slope}")

    goal = safe_read(
        ax.read_word, servo_id, Register.GOAL_POSITION_L, label="Goal Position"
    )
    if goal is not None:
        row("Goal Position", f"{goal}  ({position_to_degrees(goal):.1f}°)")

    moving_speed = safe_read(
        ax.read_word, servo_id, Register.MOVING_SPEED_L, label="Moving Speed"
    )
    if moving_speed is not None:
        row(
            "Moving Speed", "max (no limit)" if moving_speed == 0 else str(moving_speed)
        )

    torque_limit = safe_read(
        ax.read_word, servo_id, Register.TORQUE_LIMIT_L, label="Torque Limit"
    )
    if torque_limit is not None:
        warn = "Torque limit is zero — servo has no power." if torque_limit == 0 else ""
        if warn:
            warnings.append(warn)
        row(
            "Torque Limit",
            f"{torque_limit}  ({torque_limit / 1023 * 100:.0f}%)",
            warn=warn,
        )

    pos = safe_read(ax.get_position, servo_id, label="Present Position")
    if pos is not None:
        row("Present Position", f"{pos}  ({position_to_degrees(pos):.1f}°)")

    if goal is not None and pos is not None:
        diff = abs(goal - pos)
        if diff > 10:
            warn = (
                f"Not reaching target — goal {goal}, actual {pos}, error {diff} steps."
            )
            warnings.append(warn)
            row("Position Error", f"{diff} steps", warn=warn)

    spd_raw = safe_read(ax.get_speed, servo_id, label="Present Speed")
    if spd_raw is not None:
        row(
            "Present Speed",
            f"{spd_raw & 0x3FF}  ({'CCW' if spd_raw & 0x400 else 'CW'})",
        )

    load_raw = safe_read(ax.get_load, servo_id, label="Present Load")
    if load_raw is not None:
        load_val = load_raw & 0x3FF
        row(
            "Present Load",
            f"{load_val}  ({'CCW' if load_raw & 0x400 else 'CW'},  {load_val / 1023 * 100:.0f}%)",
        )

    voltage = safe_read(ax.get_voltage, servo_id, label="Voltage")
    if voltage is not None:
        warn = ""
        if voltage < 9.5:
            warn = f"Low supply voltage ({voltage:.1f} V). Check power source."
            warnings.append(warn)
        elif voltage > 14.0:
            warn = f"High supply voltage ({voltage:.1f} V). Risk of damage."
            warnings.append(warn)
        row("Present Voltage", f"{voltage:.1f} V", warn=warn)

    temp = safe_read(ax.get_temperature, servo_id, label="Temperature")
    if temp is not None:
        warn = (
            f"High temperature ({temp} °C). Check load and duty cycle."
            if temp > 60
            else ""
        )
        if warn:
            warnings.append(warn)
        row("Present Temperature", f"{temp} °C", warn=warn)

    moving = safe_read(ax.is_moving, servo_id, label="Moving")
    if moving is not None:
        row("Moving", "YES" if moving else "no")

    registered = safe_read(
        ax.read_byte, servo_id, Register.REGISTERED_INSTRUCTION, label="Registered"
    )
    if registered is not None:
        row("Registered Instruction", str(registered))

    lock = safe_read(ax.read_byte, servo_id, Register.LOCK, label="Lock")
    if lock is not None:
        row(
            "Lock",
            "LOCKED" if lock else "unlocked",
            warn="EEPROM is locked." if lock else "",
        )

    punch = safe_read(ax.read_word, servo_id, Register.PUNCH_L, label="Punch")
    if punch is not None:
        row("Punch", str(punch))

    # ── Summary ──────────────────────────────────────────────────────────────
    header("Summary")

    if not warnings:
        print("  ✓   No issues detected.")
    else:
        for i, w in enumerate(warnings, 1):
            print(f"  {i:>2}.  {w}")

    print(f"\n  {'─' * (W - 4)}\n")
    return True
