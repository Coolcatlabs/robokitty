"""
AX-12A Control Table Register Mappings.

This module provides the structural hardware offsets for the AX-12A servo
memory space, split cleanly into persistent non-volatile configuration (EEPROM)
and runtime state modification zones (RAM).
"""


class Register:
    """EEPROM and RAM register addresses for the AX-12A."""

    # EEPROM
    MODEL_NUMBER_L = 0x00
    MODEL_NUMBER_H = 0x01
    VERSION = 0x02
    ID = 0x03
    BAUD_RATE = 0x04
    RETURN_DELAY_TIME = 0x05
    CW_ANGLE_LIMIT_L = 0x06
    CW_ANGLE_LIMIT_H = 0x07
    CCW_ANGLE_LIMIT_L = 0x08
    CCW_ANGLE_LIMIT_H = 0x09
    TEMPERATURE_LIMIT = 0x0B
    MIN_VOLTAGE_LIMIT = 0x0C
    MAX_VOLTAGE_LIMIT = 0x0D
    MAX_TORQUE_L = 0x0E
    MAX_TORQUE_H = 0x0F
    STATUS_RETURN_LEVEL = 0x10
    ALARM_LED = 0x11
    ALARM_SHUTDOWN = 0x12
    DOWN_CALIBRATION_L = 0x14
    DOWN_CALIBRATION_R = 0x15
    UP_CALIBRATION_L = 0x16
    UP_CALIBRATION_R = 0x17

    # RAM
    TORQUE_ENABLE = 0x18
    LED = 0x19
    CW_COMPLIANCE_MARGIN = 0x1A
    CCW_COMPLIANCE_MARGIN = 0x1B
    CW_COMPLIANCE_SLOPE = 0x1C
    CCW_COMPLIANCE_SLOPE = 0x1D
    GOAL_POSITION_L = 0x1E
    GOAL_POSITION_H = 0x1F
    MOVING_SPEED_L = 0x20
    MOVING_SPEED_H = 0x21
    TORQUE_LIMIT_L = 0x22
    TORQUE_LIMIT_H = 0x23
    PRESENT_POSITION_L = 0x24
    PRESENT_POSITION_H = 0x25
    PRESENT_SPEED_L = 0x26
    PRESENT_SPEED_H = 0x27
    PRESENT_LOAD_L = 0x28
    PRESENT_LOAD_H = 0x29
    PRESENT_VOLTAGE = 0x2A
    PRESENT_TEMPERATURE = 0x2B
    REGISTERED_INSTRUCTION = 0x2C
    MOVING = 0x2E
    LOCK = 0x2F
    PUNCH_L = 0x30
    PUNCH_H = 0x31
