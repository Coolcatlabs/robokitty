from enum import Enum

from .config import ServoJointConfig

# Serial config
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 1000000


class LegID(Enum):
    FL = "FrontLeft"
    FR = "FrontRight"
    RL = "RearLeft"
    RR = "RearRight"


class GaitType(Enum):
    TROT = "trot"
    WALK = "walk"
    PACE = "pace"


# AX-12A constants
AX12_CENTER = 512
AX12_DEG_TO_UNITS = 1023.0 / 300.0  # ~3.41 units per degree

# ============================================================================
# ROBOKITTY SERVO MAP
# ============================================================================
# Robot front reversed from original build. Physical legs keep their servo IDs.
# Only the IK solution (front vs rear) changes based on new travel direction.
# Inversions unchanged from original calibration.

LEG_SERVO_CONFIG: dict[LegID, list[ServoJointConfig]] = {
    # [Shoulder/Coxa, Femur, Tibia/Leg]
    # Offsets from --calibrate 2026-02-26
    LegID.FL: [
        ServoJointConfig(servo_id=8, offset_deg=+0.9, inverted=True),  # Shoulder
        ServoJointConfig(servo_id=10, offset_deg=-33.7, inverted=False),  # Femur
        ServoJointConfig(servo_id=0, offset_deg=+48.1, inverted=False),  # Leg
    ],
    LegID.FR: [
        ServoJointConfig(servo_id=11, offset_deg=+6.1, inverted=False),  # Shoulder
        ServoJointConfig(servo_id=9, offset_deg=-47.2, inverted=True),  # Femur
        ServoJointConfig(servo_id=7, offset_deg=+42.3, inverted=True),  # Leg
    ],
    LegID.RL: [
        ServoJointConfig(servo_id=5, offset_deg=+2.6, inverted=False),  # Shoulder
        ServoJointConfig(servo_id=3, offset_deg=+16.4, inverted=True),  # Femur
        ServoJointConfig(servo_id=6, offset_deg=-12.0, inverted=True),  # Leg
    ],
    LegID.RR: [
        ServoJointConfig(servo_id=2, offset_deg=-38.4, inverted=True),  # Shoulder
        ServoJointConfig(servo_id=1, offset_deg=+15.6, inverted=False),  # Femur
        ServoJointConfig(servo_id=4, offset_deg=-26.4, inverted=False),  # Leg
    ],
}


GAIT_PHASES = {
    GaitType.TROT: {
        LegID.FL: 0.0,
        LegID.FR: 0.5,
        LegID.RL: 0.5,
        LegID.RR: 0.0,
    },
    GaitType.WALK: {
        LegID.FL: 0.5,
        LegID.FR: 0.25,
        LegID.RL: 0.0,
        LegID.RR: 0.75,
    },
    GaitType.PACE: {
        LegID.FL: 0.0,
        LegID.FR: 0.5,
        LegID.RL: 0.0,
        LegID.RR: 0.5,
    },
}
