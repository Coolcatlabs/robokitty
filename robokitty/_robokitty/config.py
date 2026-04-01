"""CONFIGURATION - ROBOKITTY ACTUAL MEASUREMENTS."""

from dataclasses import dataclass


@dataclass
class LegDimensions:
    """Leg segment lengths in mm (shaft-center to shaft-center, shaft to foot)."""

    coxa_length: float = 58.0  # Shoulder shaft to femur pivot
    femur_length: float = 68.0  # Femur pivot to knee pivot
    tibia_length: float = 75.0  # Knee pivot to foot contact point


@dataclass
class BodyDimensions:
    """Distance from body CENTER to coxa shaft center in mm."""

    half_length: float = 171.0  # Center to front/rear coxa shaft
    half_width: float = 101.5  # Center to left/right coxa shaft


@dataclass
class GaitConfig:
    """Gait timing and geometry - tuned for 1.65kg on carpet."""

    step_height: float = 25.0  # Foot lift mm (reliable clearance)
    step_length: float = 80.0  # Forward travel per step mm (big enough for grip)
    cycle_time: float = 1.6  # Full gait cycle seconds (faster for momentum)
    duty_factor: float = 0.75  # 0.75=walk/creep (1 foot up, 3 on ground)
    body_height: float = (
        110.0  # Standing height mm (lower = more knee bend = more grip)
    )
    update_rate_hz: float = 50.0  # Control loop frequency


@dataclass
class ServoJointConfig:
    """Configuration for one servo/joint."""

    servo_id: int
    offset_deg: float = 0.0  # Mechanical offset (tune during calibration)
    inverted: bool = False  # True = positive angle moves servo negative
    min_deg: float = -150.0  # Software limit (AX-12A range is +-150)
    max_deg: float = 150.0  # Software limit
