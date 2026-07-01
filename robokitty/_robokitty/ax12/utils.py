"""
Unit conversion helpers for AX-12A servo values.

All functions are pure / stateless — no imports from the rest of the package.
"""

from ... import _log as logger


# AX-12A physical range
_MAX_POSITION = 1023
_MIN_DEGREES = 0.0
_MAX_DEGREES = 300.0

# Byte packing
_LOW_BYTE_MASK: int = 0xFF
_HIGH_BYTE_SHIFT: int = 8


def degrees_to_position(degrees: float) -> int:
    """
    Convert an angle in degrees to a raw position value.

    Parameters
    ----------
    degrees : float
        Angle in [0, 300]. Values outside this range are clamped.

    Returns
    -------
    int
        Position value in [0, 1023].
    """
    if not (0.0 <= degrees <= _MAX_DEGREES):
        logger.warning(
            "Target angle %.2f° is outside native AX-12A 300-degree arc limits.",
            degrees,
        )

    degrees = max(0.0, min(_MAX_DEGREES, degrees))
    return round(degrees / _MAX_DEGREES * _MAX_POSITION)


def position_to_degrees(position: int) -> float:
    """
    Convert a raw position value to degrees.

    Parameters
    ----------
    position : int
        Position value in [0, 1023]. Values outside this range are clamped.

    Returns
    -------
    float
        Angle in [0.0, 300.0].
    """
    position = max(0, min(_MAX_POSITION, position))
    return round(position / _MAX_POSITION * _MAX_DEGREES, 2)


def rpm_to_speed(rpm: float) -> int:
    """
    Convert a target RPM to a raw speed value.

    The AX-12A does approximately 0.111 RPM per unit at 1 = slowest.
    Unit 0 means "maximum speed, no limit".

    Parameters
    ----------
    rpm : float
        Desired speed in RPM. 0 means no limit.

    Returns
    -------
    int
        Speed value in [0, 1023].
    """
    if rpm <= 0:
        return 0
    return max(1, min(_MAX_POSITION, round(rpm / 0.111)))


def speed_to_rpm(speed: int) -> float:
    """
    Convert a raw speed value to approximate RPM.

    Parameters
    ----------
    speed : int
        Speed value in [0, 1023]. 0 is treated as maximum (no limit).

    Returns
    -------
    float
        Approximate RPM (returns 0.0 when speed == 0).
    """
    if speed == 0:
        return 0.0
    return round(speed * 0.111, 3)


def voltage_raw_to_volts(raw: int) -> float:
    """Convert the PRESENT_VOLTAGE register byte to volts (divide by 10)."""
    return raw / 10.0


def split_word(value: int) -> tuple[int, int]:
    """Split a 16-bit value into (low_byte, high_byte)."""
    return value & 0xFF, (value >> 8) & 0xFF


def join_word(lo: int, hi: int) -> int:
    """Combine low and high bytes into a 16-bit value."""
    return lo | (hi << 8)


def to_le_bytes(position: int) -> list[int]:
    """Pack a servo position into a little-endian two-byte list."""
    return [position & _LOW_BYTE_MASK, (position >> _HIGH_BYTE_SHIFT) & _LOW_BYTE_MASK]


def clamp_position(position: int) -> int:
    """Clamp a raw servo position to the valid hardware register range."""
    return max(0, min(_MAX_POSITION, position))
