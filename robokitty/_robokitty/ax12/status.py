"""
Dynamixel Status Packet Telemetry and Parsing.

This module processes incoming byte streams received from actuators. It provides
error-flag decoding, immutable state containers for read-back values, and strict
checksum verification for complete hardware telemetry frames.
"""

from enum import IntFlag, auto
from dataclasses import dataclass, field
from .protocol import DynamixelV1Protocol


class ErrorFlag(IntFlag):
    NONE = 0
    INPUT_VOLTAGE = auto()  # 0x01
    ANGLE_LIMIT = auto()  # 0x02
    OVERHEATING = auto()  # 0x04
    RANGE = auto()  # 0x08
    CHECKSUM = auto()  # 0x10
    OVERLOAD = auto()  # 0x20
    INSTRUCTION = auto()  # 0x40

    @classmethod
    def decode_to_list(cls, byte: int) -> list[str]:
        """Decodes a raw byte into human-readable error messages."""
        labels = {
            cls.INPUT_VOLTAGE: "Input voltage out of range",
            cls.ANGLE_LIMIT: "Angle limit error",
            cls.OVERHEATING: "Overheating",
            cls.RANGE: "Range error",
            cls.CHECKSUM: "Checksum error",
            cls.OVERLOAD: "Overload",
            cls.INSTRUCTION: "Instruction error",
        }
        active_flags = cls(byte)
        return [msg for flag, msg in labels.items() if flag in active_flags]


@dataclass(frozen=True)
class StatusPacket:
    id: int
    error: int
    params: bytes = b""
    errors: list[str] = field(default_factory=list, init=False)

    def __post_init__(self):
        # Using object.__setattr__ because the dataclass is frozen for immutability
        object.__setattr__(self, "errors", ErrorFlag.decode_to_list(self.error))

    @property
    def ok(self) -> bool:
        return self.error == ErrorFlag.NONE

    def word(self, offset: int = 0) -> int:
        """Interpret two consecutive param bytes as a little-endian word."""
        if offset + 1 >= len(self.params):
            raise IndexError("Parameter offset out of bounds for word reading.")
        return self.params[offset] | (self.params[offset + 1] << 8)


class StatusPacketParser(DynamixelV1Protocol):
    """Handles verification and extraction of data from received Status buffers."""

    @classmethod
    def parse(cls, data: bytes) -> StatusPacket:
        if len(data) < cls.MIN_PACKET_LENGTH:
            raise ValueError(f"Packet too short ({len(data)} bytes)")

        if data[0] != 0xFF or data[1] != 0xFF:
            raise ValueError("Missing header bytes 0xFF 0xFF")

        id_ = data[2]
        length = data[3]

        if len(data) < length + 4:
            raise ValueError(
                f"Incomplete packet: expected {length + 4} bytes, got {len(data)}"
            )

        error = data[4]

        param_len = length - 2
        params = bytes(data[5 : 5 + param_len])
        chk_rx = data[5 + param_len]

        chk_calc = cls.calculate_checksum(id_, length, error, params)
        if chk_rx != chk_calc:
            raise ValueError(
                f"Checksum mismatch: received 0x{chk_rx:02X}, calculated 0x{chk_calc:02X}"
            )

        return StatusPacket(id=id_, error=error, params=params)
