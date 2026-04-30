"""
Dynamixel v1.0 Protocol Base Layer.

This module provides the core wire-protocol constants and checksum validation
utilities used across both outgoing instructions and incoming status streams.
It acts as the single source of truth for the physical-layer layout.
"""

from typing import Sequence


class DynamixelV1Protocol:
    """Encapsulates common constants and low-level math for Dynamixel v1.0."""

    HEADER = bytes([0xFF, 0xFF])
    BROADCAST_ID = 0xFE

    MIN_PACKET_LENGTH = 6  # FF FF ID LENGTH ERROR/INST CHKSUM

    @classmethod
    def calculate_checksum(
        cls, id_: int, length: int, command_byte: int, params: Sequence[int]
    ) -> int:
        """Calculates the standard Dynamixel 1’s complement checksum."""
        total = id_ + length + command_byte + sum(params)
        return (~total) & 0xFF
