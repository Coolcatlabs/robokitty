"""
Dynamixel Instruction Packet Factory.

This module handles the creation, structure, and formatting of all outgoing
command strings (Instruction Packets) sent from the host controller to the
actuators, including single writes, multi-byte reads, and broadcast sync writes.
"""

from typing import Sequence
from .protocol import DynamixelV1Protocol


class Instruction:
    PING = 0x01
    READ_DATA = 0x02
    WRITE_DATA = 0x03
    REG_WRITE = 0x04
    ACTION = 0x05
    RESET = 0x06
    SYNC_WRITE = 0x83


class InstructionPacketBuilder(DynamixelV1Protocol):
    """Factory for building outgoing instruction packets."""

    @classmethod
    def _build(cls, id_: int, instruction: int, params: Sequence[int] = ()) -> bytes:
        length = len(params) + 2
        checksum = cls.calculate_checksum(id_, length, instruction, params)
        return cls.HEADER + bytes([id_, length, instruction, *params, checksum])

    @classmethod
    def ping(cls, id_: int) -> bytes:
        return cls._build(id_, Instruction.PING)

    @classmethod
    def read(cls, id_: int, address: int, length: int) -> bytes:
        return cls._build(id_, Instruction.READ_DATA, [address, length])

    @classmethod
    def write(cls, id_: int, address: int, data: Sequence[int]) -> bytes:
        return cls._build(id_, Instruction.WRITE_DATA, [address, *data])

    @classmethod
    def write_word(cls, id_: int, address: int, value: int) -> bytes:
        return cls.write(id_, address, [value & 0xFF, (value >> 8) & 0xFF])

    @classmethod
    def reg_write(cls, id_: int, address: int, data: Sequence[int]) -> bytes:
        return cls._build(id_, Instruction.REG_WRITE, [address, *data])

    @classmethod
    def action(cls, id_: int = DynamixelV1Protocol.BROADCAST_ID) -> bytes:
        return cls._build(id_, Instruction.ACTION)

    @classmethod
    def reset(cls, id_: int) -> bytes:
        return cls._build(id_, Instruction.RESET)

    @classmethod
    def sync_write(
        cls, address: int, data_per_servo: dict[int, Sequence[int]]
    ) -> bytes:
        if not data_per_servo:
            raise ValueError("data_per_servo must not be empty")

        data_len = len(next(iter(data_per_servo.values())))
        params: list[int] = [address, data_len]

        for servo_id, data in data_per_servo.items():
            if len(data) != data_len:
                raise ValueError(
                    f"Servo {servo_id}: expected {data_len} bytes, got {len(data)}"
                )
            params += [servo_id, *data]

        return cls._build(cls.BROADCAST_ID, Instruction.SYNC_WRITE, params)
