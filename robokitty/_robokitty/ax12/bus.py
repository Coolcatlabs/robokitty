"""
SerialBus – low-level serial transport for the Dynamixel bus.

Owns the serial.Serial instance and handles synchronized streaming raw I/O.
Adapts stream chunk processing safely into higher-level StatusPacket parsing.
"""

import serial
from .status import StatusPacket, StatusPacketParser


class SerialBus:
    """
    Wraps a pyserial port and provides synchronized send / receive / transact primitives.

    Parameters
    ----------
    port : str
        Serial port path, e.g. '/dev/ttyUSB0' or 'COM3'.
    baud : int
        Baud rate matching the servo firmware (default 1 000 000).
    timeout : float
        Read timeout in seconds (default 0.05).
    """

    def __init__(self, port: str, baud: int, timeout: float):
        self._serial = serial.Serial(port, baudrate=baud, timeout=timeout)

    # -- Lifecycle -----------------------------------------------------------
    def close(self) -> None:
        """Close the underlying serial connection safely."""
        if self._serial.is_open:
            self._serial.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    # -- Primitives ----------------------------------------------------------
    def send(self, packet: bytes) -> None:
        """Write raw bytes directly to the physical bus."""
        self._serial.write(packet)

    def recv(self) -> StatusPacket:
        """
        Read and dynamically frame a single status packet from the bus stream.

        This method reads byte-by-byte until a valid double 0xFF header is found,
        then reads the metadata length field to fetch the exact remainder
        of the frame safely.

        Raises
        ------
        TimeoutError
            If reading headers or the dynamic frame payload times out.
        ValueError
            If the incoming data fails internal structural or checksum parsing.
        """
        # Synchronize Stream: Find consecutive [0xFF, 0xFF] header bytes
        header_match = 0
        while header_match < 2:
            byte = self._serial.read(1)
            if not byte:
                raise TimeoutError(
                    "Timeout waiting for packet header sync (0xFF 0xFF)."
                )

            if byte == b"\xff":
                header_match += 1
            else:
                header_match = 0

        # Extract fixed metadata: ID (1 byte) and LENGTH (1 byte)
        metadata = self._serial.read(2)
        if len(metadata) < 2:
            raise TimeoutError("Timeout reading packet ID and Length fields.")

        servo_id = metadata[0]
        length = metadata[1]

        # Read Remaining Frame Body: Length field corresponds to (ERROR + PARAMS + CHECKSUM)
        remaining_bytes = self._serial.read(length)
        if len(remaining_bytes) < length:
            raise TimeoutError(
                f"Timeout reading packet payload. Expected {length} bytes, got {len(remaining_bytes)}."
            )

        # Assemble standard package packet structure back together for Parser consumption
        full_packet = bytes([0xFF, 0xFF, servo_id, length]) + remaining_bytes
        return StatusPacketParser.parse(full_packet)

    def transact(self, packet: bytes) -> StatusPacket:
        """
        Flush the input buffer, transmit a command, and return parsed status.

        Use this for single-cast instructions that mandate status replies.
        Do not use for SYNC_WRITE instructions or Broadcast ID (0xFE) commands.
        """
        self._serial.reset_input_buffer()
        self.send(packet)
        return self.recv()
