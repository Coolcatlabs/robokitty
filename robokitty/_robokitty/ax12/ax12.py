"""
AX12 – High-level driver for ROBOTIS AX-12A servos.

Depends on SerialBus for transport and InstructionPacketBuilder for packet construction.
All servo-behaviour logic lives here; no raw bytes or serial calls.
"""

import time

from .instruction import InstructionPacketBuilder
from .registers import Register
from .bus import SerialBus
from .utils import degrees_to_position, position_to_degrees, split_word


class AX12Interface:
    """
    High-level interface for AX-12A Dynamixel servos.

    Parameters
    ----------
    port : str
        Serial port path (e.g. '/dev/ttyUSB0', 'COM3').
    baud : int
        Baud rate matching the servo (default 1 000 000).
    timeout : float
        Serial read timeout in seconds (default 0.05).
    """

    MIN_POSITION = 0
    MAX_POSITION = 1023
    MIN_SPEED = 0
    MAX_SPEED = 1023
    MIN_TORQUE = 0
    MAX_TORQUE = 1023

    # Servo Configuration Registers
    COMPLIANCE_MARGIN = 1
    COMPLIANCE_SLOPE = 32

    def __init__(self, port: str, baud: int, timeout: float = 0.05):
        self._bus = SerialBus(port, baud, timeout)

    # -- Lifecycle -----------------------------------------------------------
    def close(self) -> None:
        """Close the underlying serial bus."""
        self._bus.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    # -- Low-level register access -------------------------------------------
    def read_byte(self, id_: int, address: int) -> int:
        """Read a single byte from the specified register address."""
        pkt = InstructionPacketBuilder.read(id_, address, 1)
        status = self._bus.transact(pkt)
        return status.params[0]

    def read_word(self, id_: int, address: int) -> int:
        """Read a two-byte little-endian word from the specified register address."""
        pkt = InstructionPacketBuilder.read(id_, address, 2)
        status = self._bus.transact(pkt)
        return status.word()

    def write_byte(self, id_: int, address: int, value: int) -> None:
        """Write a single byte to the specified register address."""
        pkt = InstructionPacketBuilder.write(id_, address, [value & 0xFF])
        self._bus.transact(pkt)

    def write_word(self, id_: int, address: int, value: int) -> None:
        """Write a two-byte little-endian word to the specified register address."""
        pkt = InstructionPacketBuilder.write_word(id_, address, value)
        self._bus.transact(pkt)

    # -- Discovery -----------------------------------------------------------
    def ping(self, id_: int) -> bool:
        """Return True if a servo with the given ID responds."""
        try:
            self._bus.transact(InstructionPacketBuilder.ping(id_))
            return True
        except (TimeoutError, ValueError):
            return False

    def scan(self, id_range: range = range(1, 254)) -> list[int]:
        """Scan the bus and return a list of responding servo IDs."""
        return [id_ for id_ in id_range if self.ping(id_)]

    # -- Motion control ------------------------------------------------------
    def set_goal_position(self, id_: int, position: int) -> None:
        """Set target position [0 – 1023]."""
        position = max(self.MIN_POSITION, min(self.MAX_POSITION, position))
        self.write_word(id_, Register.GOAL_POSITION_L, position)

    def set_moving_speed(self, id_: int, speed: int) -> None:
        """Set moving speed [0 – 1023]. 0 = no limit."""
        speed = max(self.MIN_SPEED, min(self.MAX_SPEED, speed))
        self.write_word(id_, Register.MOVING_SPEED_L, speed)

    def set_torque_limit(self, id_: int, torque: int = MAX_TORQUE) -> None:
        """Set torque limit [0 – 1023]."""
        torque = max(self.MIN_TORQUE, min(self.MAX_TORQUE, torque))
        self.write_word(id_, Register.TORQUE_LIMIT_L, torque)

    def torque_enable(self, id_: int, enable: bool = True) -> None:
        """Enable or disable motor torque."""
        self.write_byte(id_, Register.TORQUE_ENABLE, int(enable))

    def move(self, id_: int, position: int, speed: int = 0) -> None:
        """Set speed (optional) then goal position in one call."""
        if speed:
            self.set_moving_speed(id_, speed)
        self.set_goal_position(id_, position)

    def move_degrees(self, id_: int, degrees: float, speed: int = 0) -> None:
        """Move to an angle in degrees [0 – 300]."""
        self.move(id_, degrees_to_position(degrees), speed)

    def sync_move(self, targets: dict[int, int], speed: int = 0) -> None:
        """
        Move multiple servos simultaneously via SYNC_WRITE.

        Parameters
        ----------
        targets : dict[servo_id -> position]
        speed : int
            Applied to all listed servos when non-zero.
        """
        if speed:
            speed_data = {id_: list(split_word(speed)) for id_ in targets}
            self._bus.send(
                InstructionPacketBuilder.sync_write(Register.MOVING_SPEED_L, speed_data)
            )

        pos_data = {id_: list(split_word(pos)) for id_, pos in targets.items()}
        self._bus.send(
            InstructionPacketBuilder.sync_write(Register.GOAL_POSITION_L, pos_data)
        )

    def wait_for_stop(
        self,
        id_: int,
        poll_interval: float = 0.02,
        timeout: float = 5.0,
    ) -> bool:
        """
        Block until the servo stops moving or timeout elapses.

        Returns True if the servo stopped in time, False on timeout.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self.is_moving(id_):
                return True
            time.sleep(poll_interval)
        return False

    # -- Status reads --------------------------------------------------------
    def get_position(self, id_: int) -> int:
        """Get the present position coordinate [0 - 1023]."""
        return self.read_word(id_, Register.PRESENT_POSITION_L)

    def get_position_degrees(self, id_: int) -> float:
        """Get the present position converted into angular degrees."""
        return position_to_degrees(self.get_position(id_))

    def get_speed(self, id_: int) -> int:
        """Get the present moving speed."""
        return self.read_word(id_, Register.PRESENT_SPEED_L)

    def get_load(self, id_: int) -> int:
        """Get the current physical load vector metric."""
        return self.read_word(id_, Register.PRESENT_LOAD_L)

    def get_voltage(self, id_: int) -> float:
        """Get present operating voltage in Volts."""
        return self.read_byte(id_, Register.PRESENT_VOLTAGE) / 10.0

    def get_temperature(self, id_: int) -> int:
        """Get the internal temperature in Degrees Celsius."""
        return self.read_byte(id_, Register.PRESENT_TEMPERATURE)

    def is_moving(self, id_: int) -> bool:
        """Check if the servo is currently moving."""
        return bool(self.read_byte(id_, Register.MOVING))

    def get_status(self, id_: int) -> dict:
        """Return a snapshot of all commonly used status fields."""
        return {
            "id": id_,
            "position": self.get_position(id_),
            "position_deg": self.get_position_degrees(id_),
            "speed": self.get_speed(id_),
            "load": self.get_load(id_),
            "voltage_V": self.get_voltage(id_),
            "temperature_C": self.get_temperature(id_),
            "moving": self.is_moving(id_),
        }

    # -- Configuration -------------------------------------------------------
    def set_id(self, current_id: int, new_id: int) -> None:
        """Permanently change servo ID (EEPROM write)."""
        if not 1 <= new_id <= 253:
            raise ValueError("ID must be in [1, 253]")
        self.write_byte(current_id, Register.ID, new_id)

    def set_baud_rate(self, id_: int, baud_index: int) -> None:
        """
        Set baud rate by index:
            0=1Mbps  1=500k  3=400k  4=250k  7=115200  9=57600  34=9600
        """
        self.write_byte(id_, Register.BAUD_RATE, baud_index)

    def set_angle_limits(self, id_: int, cw: int, ccw: int) -> None:
        """Set CW/CCW angle limits. Both 0 → wheel (continuous) mode."""
        cw = max(self.MIN_POSITION, min(self.MAX_POSITION, cw))
        ccw = max(self.MIN_POSITION, min(self.MAX_POSITION, ccw))
        self.write_word(id_, Register.CW_ANGLE_LIMIT_L, cw)
        self.write_word(id_, Register.CCW_ANGLE_LIMIT_L, ccw)

    def enable_wheel_mode(self, id_: int) -> None:
        """Switch to continuous rotation mode."""
        self.set_angle_limits(id_, 0, 0)

    def enable_joint_mode(self, id_: int) -> None:
        """Switch to position control mode with full range."""
        self.set_angle_limits(id_, 0, 1023)

    def set_led(self, id_: int, on: bool) -> None:
        """Turn the hardware LED indicator on or off."""
        self.write_byte(id_, Register.LED, int(on))

    def set_compliance(
        self, id_: int, margin: int = COMPLIANCE_MARGIN, slope: int = COMPLIANCE_SLOPE
    ) -> None:
        """Set compliance margin and slope (applied to both CW and CCW)."""
        self.write_byte(id_, Register.CW_COMPLIANCE_MARGIN, margin & 0xFF)
        self.write_byte(id_, Register.CCW_COMPLIANCE_MARGIN, margin & 0xFF)
        self.write_byte(id_, Register.CW_COMPLIANCE_SLOPE, slope & 0xFF)
        self.write_byte(id_, Register.CCW_COMPLIANCE_SLOPE, slope & 0xFF)

    def reset(self, id_: int) -> None:
        """
        Reset servo to factory defaults.

        Resets ID → 1 and baud → 1 Mbps.
        """
        self._bus.send(InstructionPacketBuilder.reset(id_))
