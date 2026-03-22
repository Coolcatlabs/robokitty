import time

from .. import _log
from .constants import DEFAULT_PORT, DEFAULT_BAUD


class AX12Interface:
    """
    Minimal AX-12A driver using pyserial.

    For Pi 4 with USB2Dynamixel or U2D2: direction control is automatic.
    For direct UART with tri-state buffer: set direction_pin.
    """

    HEADER = bytes([0xFF, 0xFF])

    INST_READ = 0x02
    INST_WRITE = 0x03
    INST_REG_WRITE = 0x04
    INST_ACTION = 0x05
    INST_SYNC_WRITE = 0x83

    ADDR_TORQUE_ENABLE = 24
    ADDR_LED = 25
    ADDR_CW_COMPLIANCE_MARGIN = 26
    ADDR_CCW_COMPLIANCE_MARGIN = 27
    ADDR_CW_COMPLIANCE_SLOPE = 28
    ADDR_CCW_COMPLIANCE_SLOPE = 29
    ADDR_GOAL_POSITION = 30
    ADDR_MOVING_SPEED = 32
    ADDR_PRESENT_POSITION = 36

    def __init__(
        self,
        port: str = DEFAULT_PORT,
        baudrate: int = DEFAULT_BAUD,
        direction_pin: int | None = None,
    ):
        self.port_path = port
        self.baudrate = baudrate
        self.direction_pin = direction_pin
        self.serial = None
        self._connected = False
        self._gpio_setup = False

    def connect(self) -> bool:
        try:
            import serial

            self.serial = serial.Serial(
                port=self.port_path,
                baudrate=self.baudrate,
                timeout=0.01,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )

            if self.direction_pin is not None:
                try:
                    import RPi.GPIO as GPIO

                    GPIO.setmode(GPIO.BCM)
                    GPIO.setup(self.direction_pin, GPIO.OUT)
                    GPIO.output(self.direction_pin, GPIO.LOW)
                    self._gpio_setup = True
                except ImportError:
                    _log.warning("RPi.GPIO not available, direction pin ignored")

            self._connected = True
            _log.info(f"AX-12A bus connected: {self.port_path} @ {self.baudrate}")
            return True

        except Exception as e:
            _log.error(f"ERROR connecting to {self.port_path}: {e}")
            return False

    def _set_tx_mode(self):
        if self._gpio_setup:
            import RPi.GPIO as GPIO

            GPIO.output(self.direction_pin, GPIO.HIGH)

    def _set_rx_mode(self):
        if self._gpio_setup:
            import RPi.GPIO as GPIO

            GPIO.output(self.direction_pin, GPIO.LOW)

    @staticmethod
    def _checksum(data: bytes) -> int:
        return (~sum(data)) & 0xFF

    def _send_packet(self, servo_id: int, instruction: int, params: bytes = b""):
        if not self._connected:
            return
        length = len(params) + 2
        packet_body = bytes([servo_id, length, instruction]) + params
        chk = self._checksum(packet_body)
        packet = self.HEADER + packet_body + bytes([chk])

        self._set_tx_mode()
        self.serial.write(packet)
        self.serial.flush()
        time.sleep(0.0001)
        self._set_rx_mode()

    def _read_response(
        self, expected_params: int = 2, timeout: float = 0.05
    ) -> tuple[int | None, bytes | None]:
        """
        Read a status packet from the bus after a READ instruction.

        AX-12A status packet: [0xFF][0xFF][ID][Length][Error][Param1..N][Checksum]
        Total bytes = 6 + expected_params

        Returns (error_byte, param_bytes) or (None, None) on comm failure.
        The error byte contains the actual hardware error flags:
          bit 0: Input Voltage  bit 1: Angle Limit  bit 2: Overheating
          bit 3: Range          bit 4: Checksum      bit 5: Overload
          bit 6: Instruction
        """
        total = 6 + expected_params
        self.serial.reset_input_buffer()
        time.sleep(0.001)
        self.serial.reset_input_buffer()

        deadline = time.monotonic() + timeout
        buf = b""
        while len(buf) < total and time.monotonic() < deadline:
            remaining = total - len(buf)
            chunk = self.serial.read(remaining)
            if chunk:
                buf += chunk
            else:
                time.sleep(0.001)

        if len(buf) < total:
            return (None, None)

        idx = buf.find(self.HEADER)
        if idx < 0:
            extra = self.serial.read(16)
            buf += extra
            idx = buf.find(self.HEADER)
            if idx < 0:
                return (None, None)

        pkt = buf[idx : idx + total]
        if len(pkt) < total:
            return (None, None)

        body = pkt[2:-1]
        if pkt[-1] != self._checksum(body):
            return (None, None)

        error = pkt[4]
        params = pkt[5 : 5 + expected_params]
        return (error, params)

    def read_position(self, servo_id: int) -> int | None:
        """Read present position (0-1023) or None on failure."""
        if not self._connected:
            return None
        self.serial.reset_input_buffer()
        self._send_packet(
            servo_id, self.INST_READ, bytes([self.ADDR_PRESENT_POSITION, 2])
        )
        err, params = self._read_response(expected_params=2, timeout=0.05)
        if params is None or len(params) < 2:
            return None
        position = params[0] | (params[1] << 8)
        if position > 1023:
            return None
        return position

    def read_error(self, servo_id: int) -> int | None:
        """
        Read the hardware error status from a servo.

        Sends a benign read (present position) and extracts the error
        byte from the status packet response. This is the ACTUAL current
        error state, not the alarm mask configuration.

        Returns error byte (0=healthy) or None if servo not responding.
        Error bits:
          bit 0: Input Voltage   bit 1: Angle Limit   bit 2: Overheating
          bit 3: Range           bit 4: Checksum       bit 5: Overload
          bit 6: Instruction
        """
        if not self._connected:
            return None
        self.serial.reset_input_buffer()
        # Read present position just to get the status packet error byte
        self._send_packet(
            servo_id, self.INST_READ, bytes([self.ADDR_PRESENT_POSITION, 2])
        )
        err, params = self._read_response(expected_params=2, timeout=0.05)
        if err is None:
            return None
        return err

    def read_voltage(self, servo_id: int) -> float | None:
        """Read present voltage (address 42, 1 byte). Returns volts."""
        if not self._connected:
            return None
        self.serial.reset_input_buffer()
        self._send_packet(servo_id, self.INST_READ, bytes([42, 1]))
        err, params = self._read_response(expected_params=1, timeout=0.05)
        if params is None or len(params) < 1:
            return None
        return params[0] / 10.0

    def read_temperature(self, servo_id: int) -> int | None:
        """Read present temperature (address 43, 1 byte). Returns deg C."""
        if not self._connected:
            return None
        self.serial.reset_input_buffer()
        self._send_packet(servo_id, self.INST_READ, bytes([43, 1]))
        err, params = self._read_response(expected_params=1, timeout=0.05)
        if params is None or len(params) < 1:
            return None
        return params[0]

    def clear_error(self, servo_id: int):
        """
        Clear latched error on AX-12A.

        AX-12A latches overload errors - torque gets disabled and stays off
        until the error condition is resolved. Steps to clear:
        1. Disable torque
        2. Set Torque Limit (addr 34) back to max (0x3FF)
        3. Temporarily clear Alarm Shutdown (addr 18) to prevent re-latch
        4. Re-enable torque
        5. Restore Alarm Shutdown to default
        """
        # Disable torque
        self.enable_torque(servo_id, False)
        time.sleep(0.05)
        # Reset torque limit to max (addr 34, 2 bytes)
        self._send_packet(servo_id, self.INST_WRITE, bytes([34, 0xFF, 0x03]))
        time.sleep(0.01)
        # Temporarily set Alarm Shutdown to 0 (addr 18) to prevent re-latch
        self._send_packet(servo_id, self.INST_WRITE, bytes([18, 0x00]))
        time.sleep(0.01)
        # Turn off LED
        self.set_led(servo_id, False)
        time.sleep(0.05)
        # Re-enable torque
        self.enable_torque(servo_id, True)
        time.sleep(0.1)
        # Restore Alarm Shutdown to default (0x24 = overload + overheat)
        self._send_packet(servo_id, self.INST_WRITE, bytes([18, 0x24]))
        time.sleep(0.01)

    def enable_torque(self, servo_id: int, enable: bool = True):
        self._send_packet(
            servo_id,
            self.INST_WRITE,
            bytes([self.ADDR_TORQUE_ENABLE, 1 if enable else 0]),
        )
        time.sleep(0.001)

    def set_led(self, servo_id: int, on: bool = True):
        """Flash servo LED - useful for identifying physical servo location."""
        self._send_packet(
            servo_id, self.INST_WRITE, bytes([self.ADDR_LED, 1 if on else 0])
        )
        time.sleep(0.001)

    def set_moving_speed(self, servo_id: int, speed: int = 200):
        speed = max(0, min(1023, speed))
        self._send_packet(
            servo_id,
            self.INST_WRITE,
            bytes([self.ADDR_MOVING_SPEED, speed & 0xFF, (speed >> 8) & 0xFF]),
        )
        time.sleep(0.001)

    def set_compliance(self, servo_id: int, margin: int = 0, slope: int = 32):
        """Set compliance margin and slope. margin=0 eliminates dead zone."""
        self._send_packet(
            servo_id, self.INST_WRITE, bytes([self.ADDR_CW_COMPLIANCE_MARGIN, margin])
        )
        self._send_packet(
            servo_id, self.INST_WRITE, bytes([self.ADDR_CCW_COMPLIANCE_MARGIN, margin])
        )
        self._send_packet(
            servo_id, self.INST_WRITE, bytes([self.ADDR_CW_COMPLIANCE_SLOPE, slope])
        )
        self._send_packet(
            servo_id, self.INST_WRITE, bytes([self.ADDR_CCW_COMPLIANCE_SLOPE, slope])
        )
        time.sleep(0.001)

    def sync_write_positions(self, positions: dict[int, int]):
        """
        Write positions to all servos in ONE packet.
        All servos start moving simultaneously - critical for smooth gait.
        """
        if not self._connected or not positions:
            return

        params = bytes([self.ADDR_GOAL_POSITION, 2])  # start addr, data length
        for sid, pos in positions.items():
            pos = max(0, min(1023, int(pos)))
            params += bytes([sid, pos & 0xFF, (pos >> 8) & 0xFF])

        self._send_packet(0xFE, self.INST_SYNC_WRITE, params)

    def identify_servo(self, servo_id: int, flashes: int = 3):
        """Flash a servo's LED to identify it physically."""
        for _ in range(flashes):
            self.set_led(servo_id, True)
            time.sleep(0.3)
            self.set_led(servo_id, False)
            time.sleep(0.3)

    def disconnect(self):
        if self._connected and self.serial:
            self.serial.close()
            self._connected = False
        if self._gpio_setup:
            try:
                import RPi.GPIO as GPIO

                GPIO.cleanup(self.direction_pin)
            except:  # noqa E722
                pass
        _log.info("AX-12A bus disconnected")

    @property
    def connected(self):
        return self._connected
