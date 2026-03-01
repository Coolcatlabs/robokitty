#!/usr/bin/env python3
"""
RoboKitty3 - Quadruped IK Walking Controller
=============================================
Version: 1.8 | 2026-02-26

Changes:
    v1.8 - MAJOR gait update: step_length 40->100mm for large visible
           leg swing (~77deg femur arc vs previous ~38deg). Step height
           25->35mm for ground clearance. Cycle time 2.0->1.6s trot,
           2.0->2.4s walk. Added audit logging: records all 12 servo
           angles every 10th cycle during walking. Compact CSV format
           with abbreviated headers (FL_s,FL_f,FL_l,...). Writes to
           timestamped file on exit.
    v1.7 - Applied calibrated offsets from --calibrate.
    v1.2 - Fixed read_error(): now reads error byte from status packet
           instead of register 18 (which is the alarm mask config).
           r/t/p keys now stop walk thread before bus reads to prevent
           50Hz sync_write from stomping on read packets.
           Added scan_all_errors() for full servo health report.
    v1.1 - Front/rear swapped. New IK: front legs alpha-beta, rear
           alpha+beta. Body height 128mm from read-pose calibration.
           Added read_position(), read_voltage(), read_temperature().
           Added --read-pose mode and 'p' key for live position reads.
    v1.0 - Initial RoboKitty3 release (renamed from quad7.py v0.23).

12-DOF (3 per leg) inverse kinematics with trot/walk/pace gaits.
AX-12A Dynamixel servos via half-duplex UART. Target: Raspberry Pi 4.

Leg layout (top view, new front facing up):
    FL (Front-Left)     FR (Front-Right)     IDs: 8/10/0   11/9/7
    RL (Rear-Left)      RR (Rear-Right)      IDs: 5/3/6    2/1/4

All legs: femur bends AWAY from front, tibia folds TOWARD front.
IK convention: X=forward, Y=lateral(+left), Z=up(+)/down(-).

Usage:
    python3 RoboKitty3.py --simulate       # No hardware
    python3 RoboKitty3.py --stand          # Stand only (calibration)
    python3 RoboKitty3.py --diag           # IK diagnostics
    python3 RoboKitty3.py --identify       # Flash servo LEDs
    python3 RoboKitty3.py --read-pose      # Read positions (torque off)
    python3 RoboKitty3.py --calibrate      # Pose by hand, compute offsets
"""

import math
import time
import threading
import argparse
import signal
import sys
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
from enum import Enum


# ============================================================================
# CONFIGURATION - ROBOKITTY ACTUAL MEASUREMENTS
# ============================================================================

@dataclass
class LegDimensions:
    """Leg segment lengths in mm (shaft-center to shaft-center, shaft to foot)."""
    coxa_length: float = 58.0     # Shoulder shaft to femur pivot
    femur_length: float = 68.0    # Femur pivot to knee pivot
    tibia_length: float = 75.0    # Knee pivot to foot contact point


@dataclass
class BodyDimensions:
    """Distance from body CENTER to coxa shaft center in mm."""
    half_length: float = 171.0    # Center to front/rear coxa shaft
    half_width: float = 101.5     # Center to left/right coxa shaft


@dataclass
class GaitConfig:
    """Gait timing and geometry - tuned for 1.65kg on carpet."""
    step_height: float = 35.0      # Foot lift height mm (good ground clearance)
    step_length: float = 100.0     # Forward travel per step mm (large visible swing)
    cycle_time: float = 1.6        # Full gait cycle seconds
    duty_factor: float = 0.5       # 0.5=trot, 0.75=walk
    body_height: float = 128.0     # Standing height mm (from calibration)
    update_rate_hz: float = 50.0   # Control loop frequency


# ============================================================================
# SERVO CONFIGURATION FOR AX-12A
# ============================================================================

class LegID(Enum):
    FL = "FrontLeft"
    FR = "FrontRight"
    RL = "RearLeft"
    RR = "RearRight"


@dataclass
class ServoJointConfig:
    """Configuration for one servo/joint."""
    servo_id: int
    offset_deg: float = 0.0     # Mechanical offset (tune during calibration)
    inverted: bool = False       # True = positive angle moves servo negative
    min_deg: float = -150.0      # Software limit (AX-12A range is +-150)
    max_deg: float = 150.0       # Software limit


# AX-12A constants
AX12_CENTER = 512
AX12_DEG_TO_UNITS = 1023.0 / 300.0  # ~3.41 units per degree

# ============================================================================
# ROBOKITTY SERVO MAP
# ============================================================================
# Robot front reversed from original build. Physical legs keep their servo IDs.
# Only the IK solution (front vs rear) changes based on new travel direction.
# Inversions unchanged from original calibration.

LEG_SERVO_CONFIG: Dict[LegID, List[ServoJointConfig]] = {
    # [Shoulder/Coxa, Femur, Tibia/Leg]
    # Offsets from --calibrate 2026-02-26
    LegID.FL: [
        ServoJointConfig(servo_id=8,  offset_deg=+0.9,  inverted=True),   # Shoulder
        ServoJointConfig(servo_id=10, offset_deg=-33.7, inverted=False),  # Femur
        ServoJointConfig(servo_id=0,  offset_deg=+48.1, inverted=False),  # Leg
    ],
    LegID.FR: [
        ServoJointConfig(servo_id=11, offset_deg=+6.1,  inverted=False),  # Shoulder
        ServoJointConfig(servo_id=9,  offset_deg=-47.2, inverted=True),   # Femur
        ServoJointConfig(servo_id=7,  offset_deg=+42.3, inverted=True),   # Leg
    ],
    LegID.RL: [
        ServoJointConfig(servo_id=5,  offset_deg=+2.6,  inverted=False),  # Shoulder
        ServoJointConfig(servo_id=3,  offset_deg=+16.4, inverted=True),   # Femur
        ServoJointConfig(servo_id=6,  offset_deg=-12.0, inverted=True),   # Leg
    ],
    LegID.RR: [
        ServoJointConfig(servo_id=2,  offset_deg=-38.4, inverted=True),   # Shoulder
        ServoJointConfig(servo_id=1,  offset_deg=+15.6, inverted=False),  # Femur
        ServoJointConfig(servo_id=4,  offset_deg=-26.4, inverted=False),  # Leg
    ],
}

# Serial config
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 1000000


# ============================================================================
# INVERSE KINEMATICS - 3DOF LEG
# ============================================================================

class LegIK:
    """
    3-DOF leg IK for RoboKitty3.

    All legs: femur bends AWAY from front, tibia folds TOWARD front.
    Front legs: alpha - beta (femur negative = backward, tibia positive)
    Rear legs:  alpha + beta (femur positive = away from front, tibia negative)

    Coxa swings laterally, femur/tibia swing fore-aft.
    """

    def __init__(self, dims: LegDimensions):
        self.L1 = dims.coxa_length
        self.L2 = dims.femur_length
        self.L3 = dims.tibia_length

    def solve(self, x: float, y: float, z: float,
              is_front: bool = True) -> Tuple[float, float, float]:
        """
        Compute (coxa_deg, femur_deg, tibia_deg) for foot at (x, y, z).

        is_front: True=front leg (alpha-beta), False=rear leg (alpha+beta)
        """
        # === COXA: frontal plane ===
        foot_lateral = abs(y)
        foot_down = -z

        foot_lateral = min(foot_lateral, self.L1 - 0.1)
        coxa_rad = math.acos(foot_lateral / self.L1)

        coxa_tip_drop = self.L1 * math.sin(coxa_rad)
        D_vertical = foot_down - coxa_tip_drop

        # === FEMUR + TIBIA: sagittal plane ===
        d_forward = x
        d_down = D_vertical

        d = math.sqrt(d_forward * d_forward + d_down * d_down)

        reach_max = self.L2 + self.L3 - 0.1
        reach_min = abs(self.L2 - self.L3) + 0.1
        d = max(reach_min, min(reach_max, d))

        cos_knee = (self.L2**2 + self.L3**2 - d**2) / (2.0 * self.L2 * self.L3)
        cos_knee = max(-1.0, min(1.0, cos_knee))
        knee_rad = math.acos(cos_knee)

        alpha = math.atan2(d_forward, d_down)
        cos_beta = (self.L2**2 + d**2 - self.L3**2) / (2.0 * self.L2 * d)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)

        if is_front:
            # Front: femur positive (tilts away from front), tibia negative (folds toward front)
            femur_deg = math.degrees(alpha + beta)
            tibia_deg = -math.degrees(math.pi - knee_rad)
        else:
            # Rear: femur negative (tilts away from front), tibia positive (folds toward front)
            femur_deg = math.degrees(alpha - beta)
            tibia_deg = math.degrees(math.pi - knee_rad)

        coxa_deg = math.degrees(coxa_rad)
        return (coxa_deg, femur_deg, tibia_deg)


# ============================================================================
# GAIT GENERATOR
# ============================================================================

class GaitType(Enum):
    TROT = "trot"
    WALK = "walk"
    PACE = "pace"


GAIT_PHASES = {
    GaitType.TROT: {
        LegID.FL: 0.0, LegID.FR: 0.5,
        LegID.RL: 0.5, LegID.RR: 0.0,
    },
    GaitType.WALK: {
        LegID.FL: 0.0,  LegID.FR: 0.5,
        LegID.RL: 0.75, LegID.RR: 0.25,
    },
    GaitType.PACE: {
        LegID.FL: 0.0, LegID.FR: 0.5,
        LegID.RL: 0.0, LegID.RR: 0.5,
    },
}


class FootTrajectory:
    """
    Generates smooth foot paths through the gait cycle.

    Stance phase: Linear slide backward (pushes body forward)
    Swing phase: Raised cosine arc forward + up
    """

    def __init__(self, cfg: GaitConfig):
        self.cfg = cfg

    def compute(self, phase: float, speed: float = 1.0) -> Tuple[float, float, float]:
        """
        Get (dx, dy, dz) foot offset from neutral position.

        phase: 0.0 to 1.0 within the gait cycle
        speed: -1.0 (reverse) to 1.0 (forward), 0 = standing
        """
        if abs(speed) < 0.01:
            return (0.0, 0.0, 0.0)

        duty = self.cfg.duty_factor
        half_step = self.cfg.step_length * 0.5

        if phase < duty:
            # STANCE: foot slides backward on ground
            t = phase / duty
            dx = half_step * (1.0 - 2.0 * t) * speed
            dz = 0.0
        else:
            # SWING: foot lifts and moves forward
            t = (phase - duty) / (1.0 - duty)
            dx = half_step * (-1.0 + 2.0 * t) * speed
            # Half-sine profile: lifts sharply at start, stays high, drops sharply
            # Much better ground clearance than raised cosine
            dz = self.cfg.step_height * math.sin(math.pi * t)

        return (dx, 0.0, dz)

    def compute_turn(self, phase: float, turn_rate: float,
                     hip_x: float, hip_y: float) -> Tuple[float, float, float]:
        """
        Foot offset for turning in place.

        turn_rate: -1.0 (left) to 1.0 (right)
        hip_x, hip_y: this leg's hip position relative to body center
        """
        if abs(turn_rate) < 0.01:
            return (0.0, 0.0, 0.0)

        duty = self.cfg.duty_factor
        max_yaw = math.radians(15.0) * turn_rate

        if phase < duty:
            t = phase / duty
            yaw = max_yaw * (0.5 - t)
            dz = 0.0
        else:
            t = (phase - duty) / (1.0 - duty)
            yaw = max_yaw * (-0.5 + t)
            dz = self.cfg.step_height * math.sin(math.pi * t)

        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        dx = hip_x * (cos_y - 1.0) - hip_y * sin_y
        dy = hip_x * sin_y + hip_y * (cos_y - 1.0)

        return (dx, dy, dz)


# ============================================================================
# AX-12A SERVO INTERFACE (half-duplex UART, Protocol 1.0)
# ============================================================================

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
    ADDR_GOAL_POSITION = 30
    ADDR_MOVING_SPEED = 32
    ADDR_PRESENT_POSITION = 36

    def __init__(self, port: str = DEFAULT_PORT, baudrate: int = DEFAULT_BAUD,
                 direction_pin: Optional[int] = None):
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
                stopbits=serial.STOPBITS_ONE
            )

            if self.direction_pin is not None:
                try:
                    import RPi.GPIO as GPIO
                    GPIO.setmode(GPIO.BCM)
                    GPIO.setup(self.direction_pin, GPIO.OUT)
                    GPIO.output(self.direction_pin, GPIO.LOW)
                    self._gpio_setup = True
                except ImportError:
                    print("WARN: RPi.GPIO not available, direction pin ignored")

            self._connected = True
            print(f"AX-12A bus connected: {self.port_path} @ {self.baudrate}")
            return True

        except Exception as e:
            print(f"ERROR connecting to {self.port_path}: {e}")
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

    def _send_packet(self, servo_id: int, instruction: int, params: bytes = b''):
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

    def _read_response(self, expected_params: int = 2,
                       timeout: float = 0.05) -> Tuple[Optional[int], Optional[bytes]]:
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
        buf = b''
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

        pkt = buf[idx:idx + total]
        if len(pkt) < total:
            return (None, None)

        body = pkt[2:-1]
        if pkt[-1] != self._checksum(body):
            return (None, None)

        error = pkt[4]
        params = pkt[5:5 + expected_params]
        return (error, params)

    def read_position(self, servo_id: int) -> Optional[int]:
        """Read present position (0-1023) or None on failure."""
        if not self._connected:
            return None
        self.serial.reset_input_buffer()
        self._send_packet(servo_id, self.INST_READ,
                          bytes([self.ADDR_PRESENT_POSITION, 2]))
        err, params = self._read_response(expected_params=2, timeout=0.05)
        if params is None or len(params) < 2:
            return None
        position = params[0] | (params[1] << 8)
        if position > 1023:
            return None
        return position

    def read_error(self, servo_id: int) -> Optional[int]:
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
        self._send_packet(servo_id, self.INST_READ,
                          bytes([self.ADDR_PRESENT_POSITION, 2]))
        err, params = self._read_response(expected_params=2, timeout=0.05)
        if err is None:
            return None
        return err

    def read_voltage(self, servo_id: int) -> Optional[float]:
        """Read present voltage (address 42, 1 byte). Returns volts."""
        if not self._connected:
            return None
        self.serial.reset_input_buffer()
        self._send_packet(servo_id, self.INST_READ, bytes([42, 1]))
        err, params = self._read_response(expected_params=1, timeout=0.05)
        if params is None or len(params) < 1:
            return None
        return params[0] / 10.0

    def read_temperature(self, servo_id: int) -> Optional[int]:
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
        self._send_packet(servo_id, self.INST_WRITE,
                          bytes([34, 0xFF, 0x03]))
        time.sleep(0.01)
        # Temporarily set Alarm Shutdown to 0 (addr 18) to prevent re-latch
        self._send_packet(servo_id, self.INST_WRITE,
                          bytes([18, 0x00]))
        time.sleep(0.01)
        # Turn off LED
        self.set_led(servo_id, False)
        time.sleep(0.05)
        # Re-enable torque
        self.enable_torque(servo_id, True)
        time.sleep(0.1)
        # Restore Alarm Shutdown to default (0x24 = overload + overheat)
        self._send_packet(servo_id, self.INST_WRITE,
                          bytes([18, 0x24]))
        time.sleep(0.01)

    def enable_torque(self, servo_id: int, enable: bool = True):
        self._send_packet(servo_id, self.INST_WRITE,
                          bytes([self.ADDR_TORQUE_ENABLE, 1 if enable else 0]))
        time.sleep(0.001)

    def set_led(self, servo_id: int, on: bool = True):
        """Flash servo LED - useful for identifying physical servo location."""
        self._send_packet(servo_id, self.INST_WRITE,
                          bytes([self.ADDR_LED, 1 if on else 0]))
        time.sleep(0.001)

    def set_moving_speed(self, servo_id: int, speed: int = 200):
        speed = max(0, min(1023, speed))
        self._send_packet(servo_id, self.INST_WRITE,
                          bytes([self.ADDR_MOVING_SPEED, speed & 0xFF, (speed >> 8) & 0xFF]))
        time.sleep(0.001)

    def sync_write_positions(self, positions: Dict[int, int]):
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
            except:
                pass
        print("AX-12A bus disconnected")

    @property
    def connected(self):
        return self._connected


# ============================================================================
# SIMULATED SERVO INTERFACE
# ============================================================================

class SimulatedServoInterface:
    """Drop-in replacement for testing without hardware."""

    def __init__(self):
        self._connected = True
        self._positions: Dict[int, int] = {}

    def connect(self) -> bool:
        print("[SIM] Simulated AX-12A interface ready")
        return True

    def enable_torque(self, servo_id: int, enable: bool = True):
        pass

    def set_led(self, servo_id: int, on: bool = True):
        pass

    def set_moving_speed(self, servo_id: int, speed: int = 200):
        pass

    def sync_write_positions(self, positions: Dict[int, int]):
        self._positions.update(positions)

    def read_position(self, servo_id: int) -> Optional[int]:
        return self._positions.get(servo_id, 512)

    def read_error(self, servo_id: int) -> Optional[int]:
        return 0

    def read_voltage(self, servo_id: int) -> Optional[float]:
        return 11.1

    def read_temperature(self, servo_id: int) -> Optional[int]:
        return 35

    def clear_error(self, servo_id: int):
        pass

    def _send_packet(self, servo_id: int, instruction: int, params: bytes = b''):
        pass

    def identify_servo(self, servo_id: int, flashes: int = 3):
        print(f"[SIM] Flashing servo {servo_id}")

    def disconnect(self):
        self._connected = False
        print("[SIM] Disconnected")

    @property
    def connected(self):
        return self._connected

    def dump_angles(self) -> Dict[int, float]:
        return {sid: round((pos - 512) / 3.41, 1)
                for sid, pos in sorted(self._positions.items())}


# ============================================================================
# ANGLE -> RAW POSITION CONVERSION
# ============================================================================

def angle_to_raw(angle_deg: float, config: ServoJointConfig) -> int:
    """Convert joint angle in degrees to AX-12A raw position (0-1023)."""
    clamped = max(config.min_deg, min(config.max_deg, angle_deg))
    effective = clamped + config.offset_deg
    if config.inverted:
        effective = -effective
    raw = int(AX12_CENTER + effective * AX12_DEG_TO_UNITS)
    return max(0, min(1023, raw))


# ============================================================================
# MAIN CONTROLLER
# ============================================================================

class QuadrupedWalker:
    """Top-level walking controller for RoboKitty."""

    def __init__(self, leg_dims: LegDimensions = None,
                 body_dims: BodyDimensions = None,
                 gait_config: GaitConfig = None,
                 simulate: bool = False,
                 port: str = DEFAULT_PORT,
                 baudrate: int = DEFAULT_BAUD,
                 direction_pin: Optional[int] = None):

        self.leg_dims = leg_dims or LegDimensions()
        self.body_dims = body_dims or BodyDimensions()
        self.gait_cfg = gait_config or GaitConfig()

        self.ik = LegIK(self.leg_dims)
        self.trajectory = FootTrajectory(self.gait_cfg)

        self.gait_type = GaitType.TROT
        self.gait_phases = dict(GAIT_PHASES[self.gait_type])

        if simulate:
            self.servos = SimulatedServoInterface()
        else:
            self.servos = AX12Interface(port, baudrate, direction_pin)

        # Hip positions relative to body center
        bL = self.body_dims.half_length
        bW = self.body_dims.half_width
        self.hip_positions = {
            LegID.FL: ( bL,  bW),
            LegID.FR: ( bL, -bW),
            LegID.RL: (-bL,  bW),
            LegID.RR: (-bL, -bW),
        }

        # Per-leg front/rear designation
        self.leg_is_front = {
            LegID.FL: True,
            LegID.FR: True,
            LegID.RL: False,
            LegID.RR: False,
        }

        # Neutral foot positions relative to each hip
        L1 = self.leg_dims.coxa_length
        foot_lateral = 5.0   # Nearly vertical legs
        h = self.gait_cfg.body_height

        self.neutral_feet = {
            LegID.FL: (0.0,  foot_lateral, -h),
            LegID.FR: (0.0, -foot_lateral, -h),
            LegID.RL: (0.0,  foot_lateral, -h),
            LegID.RR: (0.0, -foot_lateral, -h),
        }

        # Lock standing coxa angle per leg
        self.locked_coxa_angles = {}
        for leg_id in LegID:
            foot = self.neutral_feet[leg_id]
            angles = self.ik.solve(*foot,
                                   is_front=self.leg_is_front[leg_id])
            self.locked_coxa_angles[leg_id] = angles[0]
        self._running = False
        self._walk_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._speed = 0.0
        self._turn = 0.0
        self._phase = 0.0
        self.on_update = None  # Optional telemetry callback

        # Audit log: stores servo angles every N cycles during walking
        self._audit_log: List[str] = []
        self._audit_cycle = 0
        self._audit_interval = 10  # Log every 10th cycle
        # Build compact column header: FL_s,FL_f,FL_l,FR_s,...
        self._audit_header = "cyc,ph,spd," + ",".join(
            f"{lid.name}_{jn}" for lid in LegID
            for jn in ["s", "f", "l"]
        )

    def connect(self) -> bool:
        if not self.servos.connect():
            return False
        all_ids = self._all_servo_ids()
        shoulder_ids = [LEG_SERVO_CONFIG[lid][0].servo_id for lid in LegID]
        for sid in all_ids:
            self.servos.enable_torque(sid, True)
            if sid in shoulder_ids:
                # Shoulders: moderate speed for better position holding
                # Speed 0 on AX-12A = max speed but weaker hold
                # Moderate speed gives better compliance/holding
                self.servos.set_moving_speed(sid, 300)
            else:
                # Femur/tibia: max speed to keep up with 50Hz updates
                self.servos.set_moving_speed(sid, 0)
            time.sleep(0.003)
        print(f"All {len(all_ids)} servos enabled (shoulders=300, legs=max)")
        return True

    def disconnect(self):
        self.stop()
        self.save_audit_log()
        all_ids = self._all_servo_ids()
        for sid in all_ids:
            self.servos.enable_torque(sid, False)
            time.sleep(0.003)
        self.servos.disconnect()

    def save_audit_log(self):
        """Write audit log to CSV if any data was recorded."""
        if not self._audit_log:
            return
        fname = f"robokitty_audit_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        try:
            with open(fname, "w") as f:
                f.write(self._audit_header + "\n")
                f.write("\n".join(self._audit_log) + "\n")
            print(f"Audit log saved: {fname} ({len(self._audit_log)} rows)")
        except Exception as e:
            print(f"Failed to save audit log: {e}")

    def _all_servo_ids(self) -> List[int]:
        ids = []
        for leg_id in LegID:
            for cfg in LEG_SERVO_CONFIG[leg_id]:
                ids.append(cfg.servo_id)
        return ids

    def diagnose_servo(self, servo_id: int):
        """Full diagnostic for a servo - reads error register, voltage, temp."""
        print(f"\n  === Diagnosing Servo ID {servo_id} ===")

        # Step 1: Read error register
        print(f"  1. Reading error register...")
        err = self.servos.read_error(servo_id)
        if err is None:
            print(f"     FAILED to read - servo not responding on bus!")
            print(f"     Check: wiring, power, servo ID, baud rate")
        else:
            error_names = [
                (0, "Input Voltage"),
                (1, "Angle Limit"),
                (2, "Overheating"),
                (3, "Range"),
                (4, "Checksum"),
                (5, "Overload"),
                (6, "Instruction"),
            ]
            if err == 0:
                print(f"     Error register: 0x00 (no errors)")
            else:
                print(f"     Error register: 0x{err:02X}")
                for bit, name in error_names:
                    if err & (1 << bit):
                        print(f"     ** {name} Error (bit {bit}) **")

        # Step 2: Read voltage
        print(f"  2. Reading voltage...")
        volts = self.servos.read_voltage(servo_id)
        if volts is not None:
            status = "OK" if 9.0 <= volts <= 12.6 else "WARNING"
            print(f"     Voltage: {volts:.1f}V ({status})")
        else:
            print(f"     Failed to read voltage")

        # Step 3: Read temperature
        print(f"  3. Reading temperature...")
        temp = self.servos.read_temperature(servo_id)
        if temp is not None:
            status = "OK" if temp < 65 else "HOT!" if temp < 75 else "CRITICAL!"
            print(f"     Temperature: {temp}C ({status})")
        else:
            print(f"     Failed to read temperature")

        # Step 4: Read current position
        print(f"  4. Reading position...")
        pos = self.servos.read_position(servo_id)
        if pos is not None:
            print(f"     Position: {pos} (center=512)")
        else:
            print(f"     Failed to read position")

        # Step 5: Attempt recovery
        print(f"  5. Attempting error clear (torque cycle + LED off)...")
        self.servos.clear_error(servo_id)
        time.sleep(0.5)

        # Re-read error
        err2 = self.servos.read_error(servo_id)
        if err2 is not None and err2 == 0:
            print(f"     Error cleared successfully!")
        elif err2 is not None:
            print(f"     Error persists: 0x{err2:02X}")
            print(f"     Try: power cycle, check for mechanical bind, reduce load")
        else:
            print(f"     Still can't communicate with servo")

        # Step 6: Try position command
        print(f"  6. Setting joint mode and testing movement...")
        self.servos._send_packet(servo_id, 0x03, bytes([6, 0x00, 0x00]))
        time.sleep(0.01)
        self.servos._send_packet(servo_id, 0x03, bytes([8, 0xFF, 0x03]))
        time.sleep(0.01)
        self.servos.enable_torque(servo_id, True)
        self.servos.set_moving_speed(servo_id, 200)
        time.sleep(0.1)

        print(f"     Moving to center (512)...")
        self.servos.sync_write_positions({servo_id: 512})
        time.sleep(1.5)
        print(f"     Moving to 350...")
        self.servos.sync_write_positions({servo_id: 350})
        time.sleep(1.5)
        print(f"     Moving to 650...")
        self.servos.sync_write_positions({servo_id: 650})
        time.sleep(1.5)

        # Return to standing position (not center!)
        print(f"  7. Returning to standing pose...")
        self.stand()
        time.sleep(1.0)

        print(f"\n  === Diagnosis complete for servo ID {servo_id} ===")

    def scan_all_errors(self):
        """Read error register, voltage, and temperature from every servo."""
        error_names = [
            (0, "Voltage"), (1, "AngleLimit"), (2, "Overheat"),
            (3, "Range"), (4, "Checksum"), (5, "Overload"), (6, "Instruction"),
        ]
        joint_names = ["Shoulder", "Femur", "Leg"]
        any_error = False

        for leg_id in LegID:
            configs = LEG_SERVO_CONFIG[leg_id]
            for i, cfg in enumerate(configs):
                sid = cfg.servo_id
                err = self.servos.read_error(sid)
                volts = self.servos.read_voltage(sid)
                temp = self.servos.read_temperature(sid)
                pos = self.servos.read_position(sid)

                # Build status string
                parts = []
                if err is None:
                    parts.append("NO RESPONSE")
                    any_error = True
                elif err != 0:
                    flags = [name for bit, name in error_names if err & (1 << bit)]
                    parts.append(f"ERR:{'|'.join(flags)}")
                    any_error = True

                if volts is not None and (volts < 9.0 or volts > 12.6):
                    parts.append(f"V={volts:.1f}!")
                if temp is not None and temp >= 65:
                    parts.append(f"T={temp}C!")

                v_str = f"{volts:.1f}V" if volts else "???V"
                t_str = f"{temp}C" if temp else "???C"
                p_str = f"pos={pos}" if pos is not None else "pos=???"
                e_str = f"0x{err:02X}" if err is not None else "???"

                status = " ** " + ", ".join(parts) + " **" if parts else " OK"
                print(f"    {leg_id.value:12s} {joint_names[i]:8s} ID{sid:2d}: "
                      f"err={e_str} {v_str} {t_str} {p_str}{status}")

        if not any_error:
            print("    All servos healthy!")

    def test_rr_shoulder(self):
        """Sweep RR shoulder servo (ID 2) in and out to test it."""
        sid = 2
        cfg = LEG_SERVO_CONFIG[LegID.RR][0]

        # Enable torque and set moderate speed
        self.servos.enable_torque(sid, True)
        self.servos.set_moving_speed(sid, 200)
        time.sleep(0.1)

        # Get standing position
        stand_raw = angle_to_raw(self.locked_coxa_angles[LegID.RR], cfg)

        # Sweep: center -> out -> center -> in -> center
        positions = [
            ("Standing", stand_raw),
            ("Out (+40)", stand_raw + 40),
            ("Standing", stand_raw),
            ("In (-40)", stand_raw - 40),
            ("Standing", stand_raw),
            ("Out (+80)", stand_raw + 80),
            ("Standing", stand_raw),
            ("In (-80)", stand_raw - 80),
            ("Standing", stand_raw),
        ]

        for label, pos in positions:
            pos = max(0, min(1023, pos))
            print(f"    RR Shoulder -> {label} (raw={pos})")
            self.servos.sync_write_positions({sid: pos})
            time.sleep(1.0)

        # Restore speed
        self.servos.set_moving_speed(sid, 300)

    def identify_all_servos(self):
        """Flash each servo LED one at a time with label."""
        for leg_id in LegID:
            configs = LEG_SERVO_CONFIG[leg_id]
            joint_names = ["Shoulder", "Femur", "Leg"]
            for cfg, name in zip(configs, joint_names):
                print(f"  Flashing: {leg_id.value} {name} (ID {cfg.servo_id})")
                self.servos.identify_servo(cfg.servo_id, flashes=10)
                time.sleep(0.5)

    def read_all_positions(self):
        """
        Read current position of all 12 servos and display as angles.

        Use this with torque OFF to manually pose the robot then capture
        the exact servo positions as a calibration reference.
        """
        print("\n" + "=" * 62)
        print("  READING CURRENT SERVO POSITIONS")
        print("  (Torque should be OFF - manually pose legs first)")
        print("=" * 62)

        joint_names = ["Shoulder", "Femur   ", "Leg     "]
        all_raw = {}
        all_angles = {}

        for leg_id in LegID:
            configs = LEG_SERVO_CONFIG[leg_id]
            is_front = self.leg_is_front[leg_id]
            print(f"\n  {leg_id.value} ({'FRONT' if is_front else 'REAR'}):")

            for i, (cfg, name) in enumerate(zip(configs, joint_names)):
                raw = self.servos.read_position(cfg.servo_id)
                if raw is None:
                    print(f"    {name} ID{cfg.servo_id:2d}:  ** READ FAILED **")
                    continue

                # Reverse the angle_to_raw conversion to get joint angle
                # raw = AX12_CENTER + effective * AX12_DEG_TO_UNITS
                # effective = (raw - AX12_CENTER) / AX12_DEG_TO_UNITS
                # if inverted: effective = -angle, so angle = -effective
                # then subtract offset
                effective = (raw - AX12_CENTER) / AX12_DEG_TO_UNITS
                if cfg.inverted:
                    angle = -effective
                else:
                    angle = effective
                angle -= cfg.offset_deg

                all_raw[cfg.servo_id] = raw
                all_angles[cfg.servo_id] = angle

                inv_str = " INV" if cfg.inverted else ""
                print(f"    {name} ID{cfg.servo_id:2d}:  raw={raw:4d}  "
                      f"angle={angle:+7.1f}deg{inv_str}")

        # Summary table for easy copy-paste
        print("\n" + "-" * 62)
        print("  RAW POSITION SUMMARY (for copy-paste into config):")
        print("-" * 62)
        for leg_id in LegID:
            configs = LEG_SERVO_CONFIG[leg_id]
            vals = []
            for cfg in configs:
                r = all_raw.get(cfg.servo_id)
                vals.append(f"{r:4d}" if r is not None else " ???")
            print(f"  {leg_id.value:12s}:  Shoulder={vals[0]}  "
                  f"Femur={vals[1]}  Leg={vals[2]}")

        print("\n  ANGLE SUMMARY:")
        print("-" * 62)
        for leg_id in LegID:
            configs = LEG_SERVO_CONFIG[leg_id]
            vals = []
            for cfg in configs:
                a = all_angles.get(cfg.servo_id)
                vals.append(f"{a:+7.1f}" if a is not None else "   ???")
            print(f"  {leg_id.value:12s}:  Shoulder={vals[0]}  "
                  f"Femur={vals[1]}  Leg={vals[2]}")
        print()

    def calibrate_standing(self):
        """
        Read all servo positions from manual pose, compare with IK targets,
        and compute the offset_deg needed for each joint to match.

        After running, copy the suggested offsets into LEG_SERVO_CONFIG.
        """
        print("\n" + "=" * 62)
        print("  READING POSED POSITIONS & COMPUTING OFFSETS")
        print("=" * 62)

        joint_names = ["Shoulder", "Femur   ", "Leg     "]
        suggested_offsets = {}

        for leg_id in LegID:
            configs = LEG_SERVO_CONFIG[leg_id]
            is_front = self.leg_is_front[leg_id]
            foot = self.neutral_feet[leg_id]
            ik_angles = self.ik.solve(*foot, is_front=is_front)

            print(f"\n  {leg_id.value} ({'FRONT' if is_front else 'REAR'}):")

            for i, (cfg, name) in enumerate(zip(configs, joint_names)):
                raw = self.servos.read_position(cfg.servo_id)
                if raw is None:
                    print(f"    {name} ID{cfg.servo_id:2d}:  ** READ FAILED **")
                    continue

                # What angle does the servo's current raw position represent?
                effective = (raw - AX12_CENTER) / AX12_DEG_TO_UNITS
                if cfg.inverted:
                    actual_angle = -effective - cfg.offset_deg
                else:
                    actual_angle = effective - cfg.offset_deg

                # What angle does IK want?
                ik_angle = ik_angles[i]

                # What offset would make IK target match the actual position?
                # angle_to_raw: raw = 512 + (angle + offset) * DEG  (or negated if inv)
                # We want raw_actual = angle_to_raw(ik_angle, new_offset)
                # For not inverted: raw = 512 + (ik_angle + new_offset) * DEG
                #   new_offset = (raw - 512) / DEG - ik_angle
                # For inverted: raw = 512 - (ik_angle + new_offset) * DEG
                #   new_offset = -((raw - 512) / DEG) - ik_angle
                if cfg.inverted:
                    needed_offset = -((raw - AX12_CENTER) / AX12_DEG_TO_UNITS) - ik_angle
                else:
                    needed_offset = ((raw - AX12_CENTER) / AX12_DEG_TO_UNITS) - ik_angle

                current_offset = cfg.offset_deg
                inv_str = " INV" if cfg.inverted else ""
                change = needed_offset - current_offset

                print(f"    {name} ID{cfg.servo_id:2d}:  raw={raw:4d}  "
                      f"actual={actual_angle:+7.1f}  ik_target={ik_angle:+7.1f}  "
                      f"offset: {current_offset:+.1f} -> {needed_offset:+.1f} "
                      f"(change {change:+.1f}){inv_str}")

                suggested_offsets[(leg_id, i)] = needed_offset

        # Print copy-paste config
        print("\n" + "=" * 62)
        print("  SUGGESTED LEG_SERVO_CONFIG (copy-paste into code):")
        print("=" * 62)
        for leg_id in LegID:
            configs = LEG_SERVO_CONFIG[leg_id]
            jnames = ["Shoulder", "Femur", "Leg"]
            print(f"    LegID.{leg_id.name}: [")
            for i, (cfg, jn) in enumerate(zip(configs, jnames)):
                off = suggested_offsets.get((leg_id, i), cfg.offset_deg)
                off_rounded = round(off, 1)
                inv_str = "True " if cfg.inverted else "False"
                print(f"        ServoJointConfig(servo_id={cfg.servo_id:<2d}, "
                      f"offset_deg={off_rounded:+6.1f}, inverted={inv_str}),"
                      f"  # {jn}")
            print(f"    ],")

        print()
        print("  Copy the offsets above into LEG_SERVO_CONFIG in the code.")
        print("  Then run --stand to verify the pose matches.")
        print()

    def test_joints(self):
        """
        Test each joint one at a time.
        Moves each servo slightly from center so you can see direction.
        Press Enter after each to continue.
        """
        joint_names = ["Shoulder", "Femur", "Leg"]
        
        # First center ALL servos
        print("\n  Centering all servos to 512 (neutral)...")
        center_positions = {}
        for leg_id in LegID:
            for cfg in LEG_SERVO_CONFIG[leg_id]:
                center_positions[cfg.servo_id] = 512
        self.servos.sync_write_positions(center_positions)
        time.sleep(1.0)
        
        for leg_id in LegID:
            configs = LEG_SERVO_CONFIG[leg_id]
            for i, cfg in enumerate(configs):
                # Reset all to center
                self.servos.sync_write_positions(center_positions)
                time.sleep(0.5)
                
                print(f"\n  {leg_id.value} {joint_names[i]} (ID {cfg.servo_id})")
                print(f"    Moving POSITIVE 30 degrees from center...")
                
                # Move this one servo +30 degrees from center
                test_pos = 512 + int(30 * AX12_DEG_TO_UNITS)  # ~614
                self.servos.sync_write_positions({cfg.servo_id: test_pos})
                time.sleep(1.0)
                
                print(f"    Raw position: 512 -> {test_pos}")
                print(f"    Currently inverted: {cfg.inverted}")
                print(f"    What did the servo do?")
                print(f"      Shoulder: should swing FORWARD")
                print(f"      Femur:    should swing leg DOWN/FORWARD")
                print(f"      Leg:      should swing foot DOWN/FORWARD")
                
                input(f"    Press Enter for next joint...")
                
                # Return to center
                self.servos.sync_write_positions({cfg.servo_id: 512})
                time.sleep(0.5)
        
        print("\n  Joint test complete. Report which ones went the wrong way.")
        """Flash each servo LED one at a time with label."""
        for leg_id in LegID:
            configs = LEG_SERVO_CONFIG[leg_id]
            joint_names = ["Shoulder", "Femur", "Leg"]
            for cfg, name in zip(configs, joint_names):
                print(f"  Flashing: {leg_id.value} {name} (ID {cfg.servo_id})")
                self.servos.identify_servo(cfg.servo_id, flashes=10)
                time.sleep(0.5)

    def set_gait(self, gait: GaitType):
        with self._lock:
            self.gait_type = gait
            self.gait_phases = dict(GAIT_PHASES[gait])
            if gait == GaitType.WALK:
                self.gait_cfg.duty_factor = 0.75
                self.gait_cfg.cycle_time = 2.4
            else:
                self.gait_cfg.duty_factor = 0.5
                self.gait_cfg.cycle_time = 1.6
        print(f"Gait: {gait.value}")

    def set_speed(self, speed: float):
        with self._lock:
            self._speed = max(-1.0, min(1.0, speed))

    def set_turn(self, turn: float):
        with self._lock:
            self._turn = max(-1.0, min(1.0, turn))

    def stand(self):
        """Move to neutral standing position."""
        positions = {}
        for leg_id in LegID:
            foot = self.neutral_feet[leg_id]
            angles = self.ik.solve(*foot,
                                   is_front=self.leg_is_front[leg_id])
            for i, cfg in enumerate(LEG_SERVO_CONFIG[leg_id]):
                positions[cfg.servo_id] = angle_to_raw(angles[i], cfg)
        self.servos.sync_write_positions(positions)
        print("Standing")

    def smooth_stand(self, duration: float = 1.5):
        """Gradually move to standing - prevents jerky startup."""
        for sid in self._all_servo_ids():
            self.servos.set_moving_speed(sid, 100)
            time.sleep(0.003)
        self.stand()
        time.sleep(duration)
        for sid in self._all_servo_ids():
            self.servos.set_moving_speed(sid, 0)  # Back to max for walking
            time.sleep(0.003)

    def start(self):
        if self._running:
            return
        self._running = True
        self._phase = 0.0
        self._walk_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._walk_thread.start()
        print("Walking controller started")

    def stop(self):
        if not self._running:
            return
        self._running = False
        if self._walk_thread:
            self._walk_thread.join(timeout=2.0)
            self._walk_thread = None
        self.smooth_stand(0.8)
        print("Walking controller stopped")

    def _control_loop(self):
        """Main walking loop at configured update rate."""
        dt = 1.0 / self.gait_cfg.update_rate_hz

        while self._running:
            t0 = time.monotonic()

            with self._lock:
                speed = self._speed
                turn = self._turn
                phase_offsets = dict(self.gait_phases)

            is_moving = abs(speed) > 0.01 or abs(turn) > 0.01
            positions = {}
            debug_angles = {}

            for leg_id in LegID:
                leg_phase = (self._phase + phase_offsets[leg_id]) % 1.0

                if abs(turn) > 0.01 and abs(speed) < 0.01:
                    # Pure turning
                    hip = self.hip_positions[leg_id]
                    dx, dy, dz = self.trajectory.compute_turn(
                        leg_phase, turn, hip[0], hip[1])
                elif abs(turn) > 0.01:
                    # Combined forward + turning
                    dx_fwd, _, dz_fwd = self.trajectory.compute(leg_phase, speed)
                    hip = self.hip_positions[leg_id]
                    dx_trn, dy_trn, dz_trn = self.trajectory.compute_turn(
                        leg_phase, turn * 0.5, hip[0], hip[1])
                    dx = dx_fwd + dx_trn
                    dy = dy_trn
                    dz = max(dz_fwd, dz_trn)
                else:
                    # Straight walking
                    dx, dy, dz = self.trajectory.compute(leg_phase, speed)

                if not is_moving:
                    dx, dy, dz = 0.0, 0.0, 0.0

                nx, ny, nz = self.neutral_feet[leg_id]
                foot = (nx + dx, ny + dy, nz + dz)

                try:
                    angles = self.ik.solve(*foot,
                                           is_front=self.leg_is_front[leg_id])
                    # Override coxa with locked standing angle to prevent drift
                    angles = (self.locked_coxa_angles[leg_id], angles[1], angles[2])
                except Exception as e:
                    print(f"IK fail {leg_id.value}: {e}")
                    continue

                for i, cfg in enumerate(LEG_SERVO_CONFIG[leg_id]):
                    raw = angle_to_raw(angles[i], cfg)
                    positions[cfg.servo_id] = raw
                    debug_angles[cfg.servo_id] = angles[i]

            self.servos.sync_write_positions(positions)

            if self.on_update and debug_angles:
                self.on_update(debug_angles)

            # Audit log: record every Nth cycle when moving
            if is_moving:
                self._audit_cycle += 1
                if self._audit_cycle % self._audit_interval == 0:
                    # Compact: cycle, phase, speed, then 12 angles as integers
                    vals = [str(self._audit_cycle),
                            f"{self._phase:.2f}",
                            f"{speed:.1f}"]
                    for lid in LegID:
                        for cfg in LEG_SERVO_CONFIG[lid]:
                            a = debug_angles.get(cfg.servo_id, 0)
                            vals.append(f"{a:.0f}")
                    self._audit_log.append(",".join(vals))

                self._phase = (self._phase + dt / self.gait_cfg.cycle_time) % 1.0

            elapsed = time.monotonic() - t0
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)


# ============================================================================
# KEYBOARD CONTROLLER
# ============================================================================

def run_keyboard_control(walker: QuadrupedWalker):
    """Terminal keyboard control for testing."""
    print("\n" + "=" * 50)
    print("  ROBOKITTY3 KEYBOARD CONTROL")
    print("=" * 50)
    print("  w/s     = forward / backward")
    print("  a/d     = turn left / right")
    print("  SPACE   = stop movement")
    print("  1       = trot gait")
    print("  2       = walk gait (slow, stable)")
    print("  3       = pace gait")
    print("  i       = identify servos (flash LEDs)")
    print("  t       = test RR shoulder servo (sweep in/out)")
    print("  r       = diagnose RR shoulder + scan all servos")
    print("  p       = read current servo positions")
    print("  q       = quit")
    print("=" * 50 + "\n")

    speed = 0.0
    turn = 0.0
    speed_inc = 0.2

    try:
        import tty
        import termios
        import select

        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        try:
            while True:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == 'q':
                        break
                    elif key == 'w':
                        speed = min(1.0, speed + speed_inc)
                    elif key == 's':
                        speed = max(-1.0, speed - speed_inc)
                    elif key == 'a':
                        turn = max(-1.0, turn - 0.3)
                    elif key == 'd':
                        turn = min(1.0, turn + 0.3)
                    elif key == ' ':
                        speed = 0.0
                        turn = 0.0
                    elif key == '1':
                        walker.set_gait(GaitType.TROT)
                    elif key == '2':
                        walker.set_gait(GaitType.WALK)
                    elif key == '3':
                        walker.set_gait(GaitType.PACE)
                    elif key == 'i':
                        print("\n  Identifying servos...")
                        walker.identify_all_servos()
                        print("  Done!\n")
                    elif key == 't':
                        was_running = walker._running
                        if was_running:
                            walker._running = False
                            if walker._walk_thread:
                                walker._walk_thread.join(timeout=2.0)
                            time.sleep(0.1)
                        print("\n  Testing RR Shoulder (ID 2) - sweeping in/out...")
                        walker.test_rr_shoulder()
                        print("  Done!\n")
                        if was_running:
                            walker._running = True
                            walker._walk_thread = threading.Thread(
                                target=walker._control_loop, daemon=True)
                            walker._walk_thread.start()
                    elif key == 'r':
                        # Must stop walk loop so bus is free for reads
                        was_running = walker._running
                        if was_running:
                            walker._running = False
                            if walker._walk_thread:
                                walker._walk_thread.join(timeout=2.0)
                            time.sleep(0.1)
                        print("\n  Diagnosing RR Shoulder (ID 2)...")
                        walker.diagnose_servo(2)
                        # Also scan all servos for errors
                        print("\n  Scanning ALL servos for errors...")
                        walker.scan_all_errors()
                        if was_running:
                            walker._running = True
                            walker._walk_thread = threading.Thread(
                                target=walker._control_loop, daemon=True)
                            walker._walk_thread.start()
                        print("  Done!\n")
                    elif key == 'p':
                        was_running = walker._running
                        if was_running:
                            walker._running = False
                            if walker._walk_thread:
                                walker._walk_thread.join(timeout=2.0)
                            time.sleep(0.1)
                        print("\n  Reading current positions...")
                        walker.read_all_positions()
                        if was_running:
                            walker._running = True
                            walker._walk_thread = threading.Thread(
                                target=walker._control_loop, daemon=True)
                            walker._walk_thread.start()

                    walker.set_speed(speed)
                    walker.set_turn(turn)
                    sys.stdout.write(f"\r  Speed: {speed:+.1f}  Turn: {turn:+.1f}  "
                                     f"Gait: {walker.gait_type.value}       ")
                    sys.stdout.flush()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    except ImportError:
        print("(Line-input mode - type command and press Enter)")
        while True:
            try:
                cmd = input(f"[spd={speed:+.1f} trn={turn:+.1f}] > ").strip().lower()
                if cmd == 'q':
                    break
                elif cmd == 'w':
                    speed = min(1.0, speed + speed_inc)
                elif cmd == 's':
                    speed = max(-1.0, speed - speed_inc)
                elif cmd == 'a':
                    turn = max(-1.0, turn - 0.3)
                elif cmd == 'd':
                    turn = min(1.0, turn + 0.3)
                elif cmd in ('x', ''):
                    speed = 0.0
                    turn = 0.0
                elif cmd == '1':
                    walker.set_gait(GaitType.TROT)
                elif cmd == '2':
                    walker.set_gait(GaitType.WALK)
                elif cmd == 'i':
                    walker.identify_all_servos()
                walker.set_speed(speed)
                walker.set_turn(turn)
            except (EOFError, KeyboardInterrupt):
                break


# ============================================================================
# DIAGNOSTICS
# ============================================================================

def print_diagnostics(walker: QuadrupedWalker):
    """Print IK solution for standing pose."""
    print("\n--- RoboKitty3 Standing Pose Diagnostics ---")
    print(f"Leg dims: coxa={walker.leg_dims.coxa_length}mm "
          f"femur={walker.leg_dims.femur_length}mm "
          f"tibia={walker.leg_dims.tibia_length}mm")
    print(f"Body: half_length={walker.body_dims.half_length}mm "
          f"half_width={walker.body_dims.half_width}mm")
    print(f"Standing height: {walker.gait_cfg.body_height}mm")
    print(f"Max leg reach: {walker.leg_dims.femur_length + walker.leg_dims.tibia_length}mm "
          f"(using {walker.gait_cfg.body_height / (walker.leg_dims.femur_length + walker.leg_dims.tibia_length) * 100:.0f}%)")
    print()

    for leg_id in LegID:
        foot = walker.neutral_feet[leg_id]
        is_front = walker.leg_is_front[leg_id]
        angles = walker.ik.solve(*foot, is_front=is_front)
        configs = LEG_SERVO_CONFIG[leg_id]
        joint_names = ["Shoulder", "Femur   ", "Leg     "]

        print(f"  {leg_id.value:12s}  foot=({foot[0]:6.1f}, {foot[1]:6.1f}, {foot[2]:6.1f})"
              f"  {'FRONT' if is_front else 'REAR'}")
        for i, (cfg, name) in enumerate(zip(configs, joint_names)):
            raw = angle_to_raw(angles[i], cfg)
            inv = " INV" if cfg.inverted else ""
            print(f"    {name} ID{cfg.servo_id:2d}: {angles[i]:+7.1f}deg -> raw={raw:4d}"
                  f"  (offset={cfg.offset_deg:+.1f}deg{inv})")
    print()


# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="RoboKitty3 IK Walking Controller")
    parser.add_argument("--simulate", action="store_true",
                        help="Run without hardware")
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"Baudrate (default: {DEFAULT_BAUD})")
    parser.add_argument("--dir-pin", type=int, default=None,
                        help="GPIO BCM pin for half-duplex direction")
    parser.add_argument("--stand", action="store_true",
                        help="Stand only (calibration mode)")
    parser.add_argument("--diag", action="store_true",
                        help="Print IK diagnostics and exit")
    parser.add_argument("--identify", action="store_true",
                        help="Flash each servo LED to verify wiring")
    parser.add_argument("--read-pose", action="store_true",
                        help="Read current servo positions (torque off, pose manually)")
    parser.add_argument("--calibrate", action="store_true",
                        help="Pose legs by hand, read positions, compute offsets")
    args = parser.parse_args()

    walker = QuadrupedWalker(
        simulate=args.simulate,
        port=args.port,
        baudrate=args.baud,
        direction_pin=args.dir_pin,
    )

    if args.diag:
        print_diagnostics(walker)
        return

    if not walker.connect():
        print("Failed to connect. Check port and power.")
        return

    def signal_handler(sig, frame):
        print("\nShutting down...")
        walker.disconnect()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    if args.identify:
        print("\nIdentifying all servos by flashing LEDs...")
        walker.identify_all_servos()
        walker.disconnect()
        return

    if args.read_pose:
        # Disable torque so servos can be moved by hand
        print("\nDisabling torque on all servos for manual posing...")
        all_ids = walker._all_servo_ids()
        for sid in all_ids:
            walker.servos.enable_torque(sid, False)
            time.sleep(0.003)
        print("Torque OFF. Pose the robot into the desired standing position.")
        print("Press Enter when ready to read positions...")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            walker.disconnect()
            return
        walker.read_all_positions()
        walker.disconnect()
        return

    if args.calibrate:
        print("\n" + "=" * 62)
        print("  ROBOKITTY3 STANDING CALIBRATION")
        print("=" * 62)
        print("  1. All servo torque will be DISABLED")
        print("  2. Manually pose ALL legs into your desired standing position")
        print("  3. Press Enter to read positions")
        print("  4. Offsets will be calculated and displayed")
        print("=" * 62)

        # Disable torque
        all_ids = walker._all_servo_ids()
        for sid in all_ids:
            walker.servos.enable_torque(sid, False)
            time.sleep(0.003)
        print("\nTorque OFF on all servos. Pose the legs now.")
        print("Press Enter when the robot is in the desired standing position...")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            walker.disconnect()
            return

        walker.calibrate_standing()
        walker.disconnect()
        return

    if args.stand:
        print_diagnostics(walker)
        print("Moving to standing pose (slowly)...")
        walker.smooth_stand(2.0)
        print("Standing. Adjust offsets/inversions as needed. Ctrl+C to exit.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        walker.disconnect()
        return

    # Normal operation
    print_diagnostics(walker)
    walker.smooth_stand(1.5)

    if args.simulate:
        def show_angles(angles):
            parts = [f"{sid}:{a:+5.1f}d" for sid, a in sorted(angles.items())]
            sys.stdout.write(f"\r  {' '.join(parts)}    ")
            sys.stdout.flush()
        walker.on_update = show_angles

    walker.start()

    try:
        run_keyboard_control(walker)
    finally:
        walker.disconnect()
        print("\nRoboKitty3 shutting down. Bye!")


if __name__ == "__main__":
    main()