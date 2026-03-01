#!/usr/bin/env python3
"""
Quadruped IK Walking Controller for RoboKitty
==============================================
12-DOF (3 per leg) inverse kinematics with trot/walk gaits.
No ROS. Pure Python. AX-12A Dynamixel servos via half-duplex UART.
Target: Raspberry Pi 4 (8GB)

Leg layout (top view, front facing up):
    FL (Front-Left)     FR (Front-Right)
    RL (Rear-Left)      RR (Rear-Right)

Each leg: Coxa (shoulder) → Femur (upper) → Tibia (lower)

Coordinate convention per leg:
    X = forward (+) / backward (-)
    Y = lateral outward (+left, -right)
    Z = up (+) / down (-)

Hardware:
    - AX-12A servos @ 11.1V nominal (3S 18650)
    - Raspberry Pi 4 8GB
    - Carbon fibre cylindrical feet (17mm dia x 29mm) with rubber wrap
    - Total weight: ~1.65kg
    - Surface: short fibre carpet

Usage:
    python3 quadruped_ik_walker.py --simulate     # Test without hardware
    python3 quadruped_ik_walker.py                # Run with real servos
    python3 quadruped_ik_walker.py --stand        # Stand only (calibration)
    python3 quadruped_ik_walker.py --diag         # Print IK diagnostics
    python3 quadruped_ik_walker.py --identify     # Flash servo LEDs to verify wiring

Author: Built for Chris's RoboKitty project
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
# CONFIGURATION — ROBOKITTY ACTUAL MEASUREMENTS
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
    """Gait timing and geometry — tuned for 1.65kg on carpet."""
    step_height: float = 35.0      # Foot lift height mm (carpet needs decent clearance)
    step_length: float = 45.0      # Forward travel per step mm
    cycle_time: float = 1.0        # Full gait cycle seconds
    duty_factor: float = 0.5       # 0.5=trot, 0.75=walk
    body_height: float = 85.0      # Standing height mm (low CG for long body)
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
    min_deg: float = -90.0       # Software limit
    max_deg: float = 90.0        # Software limit


# AX-12A constants
AX12_CENTER = 512
AX12_DEG_TO_UNITS = 1023.0 / 300.0  # ~3.41 units per degree

# ============================================================================
# ROBOKITTY SERVO MAP — ACTUAL SERVO IDS FROM CHRIS'S WIRING
# ============================================================================
# CALIBRATION PROCESS:
#   1. Run: python3 quadruped_ik_walker.py --identify
#      → Verify each LED flashes on the correct physical servo
#   2. Run: python3 quadruped_ik_walker.py --stand
#      → If a joint moves the wrong direction: toggle inverted
#      → If standing pose is crooked: adjust offset_deg
#   3. Repeat until robot stands square on all four feet

LEG_SERVO_CONFIG: Dict[LegID, List[ServoJointConfig]] = {
    # [Shoulder/Coxa, Femur, Tibia/Leg]
    LegID.FL: [
        ServoJointConfig(servo_id=8,  offset_deg=0.0, inverted=False),   # FL Shoulder
        ServoJointConfig(servo_id=10, offset_deg=0.0, inverted=False),   # FL Femur
        ServoJointConfig(servo_id=0,  offset_deg=0.0, inverted=False),   # FL Leg
    ],
    LegID.FR: [
        ServoJointConfig(servo_id=11, offset_deg=0.0, inverted=True),    # FR Shoulder
        ServoJointConfig(servo_id=9,  offset_deg=0.0, inverted=True),    # FR Femur
        ServoJointConfig(servo_id=7,  offset_deg=0.0, inverted=True),    # FR Leg
    ],
    LegID.RL: [
        ServoJointConfig(servo_id=5,  offset_deg=0.0, inverted=False),   # RL Shoulder
        ServoJointConfig(servo_id=3,  offset_deg=0.0, inverted=False),   # RL Femur
        ServoJointConfig(servo_id=6,  offset_deg=0.0, inverted=False),   # RL Leg
    ],
    LegID.RR: [
        ServoJointConfig(servo_id=2,  offset_deg=0.0, inverted=True),    # RR Shoulder
        ServoJointConfig(servo_id=1,  offset_deg=0.0, inverted=True),    # RR Femur
        ServoJointConfig(servo_id=4,  offset_deg=0.0, inverted=True),    # RR Leg
    ],
}

# Serial config
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 1000000


# ============================================================================
# INVERSE KINEMATICS — 3DOF LEG
# ============================================================================

class LegIK:
    """
    Solves joint angles for a 3-DOF leg given a desired foot position
    relative to the hip/coxa joint.

    Geometry:
        Coxa rotates in the horizontal plane (XY).
        Femur and Tibia operate in the leg's sagittal plane.
    """

    def __init__(self, dims: LegDimensions):
        self.L1 = dims.coxa_length
        self.L2 = dims.femur_length
        self.L3 = dims.tibia_length

    def solve(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """
        Compute (coxa_deg, femur_deg, tibia_deg) for foot at (x, y, z)
        relative to the hip joint origin.

        x: forward(+) / back(-)
        y: lateral outward (sign handled by caller for L vs R)
        z: up(+) / down(-), typically negative when standing

        Returns angles in degrees. Clamps to reachable workspace.
        """
        # Coxa angle (horizontal rotation)
        coxa_rad = math.atan2(y, x)

        # Project into the leg's 2D sagittal plane
        d_horiz = math.sqrt(x * x + y * y) - self.L1
        d_vert = -z  # Positive downward for the leg

        # Direct distance from femur pivot to foot
        d = math.sqrt(d_horiz * d_horiz + d_vert * d_vert)

        # Clamp to reachable range
        reach_max = self.L2 + self.L3 - 0.1
        reach_min = abs(self.L2 - self.L3) + 0.1
        d = max(reach_min, min(reach_max, d))

        # Tibia (knee) angle via law of cosines
        cos_knee = (self.L2**2 + self.L3**2 - d**2) / (2.0 * self.L2 * self.L3)
        cos_knee = max(-1.0, min(1.0, cos_knee))
        knee_rad = math.acos(cos_knee)
        tibia_deg = -math.degrees(math.pi - knee_rad)

        # Femur (hip pitch) angle
        alpha = math.atan2(d_vert, d_horiz)
        cos_beta = (self.L2**2 + d**2 - self.L3**2) / (2.0 * self.L2 * d)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)
        femur_deg = math.degrees(alpha + beta) - 90.0

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
            dz = self.cfg.step_height * 0.5 * (1.0 - math.cos(2.0 * math.pi * t))

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
            dz = self.cfg.step_height * 0.5 * (1.0 - math.cos(2.0 * math.pi * t))

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

    INST_WRITE = 0x03
    INST_REG_WRITE = 0x04
    INST_ACTION = 0x05
    INST_SYNC_WRITE = 0x83

    ADDR_TORQUE_ENABLE = 24
    ADDR_LED = 25
    ADDR_GOAL_POSITION = 30
    ADDR_MOVING_SPEED = 32

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

    def enable_torque(self, servo_id: int, enable: bool = True):
        self._send_packet(servo_id, self.INST_WRITE,
                          bytes([self.ADDR_TORQUE_ENABLE, 1 if enable else 0]))
        time.sleep(0.001)

    def set_led(self, servo_id: int, on: bool = True):
        """Flash servo LED — useful for identifying physical servo location."""
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
        All servos start moving simultaneously — critical for smooth gait.
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
# ANGLE → RAW POSITION CONVERSION
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

        # Neutral foot positions relative to each hip
        L1 = self.leg_dims.coxa_length
        foot_spread = L1 + 15.0   # Slight outward offset beyond coxa
        h = self.gait_cfg.body_height

        self.neutral_feet = {
            LegID.FL: (0.0,  foot_spread, -h),
            LegID.FR: (0.0, -foot_spread, -h),
            LegID.RL: (0.0,  foot_spread, -h),
            LegID.RR: (0.0, -foot_spread, -h),
        }

        # Runtime state
        self._running = False
        self._walk_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._speed = 0.0
        self._turn = 0.0
        self._phase = 0.0
        self.on_update = None  # Optional telemetry callback

    def connect(self) -> bool:
        if not self.servos.connect():
            return False
        all_ids = self._all_servo_ids()
        for sid in all_ids:
            self.servos.enable_torque(sid, True)
            self.servos.set_moving_speed(sid, 250)  # Smooth at 11.1V
            time.sleep(0.003)
        print(f"All {len(all_ids)} servos enabled")
        return True

    def disconnect(self):
        self.stop()
        all_ids = self._all_servo_ids()
        for sid in all_ids:
            self.servos.enable_torque(sid, False)
            time.sleep(0.003)
        self.servos.disconnect()

    def _all_servo_ids(self) -> List[int]:
        ids = []
        for leg_id in LegID:
            for cfg in LEG_SERVO_CONFIG[leg_id]:
                ids.append(cfg.servo_id)
        return ids

    def identify_all_servos(self):
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
                self.gait_cfg.cycle_time = 2.0
            else:
                self.gait_cfg.duty_factor = 0.5
                self.gait_cfg.cycle_time = 1.0
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
            angles = self.ik.solve(*foot)
            for i, cfg in enumerate(LEG_SERVO_CONFIG[leg_id]):
                positions[cfg.servo_id] = angle_to_raw(angles[i], cfg)
        self.servos.sync_write_positions(positions)
        print("Standing")

    def smooth_stand(self, duration: float = 1.5):
        """Gradually move to standing — prevents jerky startup."""
        for sid in self._all_servo_ids():
            self.servos.set_moving_speed(sid, 100)
            time.sleep(0.003)
        self.stand()
        time.sleep(duration)
        for sid in self._all_servo_ids():
            self.servos.set_moving_speed(sid, 250)
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
                    angles = self.ik.solve(*foot)
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

            if is_moving:
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
    print("  ROBOKITTY KEYBOARD CONTROL")
    print("=" * 50)
    print("  w/s     = forward / backward")
    print("  a/d     = turn left / right")
    print("  SPACE   = stop movement")
    print("  1       = trot gait")
    print("  2       = walk gait (slow, stable)")
    print("  3       = pace gait")
    print("  i       = identify servos (flash LEDs)")
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

                    walker.set_speed(speed)
                    walker.set_turn(turn)
                    sys.stdout.write(f"\r  Speed: {speed:+.1f}  Turn: {turn:+.1f}  "
                                     f"Gait: {walker.gait_type.value}       ")
                    sys.stdout.flush()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    except ImportError:
        print("(Line-input mode — type command and press Enter)")
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
    print("\n--- RoboKitty Standing Pose Diagnostics ---")
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
        angles = walker.ik.solve(*foot)
        configs = LEG_SERVO_CONFIG[leg_id]
        joint_names = ["Shoulder", "Femur   ", "Leg     "]

        print(f"  {leg_id.value:12s}  foot=({foot[0]:6.1f}, {foot[1]:6.1f}, {foot[2]:6.1f})")
        for i, (cfg, name) in enumerate(zip(configs, joint_names)):
            raw = angle_to_raw(angles[i], cfg)
            inv = " INV" if cfg.inverted else ""
            print(f"    {name} ID{cfg.servo_id:2d}: {angles[i]:+7.1f}° → raw={raw:4d}"
                  f"  (offset={cfg.offset_deg:+.1f}°{inv})")
    print()


# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="RoboKitty IK Walking Controller")
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
            parts = [f"{sid}:{a:+5.1f}°" for sid, a in sorted(angles.items())]
            sys.stdout.write(f"\r  {' '.join(parts)}    ")
            sys.stdout.flush()
        walker.on_update = show_angles

    walker.start()

    try:
        run_keyboard_control(walker)
    finally:
        walker.disconnect()
        print("\nRoboKitty shutting down. Bye!")


if __name__ == "__main__":
    main()
