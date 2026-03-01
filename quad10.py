#!/usr/bin/env python3
"""
RoboKitty Quadruped IK Walker - v0.46 (2026-02-15)
12-DOF cat-style quadruped. AX-12A servos. Raspberry Pi 4. No ROS.

Geometry: Coxa=sideways, Femur/Tibia=fore-aft sagittal plane.
Legs: FL, FR, RL, RR. Each: Coxa->Femur->Tibia.
Coords per leg: X=fwd/back, Y=lateral(+left,-right), Z=up/down.
Hardware: AX-12A @11.1V 3S, CF feet 17mm, ~1.65kg total.

Usage:
    python3 quadruped_ik_walker.py              # Run with servos
    python3 quadruped_ik_walker.py --stand      # Stand only (calibration)
    python3 quadruped_ik_walker.py --identify   # Flash servo LEDs
    python3 quadruped_ik_walker.py --diag       # Print IK diagnostics
    python3 quadruped_ik_walker.py --simulate   # Test without hardware
"""

import math
import time
import threading
import argparse
import signal
import sys
import csv
import os
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
    step_height: float = 15.0      # Foot lift height mm (gentle)
    step_length: float = 80.0      # Forward travel per step mm (good front reach + rear push)
    cycle_time: float = 2.5        # Full gait cycle seconds (walk default)
    duty_factor: float = 0.75      # 0.75=walk (3 feet always down)
    body_height: float = 130.0     # Standing height mm
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
# ROBOKITTY SERVO MAP - ACTUAL SERVO IDS FROM CHRIS'S WIRING
# ============================================================================
# CALIBRATION PROCESS:
#   1. Run: python3 quadruped_ik_walker.py --identify
#      -> Verify each LED flashes on the correct physical servo
#   2. Run: python3 quadruped_ik_walker.py --stand
#      -> If a joint moves the wrong direction: toggle inverted
#      -> If standing pose is crooked: adjust offset_deg
#   3. Repeat until robot stands square on all four feet

LEG_SERVO_CONFIG: Dict[LegID, List[ServoJointConfig]] = {
    # [Shoulder/Coxa, Femur, Tibia/Leg]
    # Left legs: coxa positive = tilts leg outward/down. No inversion needed.
    # Right legs: servo is mirrored, so coxa needs inversion.
    # Femur/Tibia: may need inversion depending on servo mounting direction.
    #              Start with right-side inverted and adjust from --stand test.
    LegID.FL: [
        ServoJointConfig(servo_id=8,  offset_deg=0.0, inverted=True),    # FL Shoulder FLIPPED
        ServoJointConfig(servo_id=10, offset_deg=0.0, inverted=False),  # FL Femur
        ServoJointConfig(servo_id=0,  offset_deg=0.0, inverted=False),   # FL Leg     FLIPPED
    ],
    LegID.FR: [
        ServoJointConfig(servo_id=11, offset_deg=0.0,  inverted=False),   # FR Shoulder FLIPPED
        ServoJointConfig(servo_id=9,  offset_deg=0.0, inverted=True),    # FR Femur
        ServoJointConfig(servo_id=7,  offset_deg=0.0, inverted=True),    # FR Leg
    ],
    LegID.RL: [
        ServoJointConfig(servo_id=5,  offset_deg=0.0, inverted=False),   # RL Shoulder
        ServoJointConfig(servo_id=3,  offset_deg=0.0, inverted=True),    # RL Femur   FLIPPED
        ServoJointConfig(servo_id=6,  offset_deg=46.0, inverted=True),    # RL Leg (horn offset)
    ],
    LegID.RR: [
        ServoJointConfig(servo_id=2,  offset_deg=-89.5, inverted=True),   # RR Shoulder (drift-compensated from -86)
        ServoJointConfig(servo_id=1,  offset_deg=-5.0, inverted=False),   # RR Femur (aligned)
        ServoJointConfig(servo_id=4,  offset_deg=49.0, inverted=False),   # RR Leg (horn offset)
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
    Solves joint angles for a 3-DOF cat/dog-style leg.

    RoboKitty leg geometry:
        - Coxa/Shoulder: axis runs FORE-AFT along body length.
          Swings the leg sideways in the frontal plane.
          0 deg = coxa link pointing straight out sideways (horizontal).
          Positive = tilts downward (outward from body when leg hangs).

        - Femur: axis runs LATERALLY.
          Swings upper leg fore/aft in the sagittal plane.
          0 deg = femur hanging straight down.
          Positive = swings forward.

        - Tibia: axis runs LATERALLY.
          Swings lower leg fore/aft in the sagittal plane.
          0 deg = continues straight from femur direction.
          Negative = knee bends (foot goes backward relative to femur).

    Since femur/tibia only swing fore/aft, they contribute ZERO lateral
    movement. All lateral positioning comes from the coxa link alone.
    Therefore: max lateral foot position = coxa_length.

    Foot position relative to hip joint:
        x = forward (+) / backward (-)
        y = lateral: positive = left, negative = right
        z = down is negative (standing z ~ -85mm)
    """

    def __init__(self, dims: LegDimensions):
        self.L1 = dims.coxa_length   # coxa (horizontal sideways link)
        self.L2 = dims.femur_length  # upper leg
        self.L3 = dims.tibia_length  # lower leg to foot

    def solve(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """
        Compute (coxa_deg, femur_deg, tibia_deg) for foot at (x, y, z).
        """
        # === COXA: frontal plane (looking from front of robot) ===
        # Coxa link extends sideways then the leg hangs down from its tip.
        # In the frontal plane:
        #   lateral = |y|, vertical_down = -z
        # The coxa tip is at: (L1 * cos(coxa), L1 * sin(coxa))
        #   where coxa=0 is horizontal sideways
        # The foot hangs some distance D below the coxa tip (from femur+tibia).
        # So: lateral = L1 * cos(coxa)
        #     vertical_down = L1 * sin(coxa) + D
        
        foot_lateral = abs(y)
        foot_down = -z  # positive downward
        
        # Clamp lateral to coxa length (can't reach further sideways)
        foot_lateral = min(foot_lateral, self.L1 - 0.1)
        
        # Coxa angle from horizontal
        coxa_rad = math.acos(foot_lateral / self.L1)
        
        # Distance below coxa tip that femur+tibia must reach
        # Coxa tip is at height: L1 * sin(coxa_angle) below the hip
        coxa_tip_drop = self.L1 * math.sin(coxa_rad)
        D_vertical = foot_down - coxa_tip_drop  # remaining vertical for leg
        
        # === FEMUR + TIBIA: sagittal plane (side view) ===
        # From the coxa tip, looking from the side:
        #   forward/back = x
        #   downward = D_vertical
        # Femur pivot is at coxa tip. Leg must reach (x, D_vertical).
        
        d_forward = x
        d_down = D_vertical
        
        # Distance from femur pivot to foot
        d = math.sqrt(d_forward * d_forward + d_down * d_down)
        
        # Clamp to reachable range
        reach_max = self.L2 + self.L3 - 0.1
        reach_min = abs(self.L2 - self.L3) + 0.1
        d = max(reach_min, min(reach_max, d))
        
        # Knee (tibia) angle via law of cosines
        cos_knee = (self.L2**2 + self.L3**2 - d**2) / (2.0 * self.L2 * self.L3)
        cos_knee = max(-1.0, min(1.0, cos_knee))
        knee_rad = math.acos(cos_knee)
        tibia_deg = -math.degrees(math.pi - knee_rad)  # negative = bent
        
        # Femur angle: 0 = straight down
        alpha = math.atan2(d_forward, d_down)  # angle from vertical towards forward
        cos_beta = (self.L2**2 + d**2 - self.L3**2) / (2.0 * self.L2 * d)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)
        femur_deg = math.degrees(alpha + beta)
        
        coxa_deg = math.degrees(coxa_rad)
        
        return (coxa_deg, femur_deg, tibia_deg)


# ============================================================================
# GAIT GENERATOR
# ============================================================================

class GaitType(Enum):
    TROT = "trot"
    WALK = "walk"
    CREEP = "creep"


GAIT_PHASES = {
    # Trot: diagonal pairs (FL+RR, FR+RL) - classic stable trot
    GaitType.TROT: {
        LegID.FL: 0.0, LegID.FR: 0.5,
        LegID.RL: 0.5, LegID.RR: 0.0,
    },
    # Walk: 4-beat lateral sequence (same-side legs alternate, like a real cat)
    # Sequence: RL -> FL -> RR -> FR (each 25% apart)
    GaitType.WALK: {
        LegID.FL: 0.25, LegID.FR: 0.75,
        LegID.RL: 0.0,  LegID.RR: 0.5,
    },
    # Creep: ultra-stable 4-beat with 75% duty (3 feet always on ground)
    GaitType.CREEP: {
        LegID.FL: 0.25, LegID.FR: 0.75,
        LegID.RL: 0.0,  LegID.RR: 0.5,
    },
}


class FootTrajectory:
    """
    Generates smooth foot paths through the gait cycle.

    All four legs use identical kinematics (like a real cat):
      - Stance: foot pushes backward relative to body (propulsion)
      - Swing: foot lifts and reaches forward to prepare for next stance

    The foot neutral position can be biased slightly forward of the hip
    (set in QuadrupedWalker.neutral_feet) so all legs have a natural
    forward-reaching posture at mid-stance, matching feline anatomy.
    """

    def __init__(self, cfg: GaitConfig):
        self.cfg = cfg

    def compute(self, phase: float, speed: float = 1.0) -> Tuple[float, float, float]:
        """
        Get (dx, dy, dz) foot offset from neutral position.

        Cat-like foot trajectory:
          Swing: foot lifts, reaches far forward, then angles DOWN toward
                 the ground (toe-strike entry, like a cat extending its paw).
          Stance: slight settle as paw flattens, then smooth backward push.

        phase: 0.0 to 1.0 within the gait cycle
        speed: -1.0 to 1.0. Positive = forward, negative = backward.
        """
        if abs(speed) < 0.01:
            return (0.0, 0.0, 0.0)

        duty = self.cfg.duty_factor
        half_step = self.cfg.step_length * 0.5
        step_h = self.cfg.step_height

        if phase < duty:
            # STANCE: foot on ground, slides backward to push body forward
            t = phase / duty

            # Smooth backward push (cosine interpolation)
            dx = half_step * math.cos(math.pi * t) * speed

            # Paw-settle: slight dip at start of stance as paw flattens
            # onto ground, then returns to ground plane. Peak dip ~2mm at t=0.1
            if t < 0.2:
                dz = -2.0 * math.sin(math.pi * t / 0.2)
            else:
                dz = 0.0
        else:
            # SWING: lift, reach forward, angle down to touch
            t = (phase - duty) / (1.0 - duty)

            # Forward reach: cosine gives smooth acceleration
            dx = -half_step * math.cos(math.pi * t) * speed

            # Cat-like swing profile:
            #   0.0-0.5: lift and carry forward (sine rise to peak)
            #   0.5-0.85: hold height while reaching further forward
            #   0.85-1.0: angle foot DOWN to meet ground (toe-strike)
            if t < 0.5:
                # Quick lift to peak height
                dz = step_h * math.sin(math.pi * t)
            elif t < 0.85:
                # Hold at peak while leg extends forward
                dz = step_h
            else:
                # Angled descent: foot reaches down toward ground
                # Cosine descent for smooth deceleration at touchdown
                descent_t = (t - 0.85) / 0.15
                dz = step_h * 0.5 * (1.0 + math.cos(math.pi * descent_t))

        return (dx, 0.0, dz)

    def compute_turn(self, phase: float, turn_rate: float,
                     hip_x: float, hip_y: float) -> Tuple[float, float, float]:
        """
        Foot offset for turning.

        Rotates each foot position around the body center.
        Positive turn_rate = turn RIGHT (clockwise from above).
        Negative turn_rate = turn LEFT (counter-clockwise from above).

        hip_x, hip_y: this leg's hip position relative to body center
        """
        if abs(turn_rate) < 0.01:
            return (0.0, 0.0, 0.0)

        duty = self.cfg.duty_factor
        # 25 degrees max yaw per cycle - strong enough to actually turn
        max_yaw = math.radians(25.0) * turn_rate

        if phase < duty:
            # STANCE: foot rotates one way (pushes body into the turn)
            t = phase / duty
            yaw = max_yaw * (0.5 - t)
            dz = 0.0
        else:
            # SWING: foot lifts and rotates back to start position
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

    INST_PING = 0x01
    INST_READ = 0x02
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

    def _send_read(self, servo_id: int, addr: int, length: int):
        """Send READ instruction, return raw response bytes."""
        if not self._connected:
            return None
        self.serial.reset_input_buffer()
        self._send_packet(servo_id, self.INST_READ, bytes([addr, length]))
        time.sleep(0.005)
        return self.serial.read(64)

    def read_position(self, servo_id: int):
        """Read current position. Returns (raw_pos, error_byte) or (None, None)."""
        resp = self._send_read(servo_id, 36, 2)
        if resp and len(resp) >= 8:
            return resp[5] | (resp[6] << 8), resp[4]
        return None, None

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

    def _send_read(self, servo_id: int, addr: int, length: int):
        return None

    def read_position(self, servo_id: int):
        pos = self._positions.get(servo_id, 512)
        return pos, 0

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

        self.gait_type = GaitType.WALK
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
        # Real cat anatomy (viewed from the side):
        #   Front legs: foot lands AHEAD of hip. Femur angles forward,
        #               tibia angles back to ground. "Reverse-Z" shape.
        #   Rear legs:  foot lands BEHIND hip. Femur angles backward,
        #               tibia angles forward to ground. "Z" shape.
        # This is NOT symmetrical - fronts and rears are mirror images.
        L1 = self.leg_dims.coxa_length
        foot_lateral = 5.0       # Nearly vertical coxa (cat-like stance)
        front_forward = 25.0     # Front feet ahead of hip (reverse-Z stance)
        rear_backward = -8.0     # Rear feet slightly behind hip (gentle Z)
        rear_drop = 8.0          # Rear hips sit lower (like a real cat)
        h = self.gait_cfg.body_height

        self.neutral_feet = {
            LegID.FL: (front_forward,  foot_lateral, -h),
            LegID.FR: (front_forward, -foot_lateral, -h),
            LegID.RL: (rear_backward,  foot_lateral, -(h + rear_drop)),
            LegID.RR: (rear_backward, -foot_lateral, -(h + rear_drop)),
        }

        # Pre-compute and LOCK the standing coxa angle for each leg.
        # During walking only femur/tibia should change.
        # This prevents shoulder drift caused by foot Z changes
        # affecting the IK coxa calculation.
        self.locked_coxa_angles = {}
        for leg_id in LegID:
            foot = self.neutral_feet[leg_id]
            angles = self.ik.solve(*foot)
            self.locked_coxa_angles[leg_id] = angles[0]
        self._running = False
        self._walk_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._speed = 0.0          # Target speed (set by keyboard)
        self._turn = 0.0           # Target turn (set by keyboard)
        self._ramped_speed = 0.0   # Smoothed actual speed (fed to gait)
        self._ramped_turn = 0.0    # Smoothed actual turn (fed to gait)
        self._ramp_alpha = 0.08    # Smoothing: ~0.24s to 63%, ~0.72s to 95%
        self._phase = 0.0
        self.on_update = None  # Optional telemetry callback

        # Auto-recording: logs every Nth tick while moving, saves on quit
        self._auto_log = []
        self._auto_log_start = 0.0
        self._auto_log_file = None
        self._log_divisor = 10     # Log every 10th tick (~5Hz at 50Hz loop)

    def connect(self) -> bool:
        if not self.servos.connect():
            return False
        all_ids = self._all_servo_ids()
        shoulder_ids = [LEG_SERVO_CONFIG[lid][0].servo_id for lid in LegID]

        # Initialize ALL servos with correct settings
        print("Initializing servo settings...")
        for sid in all_ids:
            # Must disable torque to write EEPROM
            self.servos.enable_torque(sid, False)
            time.sleep(0.003)

            # Set return delay to 50us (register 5, value 25)
            # Prevents timing issues with 50Hz control loop
            self.servos._send_packet(sid, 0x03, bytes([5, 25]))
            time.sleep(0.003)

            # Set CW angle limit to 0 (register 6, 2 bytes)
            self.servos._send_packet(sid, 0x03, bytes([6, 0x00, 0x00]))
            time.sleep(0.003)

            # Set CCW angle limit to 1023 (register 8, 2 bytes)
            self.servos._send_packet(sid, 0x03, bytes([8, 0xFF, 0x03]))
            time.sleep(0.003)

            # Set max torque to 1023 (register 14, 2 bytes)
            self.servos._send_packet(sid, 0x03, bytes([14, 0xFF, 0x03]))
            time.sleep(0.003)

            # Set torque limit to 1023 (register 34, 2 bytes)
            self.servos._send_packet(sid, 0x03, bytes([34, 0xFF, 0x03]))
            time.sleep(0.003)

        # Now enable torque and set speeds
        for sid in all_ids:
            self.servos.enable_torque(sid, True)
            if sid in shoulder_ids:
                self.servos.set_moving_speed(sid, 300)
            else:
                self.servos.set_moving_speed(sid, 0)
            time.sleep(0.003)
        print(f"All {len(all_ids)} servos initialized and enabled")
        return True

    def disconnect(self):
        self.stop()
        self.save_auto_log()
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

    def diagnose_servo(self, servo_id: int):
        """Deep diagnostic for an unresponsive servo."""
        print(f"\n  === Diagnosing Servo ID {servo_id} ===")
        
        # Step 1: Disable then re-enable torque
        print(f"  1. Toggling torque...")
        self.servos.enable_torque(servo_id, False)
        time.sleep(0.5)
        self.servos.enable_torque(servo_id, True)
        time.sleep(0.5)
        
        # Step 2: Set to joint mode (CW limit=0, CCW limit=1023)
        # AX-12A needs both angle limits set for position mode
        print(f"  2. Setting joint mode (CW=0, CCW=1023)...")
        # CW Angle Limit = address 6, 2 bytes
        self.servos._send_packet(servo_id, 0x03,
            bytes([6, 0x00, 0x00]))  # CW limit = 0
        time.sleep(0.01)
        # CCW Angle Limit = address 8, 2 bytes
        self.servos._send_packet(servo_id, 0x03,
            bytes([8, 0xFF, 0x03]))  # CCW limit = 1023
        time.sleep(0.01)
        
        # Step 3: Re-enable torque after mode change
        print(f"  3. Re-enabling torque...")
        self.servos.enable_torque(servo_id, True)
        time.sleep(0.3)
        
        # Step 4: Set moderate speed
        print(f"  4. Setting speed to 200...")
        self.servos.set_moving_speed(servo_id, 200)
        time.sleep(0.1)
        
        # Step 5: Try moving to center
        print(f"  5. Moving to center (512)...")
        self.servos.sync_write_positions({servo_id: 512})
        time.sleep(2.0)
        
        # Step 6: Try a big move
        print(f"  6. Moving to 300...")
        self.servos.sync_write_positions({servo_id: 300})
        time.sleep(2.0)
        
        print(f"  7. Moving to 700...")
        self.servos.sync_write_positions({servo_id: 700})
        time.sleep(2.0)
        
        print(f"  8. Back to center (512)...")
        self.servos.sync_write_positions({servo_id: 512})
        time.sleep(1.0)
        
        print(f"  === Did servo ID {servo_id} move at all? ===")
        print(f"  If NO: likely stripped gears or dead motor.")
        print(f"  If YES: it was stuck in wheel mode, now fixed.")

    def dump_servo_csv(self, filename=None):
        """Read all servo positions and write state to CSV."""
        if filename is None:
            filename = os.path.expanduser("~/servo_state.csv")
        joint_names = ["Shoulder", "Femur", "Leg"]
        rows = []

        for leg_id in LegID:
            configs = LEG_SERVO_CONFIG[leg_id]
            foot = self.neutral_feet[leg_id]
            ik_angles = self.ik.solve(*foot)
            locked_coxa = self.locked_coxa_angles[leg_id]

            for i, cfg in enumerate(configs):
                angle = locked_coxa if i == 0 else ik_angles[i]
                goal_raw = angle_to_raw(angle, cfg)

                # Read actual position from servo
                actual_raw, error_byte = self.servos.read_position(cfg.servo_id)

                rows.append({
                    "leg": leg_id.value,
                    "joint": joint_names[i],
                    "servo_id": cfg.servo_id,
                    "inverted": cfg.inverted,
                    "offset_deg": cfg.offset_deg,
                    "ik_angle_deg": round(angle, 1),
                    "goal_raw": goal_raw,
                    "actual_raw": actual_raw if actual_raw else "N/A",
                    "error": error_byte if error_byte else 0,
                    "diff": abs(goal_raw - actual_raw) if actual_raw else "N/A",
                })

        with open(filename, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)

        # Also print a summary table
        print(f"\n  Servo state written to {filename}")
        print(f"  {'Leg':12s} {'Joint':10s} ID  Inv  Offset  IK_Angle  Goal  Actual  Err  Diff")
        print(f"  {'-'*85}")
        for r in rows:
            print(f"  {r['leg']:12s} {r['joint']:10s} "
                  f"{r['servo_id']:2d}  {str(r['inverted']):5s} "
                  f"{r['offset_deg']:+6.1f}  {r['ik_angle_deg']:+7.1f}   "
                  f"{r['goal_raw']:4}  {str(r['actual_raw']):6s}  "
                  f"{str(r['error']):3s}  {str(r['diff']):4s}")
        print()

    def log_walk_cycle(self, cycles=2, filename=None):
        """Log ALL servo goal positions through walk cycles to CSV.
        Runs the gait generator and records what WOULD be sent to servos."""
        if filename is None:
            filename = os.path.expanduser("~/walk_log.csv")

        dt = 1.0 / self.gait_cfg.update_rate_hz
        phase = 0.0
        rows = []
        phase_offsets = dict(GAIT_PHASES[self.gait_type])
        total_steps = int(cycles * self.gait_cfg.cycle_time * self.gait_cfg.update_rate_hz)

        print(f"  Logging {cycles} cycles ({total_steps} steps) of {self.gait_type.value}...")

        for step in range(total_steps):
            t = step * dt
            row = {"time": round(t, 3), "phase": round(phase, 4)}

            for leg_id in LegID:
                leg_phase = (phase + phase_offsets[leg_id]) % 1.0
                dx, dy, dz = self.trajectory.compute(leg_phase, speed=1.0)
                nx, ny, nz = self.neutral_feet[leg_id]
                foot = (nx + dx, ny + dy, nz + dz)

                try:
                    angles = self.ik.solve(*foot)
                    angles = (self.locked_coxa_angles[leg_id], angles[1], angles[2])
                except Exception:
                    continue

                configs = LEG_SERVO_CONFIG[leg_id]
                joint_names = ["shldr", "femur", "tibia"]
                prefix_map = {LegID.FL: "FL", LegID.FR: "FR", LegID.RL: "RL", LegID.RR: "RR"}
                prefix = prefix_map[leg_id]

                for i, cfg in enumerate(configs):
                    raw = angle_to_raw(angles[i], cfg)
                    row[f"{prefix}_{joint_names[i]}_raw"] = raw
                    row[f"{prefix}_{joint_names[i]}_deg"] = round(angles[i], 1)

                row[f"{prefix}_dx"] = round(dx, 1)
                row[f"{prefix}_dz"] = round(dz, 1)
                row[f"{prefix}_phase"] = round(leg_phase, 3)

            rows.append(row)
            phase = (phase + dt / self.gait_cfg.cycle_time) % 1.0

        # Write CSV
        with open(filename, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)

        # Print summary showing min/max for each servo
        print(f"  Written to {filename}")
        print(f"\n  Servo ranges during {self.gait_type.value} gait:")
        print(f"  {'Servo':20s}  {'Min':>5s}  {'Max':>5s}  {'Range':>5s}  {'Mid':>5s}")
        print(f"  {'-'*60}")

        prefix_map = {LegID.FL: "FL", LegID.FR: "FR", LegID.RL: "RL", LegID.RR: "RR"}
        for leg_id in LegID:
            prefix = prefix_map[leg_id]
            for jname in ["shldr", "femur", "tibia"]:
                key = f"{prefix}_{jname}_raw"
                vals = [r[key] for r in rows if key in r]
                if vals:
                    mn, mx = min(vals), max(vals)
                    print(f"  {prefix} {jname:6s}           {mn:5d}  {mx:5d}  {mx-mn:5d}  {(mn+mx)//2:5d}")
        print()

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
                self.gait_cfg.duty_factor = 0.75  # 75% stance = always 3 feet down
                self.gait_cfg.cycle_time = 2.5    # Moderate 4-beat walk
                self.gait_cfg.step_length = 80.0  # Long stride for visible motion
            elif gait == GaitType.CREEP:
                self.gait_cfg.duty_factor = 0.80  # 80% stance = very stable
                self.gait_cfg.cycle_time = 4.0    # Slow and deliberate
                self.gait_cfg.step_length = 50.0  # Shorter careful steps
            else:  # TROT
                self.gait_cfg.duty_factor = 0.55  # Slightly over 50% for overlap
                self.gait_cfg.cycle_time = 1.5    # Quick diagonal pairs
                self.gait_cfg.step_length = 80.0  # Full stride
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
        """Gradually move to standing - prevents jerky startup."""
        for sid in self._all_servo_ids():
            self.servos.set_moving_speed(sid, 100)
            time.sleep(0.003)
        self.stand()
        time.sleep(duration)
        for sid in self._all_servo_ids():
            self.servos.set_moving_speed(sid, 0)  # Back to max for walking
            time.sleep(0.003)

    def _init_auto_log(self):
        """Prepare auto-logging with timestamped filename."""
        ts = time.strftime("%Y%m%d_%H%M%S")
        self._auto_log_file = os.path.expanduser(f"~/gait_log_{ts}.csv")
        self._auto_log.clear()
        self._auto_log_start = time.monotonic()
        print(f"  Auto-logging to {self._auto_log_file}")

    def save_auto_log(self):
        """Write auto-log buffer to CSV and print summary."""
        if not self._auto_log:
            print("  No gait data recorded.")
            return
        if not self._auto_log_file:
            ts = time.strftime("%Y%m%d_%H%M%S")
            self._auto_log_file = os.path.expanduser(f"~/gait_log_{ts}.csv")

        fieldnames = list(self._auto_log[0].keys())
        with open(self._auto_log_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._auto_log)

        n = len(self._auto_log)
        print(f"\n  Saved {n} samples to {self._auto_log_file}")

        # Compact summary
        print(f"\n  {'Leg':3s} {'fem_range':>10s} {'tib_range':>10s} {'foot_x':>12s} {'err_f':>6s} {'err_t':>6s}")
        print(f"  {'-'*50}")
        for p in ['FL', 'FR', 'RL', 'RR']:
            fems = [r[f'{p}f'] for r in self._auto_log if f'{p}f' in r]
            tibs = [r[f'{p}t'] for r in self._auto_log if f'{p}t' in r]
            fxs = [r[f'{p}fx'] for r in self._auto_log if f'{p}fx' in r]
            fe = [abs(r[f'{p}fr'] - r[f'{p}fa']) for r in self._auto_log
                  if r.get(f'{p}fa', -1) != -1]
            te = [abs(r[f'{p}tr'] - r[f'{p}ta']) for r in self._auto_log
                  if r.get(f'{p}ta', -1) != -1]
            if fems:
                avg_fe = f"{sum(fe)/len(fe):.0f}" if fe else "-"
                avg_te = f"{sum(te)/len(te):.0f}" if te else "-"
                print(f"  {p:3s} {min(fems):+.0f}?{max(fems):+.0f}°"
                      f"    {min(tibs):+.0f}?{max(tibs):+.0f}°"
                      f"    {min(fxs):+.0f}?{max(fxs):+.0f}mm"
                      f"  {avg_fe:>5s} {avg_te:>5s}")
        print()

    def start(self):
        if self._running:
            return
        self._running = True
        self._phase = 0.0
        self._log_tick = 0
        self._init_auto_log()
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
                target_speed = self._speed
                target_turn = self._turn
                alpha = self._ramp_alpha
                phase_offsets = dict(self.gait_phases)

            # Exponential smoothing for jerk-free starts/stops
            self._ramped_speed += alpha * (target_speed - self._ramped_speed)
            self._ramped_turn += alpha * (target_turn - self._ramped_turn)

            # Snap to zero when close (no micro-creep)
            speed = self._ramped_speed if abs(self._ramped_speed) > 0.005 else 0.0
            turn = self._ramped_turn if abs(self._ramped_turn) > 0.005 else 0.0

            is_moving = abs(speed) > 0.01 or abs(turn) > 0.01
            positions = {}
            debug_angles = {}
            leg_data = {}  # Per-leg data for auto-logging

            for leg_id in LegID:
                leg_phase = (self._phase + phase_offsets[leg_id]) % 1.0

                if abs(turn) > 0.01 and abs(speed) < 0.01:
                    # Pure turning in place
                    hip = self.hip_positions[leg_id]
                    dx, dy, dz = self.trajectory.compute_turn(
                        leg_phase, turn, hip[0], hip[1])
                elif abs(turn) > 0.01:
                    # Combined forward + turning: blend at 70% turn authority
                    dx_fwd, _, dz_fwd = self.trajectory.compute(leg_phase, speed)
                    hip = self.hip_positions[leg_id]
                    dx_trn, dy_trn, dz_trn = self.trajectory.compute_turn(
                        leg_phase, turn * 0.7, hip[0], hip[1])
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
                    # Override coxa with locked standing angle to prevent drift
                    angles = (self.locked_coxa_angles[leg_id], angles[1], angles[2])
                except Exception as e:
                    print(f"IK fail {leg_id.value}: {e}")
                    continue

                for i, cfg in enumerate(LEG_SERVO_CONFIG[leg_id]):
                    raw = angle_to_raw(angles[i], cfg)
                    positions[cfg.servo_id] = raw
                    debug_angles[cfg.servo_id] = angles[i]

                duty = self.gait_cfg.duty_factor
                leg_data[leg_id] = {
                    'phase': leg_phase,
                    'state': 'stance' if leg_phase < duty else 'swing',
                    'dx': dx, 'dy': dy, 'dz': dz,
                    'foot_x': foot[0], 'foot_y': foot[1], 'foot_z': foot[2],
                    'angles': angles,
                }

            self.servos.sync_write_positions(positions)

            # Auto-record every Nth tick (compact format for gait tuning)
            if is_moving and self._auto_log_start > 0:
                self._log_tick += 1
                if self._log_tick >= self._log_divisor:
                    self._log_tick = 0
                    r = {
                        't': round(time.monotonic() - self._auto_log_start, 2),
                        'ph': round(self._phase, 3),
                        'spd': round(speed, 2),
                        'trn': round(turn, 2),
                    }
                    # Read actual servos on every 5th logged sample
                    do_read = (len(self._auto_log) % 5 == 0)
                    for leg_id in LegID:
                        p = leg_id.name  # FL/FR/RL/RR
                        ld = leg_data.get(leg_id)
                        if not ld:
                            continue
                        sw = 1 if ld['state'] == 'swing' else 0
                        r[f'{p}s'] = sw
                        r[f'{p}dx'] = round(ld['dx'], 1)
                        r[f'{p}dz'] = round(ld['dz'], 1)
                        r[f'{p}fx'] = round(ld['foot_x'], 1)
                        r[f'{p}fz'] = round(ld['foot_z'], 1)
                        # Femur + tibia angles and raw (skip shoulder - locked)
                        r[f'{p}f'] = round(ld['angles'][1], 1)
                        r[f'{p}t'] = round(ld['angles'][2], 1)
                        r[f'{p}fr'] = positions.get(LEG_SERVO_CONFIG[leg_id][1].servo_id, -1)
                        r[f'{p}tr'] = positions.get(LEG_SERVO_CONFIG[leg_id][2].servo_id, -1)
                        if do_read:
                            for i, jn in [(1, 'fa'), (2, 'ta')]:
                                cfg = LEG_SERVO_CONFIG[leg_id][i]
                                actual, _ = self.servos.read_position(cfg.servo_id)
                                r[f'{p}{jn}'] = actual if (actual is not None and 0 <= actual <= 1023) else -1
                        else:
                            r[f'{p}fa'] = -1
                            r[f'{p}ta'] = -1
                    self._auto_log.append(r)

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
    print("  --- WASD (incremental) ---")
    print("  w/s     = increase / decrease speed")
    print("  a/d     = steer left / right")
    print("  x/SPACE = full stop")
    print("  --- Arrow Keys (wireless KB) ---")
    print("  UP      = forward (full speed)")
    print("  DOWN    = backward (full speed)")
    print("  LEFT    = forward + gradual left")
    print("  RIGHT   = forward + gradual right")
    print("  --- Gaits ---")
    print("  1       = trot gait (diagonal pairs, quick)")
    print("  2       = walk gait (4-beat, stable)")
    print("  3       = creep gait (slow, ultra-stable)")
    print("  --- Diagnostics ---")
    print("  i       = identify servos (flash LEDs)")
    print("  t       = test RR shoulder servo")
    print("  r       = deep diagnose RR shoulder")
    print("  c       = dump servo state to CSV")
    print("  q       = quit (auto-saves gait log)")
    print("=" * 50 + "\n")

    speed = 0.0
    turn = 0.0
    speed_inc = 0.2
    arrow_turn_inc = 0.2    # Per-press turn increment for arrow keys
    arrow_turn_max = 0.8    # Max arrow turn (strong enough to actually steer)

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

                    # --- Arrow key escape sequences (ESC [ A/B/C/D) ---
                    if key == '\x1b':
                        if select.select([sys.stdin], [], [], 0.05)[0]:
                            key2 = sys.stdin.read(1)
                            if key2 == '[':
                                if select.select([sys.stdin], [], [], 0.05)[0]:
                                    key3 = sys.stdin.read(1)
                                    if key3 == 'A':    # UP -> forward
                                        speed = 1.0
                                        turn = 0.0
                                    elif key3 == 'B':  # DOWN -> backward
                                        speed = -1.0
                                        turn = 0.0
                                    elif key3 == 'D':  # LEFT -> fwd + gradual left
                                        speed = 1.0
                                        turn = max(-arrow_turn_max, turn - arrow_turn_inc)
                                    elif key3 == 'C':  # RIGHT -> fwd + gradual right
                                        speed = 1.0
                                        turn = min(arrow_turn_max, turn + arrow_turn_inc)
                        else:
                            # Bare ESC = stop all
                            speed = 0.0
                            turn = 0.0

                    # --- WASD keys ---
                    elif key == 'q':
                        break
                    elif key == 'w':
                        speed = min(1.0, speed + speed_inc)
                    elif key == 's':
                        speed = max(-1.0, speed - speed_inc)
                    elif key == 'a':
                        turn = max(-1.0, turn - 0.3)
                    elif key == 'd':
                        turn = min(1.0, turn + 0.3)
                    elif key in (' ', 'x'):
                        speed = 0.0
                        turn = 0.0
                    elif key == '1':
                        walker.set_gait(GaitType.TROT)
                    elif key == '2':
                        walker.set_gait(GaitType.WALK)
                    elif key == '3':
                        walker.set_gait(GaitType.CREEP)
                    elif key == 'i':
                        print("\n  Identifying servos...")
                        walker.identify_all_servos()
                        print("  Done!\n")
                    elif key == 't':
                        print("\n  Testing RR Shoulder (ID 2) - sweeping in/out...")
                        walker.test_rr_shoulder()
                        print("  Done!\n")
                    elif key == 'r':
                        print("\n  Deep diagnosis on RR Shoulder (ID 2)...")
                        walker.diagnose_servo(2)
                        print("  Done!\n")
                    elif key == 'c':
                        walker.dump_servo_csv()
                        print("  Done!\n")

                    walker.set_speed(speed)
                    walker.set_turn(turn)
                    log_n = len(walker._auto_log)
                    turn_dir = " <L" if turn < -0.01 else (" R>" if turn > 0.01 else "   ")
                    sys.stdout.write(f"\r  Speed: {speed:+.1f}  Turn: {turn:+.2f}{turn_dir}  "
                                     f"Gait: {walker.gait_type.value}  LOG:{log_n}       ")
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
                elif cmd == '3':
                    walker.set_gait(GaitType.CREEP)
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
            print(f"    {name} ID{cfg.servo_id:2d}: {angles[i]:+7.1f}deg -> raw={raw:4d}"
                  f"  (offset={cfg.offset_deg:+.1f}deg{inv})")
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
        print("\nStanding. Keys: c=CSV, l=log walk cycle, i=identify, t=test RR, r=diagnose RR, q=quit")
        try:
            import tty, termios, select
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            try:
                while True:
                    if select.select([sys.stdin], [], [], 0.2)[0]:
                        key = sys.stdin.read(1)
                        if key == 'q':
                            break
                        elif key == 'c':
                            print("\n  Dumping servo state...")
                            walker.dump_servo_csv()
                        elif key == 'l':
                            print("\n  Logging walk cycle...")
                            walker.log_walk_cycle()
                        elif key == 'i':
                            print("\n  Identifying servos...")
                            walker.identify_all_servos()
                            print("  Done!")
                        elif key == 't':
                            print("\n  Testing RR Shoulder...")
                            walker.test_rr_shoulder()
                            print("  Done!")
                        elif key == 'r':
                            print("\n  Diagnosing servo 2...")
                            walker.diagnose_servo(2)
                            print("  Done!")
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        except (ImportError, termios.error):
            # Fallback if no terminal
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
        print("\nRoboKitty shutting down. Bye!")


if __name__ == "__main__":
    main()