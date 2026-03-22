import math

from .config import LegDimensions, GaitConfig


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

    def solve(
        self, x: float, y: float, z: float, is_front: bool = True
    ) -> tuple[float, float, float]:
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


class FootTrajectory:
    """
    Generates foot paths optimised for actual ground walking.

    Swing has 4 sub-phases for clean foot placement:
      1. LIFT: raise foot vertically from current position (25% of swing)
      2. TRAVEL: move foot forward at full height (50% of swing)
      3. LOWER: bring foot straight down to ground (20% of swing)
      4. PLANT: brief pause at ground level before stance (5% of swing)

    Stance: linear backward slide at ground level (dz=0).
    No ground press - the lower body height (110mm) provides grip.
    """

    def __init__(self, cfg: GaitConfig):
        self.cfg = cfg

    def compute(self, phase: float, speed: float = 1.0) -> tuple[float, float, float]:
        """Get (dx, dy, dz) foot offset from neutral position."""
        if abs(speed) < 0.01:
            return (0.0, 0.0, 0.0)

        duty = self.cfg.duty_factor
        half_step = self.cfg.step_length * 0.5

        if phase < duty:
            # === STANCE: foot flat on ground, slides forward (+x) ===
            # Physical front is at -x (RL/RR side after front/rear swap).
            # Stance slides foot in +x so body moves in -x = physical forward.
            t = phase / duty
            dx = half_step * (-1.0 + 2.0 * t) * speed
            dz = 0.0
        else:
            # === SWING: 4 sub-phases ===
            t = (phase - duty) / (1.0 - duty)  # 0 to 1 within swing

            # Foot returns to -x (physical front) during swing
            if t < 0.25:
                # LIFT: foot stays near +x while rising
                st = t / 0.25
                x_progress = 0.1 * st
            elif t < 0.75:
                # TRAVEL: foot moves to -x at height
                st = (t - 0.25) / 0.5
                x_progress = 0.1 + 0.8 * st
            else:
                # LOWER + PLANT: foot at -x position, coming down
                st = (t - 0.75) / 0.25
                x_progress = 0.9 + 0.1 * st

            dx = half_step * (1.0 - 2.0 * x_progress) * speed

            # Foot Z (height) through swing
            if t < 0.25:
                # LIFT: smooth rise from ground to full height
                st = t / 0.25
                dz = self.cfg.step_height * (0.5 - 0.5 * math.cos(math.pi * st))
            elif t < 0.75:
                # TRAVEL: maintain full height
                dz = self.cfg.step_height
            elif t < 0.95:
                # LOWER: smooth descent to ground
                st = (t - 0.75) / 0.2
                dz = self.cfg.step_height * (0.5 + 0.5 * math.cos(math.pi * st))
            else:
                # PLANT: foot on ground, brief pause
                dz = 0.0

        return (dx, 0.0, dz)

    def compute_body_shift(
        self,
        phase: float,
        speed: float,
        phase_offsets: dict,
        hip_positions: dict,
    ) -> tuple[float, float]:
        """Compute body X/Y shift to keep CoM over support triangle."""
        if abs(speed) < 0.01:
            return (0.0, 0.0)

        duty = self.cfg.duty_factor

        # Find which leg is in swing and compute support centroid
        swing_weight = {}
        for leg_id, leg_offset in phase_offsets.items():
            leg_phase = (phase + leg_offset) % 1.0
            if leg_phase >= duty:
                t = (leg_phase - duty) / (1.0 - duty)
                swing_weight[leg_id] = math.sin(math.pi * t)
            else:
                swing_weight[leg_id] = 0.0

        total_support = 0.0
        cx, cy = 0.0, 0.0
        for leg_id, hip in hip_positions.items():
            support = 1.0 - swing_weight.get(leg_id, 0.0)
            cx += hip[0] * support
            cy += hip[1] * support
            total_support += support

        if total_support < 0.1:
            return (0.0, 0.0)

        cx /= total_support
        cy /= total_support

        shift_x = -cx * 0.12
        shift_y = -cy * 0.12

        return (shift_x, shift_y)

    def compute_turn(
        self, phase: float, turn_rate: float, hip_x: float, hip_y: float
    ) -> tuple[float, float, float]:
        """Foot offset for turning in place."""
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
