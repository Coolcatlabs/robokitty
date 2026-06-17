"""
Crawl Locomotion Gait Engine.
"""

import math
from .base import Gait
from ..ax12 import degrees_to_position, clamp_position

# ---------------------------------------------------------------------------
# Right-side coxa mirror axis (degrees).
# Chosen so that a neutral left coxa maps to a neutral right coxa across
# the robot's longitudinal symmetry plane.
# ---------------------------------------------------------------------------
RIGHT_COXA_MIRROR_AXIS_DEG: float = 300.0

# ---------------------------------------------------------------------------
# Gait timing constants
# ---------------------------------------------------------------------------

# Fraction of the leg cycle spent in swing (airborne).
# The remainder (1 - SWING_DUTY_CYCLE) is stance (ground contact).
SWING_DUTY_CYCLE: float = 0.25
STANCE_DUTY_CYCLE: float = 1.0 - SWING_DUTY_CYCLE  # 0.75

# Phase offsets per leg for a single-leg-at-a-time crawl sequence.
# Each leg is offset by one quarter-period so exactly one leg swings
# while the other three remain grounded.
LEG_PHASE_OFFSETS: dict[str, float] = {
    "FL": 0.00,
    "BR": 0.25,
    "FR": 0.50,
    "BL": 0.75,
}

# ---------------------------------------------------------------------------
# Swing kinematics constants
# ---------------------------------------------------------------------------

# Full sine period used to shape swing arc and lift profiles.
FULL_SINE_PERIOD: float = math.pi

# Coxa starts at +stride_amp at swing entry and arrives at -stride_amp at
# swing exit. A phase-shifted cosine achieves this:
#   offset = cos(t * π + π)  →  starts at cos(π) = -1 → *(-amp) = +amp
#                              →  ends   at cos(2π) = +1 → *(-amp) = -amp
COXA_SWING_PHASE_SHIFT: float = math.pi

# Femur lift uses a straight sine arch (0 → peak → 0) over the swing arc.
# No additional phase shift needed.
FEMUR_LIFT_PHASE_SHIFT: float = 0.0

# Tibia compensation is a partial lift opposite to femur, scaled down so it
# doesn't over-extend the foot path.
TIBIA_LIFT_SCALE: float = 0.5

# ---------------------------------------------------------------------------
# Stance kinematics constants
# ---------------------------------------------------------------------------

# The stance coxa sweeps from +stride_amp back to -stride_amp linearly.
# At stance_progress p ∈ [0, 1]:  coxa_offset = stride_amp * (1 - 2p)
# This matches the swing exit position and reaches the swing entry position
# exactly one period later, ensuring continuity.
STANCE_LINEAR_SCALE: float = 2.0  # coefficient in (1 - STANCE_LINEAR_SCALE * p)


class Crawl(Gait, alias="crawl"):
    """
    Stable 4-stage crawl gait. Moves one leg at a time while the remaining
    three maintain ground contact, maximising static stability margin.

    Leg cycle breakdown
    -------------------
    Swing (0 % – 25 %):
        Coxa  : cosine arc, +stride_amp → −stride_amp  (forward reach → rear)
        Femur : sine arch lifting the foot clear of the ground
        Tibia : partial counter-arc to keep the foot path shallow

    Stance (25 % – 100 %):
        Coxa  : linear sweep continuing from −stride_amp → +stride_amp
        Femur : held at neutral (no terrain compensation — flat ground only)
        Tibia : held at neutral

    Position continuity
    -------------------
    The coxa value at swing exit  equals the coxa value at stance entry, and
    the coxa value at stance exit equals the coxa value at the next swing
    entry, so there are no instantaneous position jumps.
    """

    def __init__(
        self,
        period_ticks: int = 40,
        stride_amp_deg: float = 20.0,
        step_lift_deg: float = 25.0,
    ) -> None:
        super().__init__(period_ticks)
        self.stride_amp_deg = stride_amp_deg
        self.step_lift_deg = step_lift_deg

        # Neutral (home) joint angles in degrees.
        self.COXA_NEUTRAL_DEG: float = 150.0
        self.FEMUR_NEUTRAL_DEG: float = 120.0
        self.TIBIA_NEUTRAL_DEG: float = 60.0

    def get_pose(
        self,
        tick: int,
        leg_map: dict[str, tuple[int, int, int]],
    ) -> dict[int, list[int]]:
        """
        Return a packet payload mapping servo IDs to little-endian position
        register pairs for the given tick.
        """
        packet_payload: dict[int, list[int]] = {}

        for leg_name, joint_ids in leg_map.items():
            phase_offset = LEG_PHASE_OFFSETS.get(leg_name)
            if phase_offset is None:
                raise ValueError(
                    f"Unknown leg name '{leg_name}'. "
                    f"Expected one of: {list(LEG_PHASE_OFFSETS)}"
                )

            normalized_phase = (tick / self.period_ticks + phase_offset) % 1.0
            coxa, femur, tibia = self._compute_joint_angles(normalized_phase)

            if "R" in leg_name:
                coxa = RIGHT_COXA_MIRROR_AXIS_DEG - coxa

            c_pos = clamp_position(degrees_to_position(coxa))
            f_pos = clamp_position(degrees_to_position(femur))
            t_pos = clamp_position(degrees_to_position(tibia))

            packet_payload[joint_ids[0]] = self._to_le_bytes(c_pos)
            packet_payload[joint_ids[1]] = self._to_le_bytes(f_pos)
            packet_payload[joint_ids[2]] = self._to_le_bytes(t_pos)

        return packet_payload

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _compute_joint_angles(
        self, normalized_phase: float
    ) -> tuple[float, float, float]:
        """
        Return (coxa, femur, tibia) angles in degrees for a normalised phase
        value in [0, 1).
        """
        coxa = self.COXA_NEUTRAL_DEG
        femur = self.FEMUR_NEUTRAL_DEG
        tibia = self.TIBIA_NEUTRAL_DEG

        if normalized_phase < SWING_DUTY_CYCLE:
            coxa, femur, tibia = self._swing_angles(
                normalized_phase, coxa, femur, tibia
            )
        else:
            coxa = self._stance_coxa(normalized_phase, coxa)

        return coxa, femur, tibia

    def _swing_angles(
        self,
        phase: float,
        coxa: float,
        femur: float,
        tibia: float,
    ) -> tuple[float, float, float]:
        """
        Compute joint angles during the swing phase.

        ``phase`` is the raw normalised phase (0 → SWING_DUTY_CYCLE).
        It is first mapped to [0, 1] within the swing window, then to a
        radian arc over [0, π].
        """
        swing_progress = phase / SWING_DUTY_CYCLE  # 0 → 1
        swing_radial = swing_progress * FULL_SINE_PERIOD  # 0 → π

        # Coxa: phase-shifted cosine so the offset goes +amp → −amp,
        # matching the stance exit position and eliminating discontinuities.
        coxa += -math.cos(swing_radial + COXA_SWING_PHASE_SHIFT) * self.stride_amp_deg

        # Femur: sine arch lifts the foot and returns it to neutral.
        femur -= math.sin(swing_radial + FEMUR_LIFT_PHASE_SHIFT) * self.step_lift_deg

        # Tibia: partial counter-arc keeps the foot path relatively shallow.
        tibia += (
            math.sin(swing_radial + FEMUR_LIFT_PHASE_SHIFT)
            * self.step_lift_deg
            * TIBIA_LIFT_SCALE
        )

        return coxa, femur, tibia

    def _stance_coxa(self, phase: float, coxa: float) -> float:
        """
        Compute the coxa angle during the stance phase.

        Linearly sweeps from +stride_amp (stance entry) back to −stride_amp
        (stance exit), matching the swing entry/exit positions exactly.
        """
        stance_progress = (phase - SWING_DUTY_CYCLE) / STANCE_DUTY_CYCLE  # 0 → 1
        coxa += self.stride_amp_deg * (1.0 - STANCE_LINEAR_SCALE * stance_progress)
        return coxa
