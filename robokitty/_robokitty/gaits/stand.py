import math
from .base import Gait
from ..ax12 import degrees_to_position, clamp_position, to_le_bytes

COXA_NEUTRAL_DEG: float = 150.0
FEMUR_NEUTRAL_DEG: float = 120.0
TIBIA_NEUTRAL_DEG: float = 60.0
RIGHT_COXA_MIRROR_AXIS_DEG: float = 300.0


class Stand(Gait, alias="stand"):
    """
    Active standing/idle gait. Provides the baseline neutral posture and
    an optional subtle breathing oscillation.
    """

    def __init__(
        self,
        period_ticks: int = 40,
        idle_breathe: bool = False,
    ):
        super().__init__(period_ticks)
        self.idle_breathe = idle_breathe

    def get_pose(
        self, tick: int, leg_map: dict[str, tuple[int, int, int]]
    ) -> dict[int, list[int]]:
        femur_offset = 0.0
        tibia_offset = 0.0

        if self.idle_breathe:
            phase = (tick % self.period_ticks) / self.period_ticks
            oscillation = math.sin(2 * math.pi * phase)
            breathe_amplitude = 3.0
            femur_offset = oscillation * breathe_amplitude
            tibia_offset = -oscillation * (breathe_amplitude * 0.5)

        stand_pose = {}
        for leg_name, joint_ids in leg_map.items():
            c_deg = COXA_NEUTRAL_DEG
            f_deg = FEMUR_NEUTRAL_DEG + femur_offset
            t_deg = TIBIA_NEUTRAL_DEG + tibia_offset

            if "R" in leg_name:
                c_deg = RIGHT_COXA_MIRROR_AXIS_DEG - c_deg

            stand_pose[joint_ids[0]] = to_le_bytes(
                clamp_position(degrees_to_position(c_deg))
            )
            stand_pose[joint_ids[1]] = to_le_bytes(
                clamp_position(degrees_to_position(f_deg))
            )
            stand_pose[joint_ids[2]] = to_le_bytes(
                clamp_position(degrees_to_position(t_deg))
            )

        return stand_pose
