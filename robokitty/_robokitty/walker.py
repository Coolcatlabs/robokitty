"""
QuadrupedWalker Controller Module.

Orchestrates multi-joint kinematics across a 12-DOF robot frame. Handles
system lifecycle, boot initialization, custom error tracking, and graceful shutdown.
"""

import time
from typing import Optional

from .ax12 import AX12Interface
from .gaits import Gait, Stand
from .. import _log as logger


class QuadrupedHardwareError(Exception):
    """Raised for fatal hardware abnormalities in the Quadruped framework."""

    pass


class QuadrupedWalker:
    """High-level interface for a 12-DOF AX-12A quadruped robot framework."""

    EXPECTED_SERVO_IDS = list(range(1, 13))

    LEG_MAP = {
        "FL": (1, 2, 3),
        "FR": (4, 5, 6),
        "BL": (7, 8, 9),
        "BR": (10, 11, 12),
    }

    GAIT_LOOP_DELAY_SEC = 0.03
    BLEND_DURATION_TICKS = 15  # ~0.45 s @ 33 Hz

    def __init__(self, port: str, baudrate: int, gait: str) -> None:
        self._ax12 = AX12Interface(port, baudrate)

        self._stand = Stand()
        self._gait: Gait = Gait.create(gait)
        self._current_gait_alias = gait

        self._previous_gait: Optional[Gait] = None
        self._previous_gait_tick: int = 0
        self._blend_tick_counter: int = 0
        self._is_blending: bool = False

        self._is_ready: bool = False
        self._gait_tick: int = 0
        logger.info("Chassis mapped with behavior model layout: %s", gait)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_exc):
        self.disconnect()

    def transition_to(self, gait_alias: str) -> None:
        """
        Smoothly switch to a new gait via a LERP cross-fade.

        Note: not thread-safe. If calling from a separate thread (keyboard
        listener, ROS topic, etc.) while loop() is running, protect with a Lock.
        """
        if gait_alias == self._current_gait_alias:
            return

        logger.info(
            "Scheduling gait transition: %s -> %s",
            self._current_gait_alias,
            gait_alias,
        )

        self._previous_gait = self._gait
        self._previous_gait_tick = self._gait_tick

        self._gait = Gait.create(gait_alias)
        self._current_gait_alias = gait_alias

        self._gait_tick = 0
        self._blend_tick_counter = 0
        self._is_blending = True

    def connect(self) -> None:
        """Scan, register, and configure the 12-joint mechanical framework."""
        logger.info("Starting structural bus scan...")
        discovered_ids = self._ax12.scan(id_range=range(1, 254))

        missing_ids = [
            id_ for id_ in self.EXPECTED_SERVO_IDS if id_ not in discovered_ids
        ]
        if missing_ids:
            raise QuadrupedHardwareError(
                f"Bus scan incomplete. Missing structural limb segments: {missing_ids}. "
                "Verify logic power distribution lines and serial cables."
            )

        logger.info("All 12 expected servos present. Initializing registers...")
        for servo_id in self.EXPECTED_SERVO_IDS:
            try:
                logger.debug(
                    "Configuring operational profiles on Joint ID: %d", servo_id
                )
                self._ax12.set_led(servo_id, True)
                self._ax12.enable_joint_mode(servo_id)
                self._ax12.set_compliance(servo_id)
                self._ax12.set_torque_limit(servo_id)
                self._ax12.torque_enable(servo_id, enable=True)
            except (TimeoutError, ValueError) as err:
                raise QuadrupedHardwareError(
                    f"Register delivery failed on Servo ID {servo_id}: {err}"
                ) from err

        self._send_to_neutral_stand()

        logger.info("Quadruped framework online, standing, and torqued.")
        self._is_ready = True

    def disconnect(self) -> None:
        """Safely deactivates torque and drops visual telemetry across the frame."""
        logger.info("Executing safe hardware shutdown sequence...")
        self._is_ready = False

        for servo_id in self.EXPECTED_SERVO_IDS:
            try:
                logger.debug("Disabling Servo %d (LED and torque)", servo_id)
                self._ax12.set_led(servo_id, False)
                self._ax12.torque_enable(servo_id, enable=False)
            except (TimeoutError, ValueError) as err:
                logger.warning(
                    "Could not cleanly shut down Servo %d: %s", servo_id, err
                )

        self._ax12.close()
        logger.info("Serial transport closed cleanly.")

    def loop(self) -> None:
        """Main operational execution loop driving the dynamic gait patterns."""
        if not self._is_ready:
            raise RuntimeError(
                "QuadrupedWalker.connect() must be called before loop(). "
                "Use the context manager or call connect() explicitly."
            )

        logger.info("Entering kinematic loop with gait: %s", self._current_gait_alias)
        try:
            while self._is_ready:
                logger.debug("Gait tick: %d", self._gait_tick)

                current_pose = self._gait.get_pose(self._gait_tick, self.LEG_MAP)

                if self._is_blending and self._previous_gait is not None:
                    if self._blend_tick_counter >= self.BLEND_DURATION_TICKS:
                        # Blend window complete — drop straight into the new gait.
                        # current_pose is already 100% the new gait so no LERP needed.
                        logger.info(
                            "Transition to '%s' complete.", self._current_gait_alias
                        )
                        self._is_blending = False
                        self._previous_gait = None
                        packet_payload = current_pose
                    else:
                        alpha = self._blend_tick_counter / self.BLEND_DURATION_TICKS
                        old_pose = self._previous_gait.get_pose(
                            self._previous_gait_tick, self.LEG_MAP
                        )
                        packet_payload = self._blend_poses(
                            old_pose, current_pose, alpha
                        )
                        self._previous_gait_tick = (
                            self._previous_gait_tick + 1
                        ) % self._previous_gait.period_ticks
                        self._blend_tick_counter += 1
                else:
                    packet_payload = current_pose

                self._ax12.sync_move(packet_payload, speed=0)
                self._gait_tick = (self._gait_tick + 1) % self._gait.period_ticks
                time.sleep(self.GAIT_LOOP_DELAY_SEC)

        except KeyboardInterrupt:
            logger.info("Control loop interrupted by operator.")
        finally:
            self.disconnect()

    def _send_to_neutral_stand(self) -> None:
        """Drive all joints to the neutral standing posture (blocking)."""
        logger.info("Transitioning to standing posture...")
        pose = self._stand.get_pose(tick=0, leg_map=self.LEG_MAP)
        self._ax12.sync_move(pose, speed=0)
        time.sleep(0.5)

    def _blend_poses(
        self,
        old_pose: dict[int, list[int]],
        new_pose: dict[int, list[int]],
        alpha: float,
    ) -> dict[int, list[int]]:
        """LERP between two register-byte poses. alpha=0 → old, alpha=1 → new."""
        blended: dict[int, list[int]] = {}
        for servo_id in self.EXPECTED_SERVO_IDS:
            # Reconstruct 10-bit position from little-endian bytes
            old_val = old_pose[servo_id][0] + (old_pose[servo_id][1] << 8)
            new_val = new_pose[servo_id][0] + (new_pose[servo_id][1] << 8)
            v = int((1.0 - alpha) * old_val + alpha * new_val)
            blended[servo_id] = [v & 0xFF, (v >> 8) & 0xFF]
        return blended
