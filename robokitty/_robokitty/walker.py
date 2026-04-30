"""
QuadrupedWalker Controller Module.

Orchestrates multi-joint kinematics across a 12-DOF robot frame. Handles
system lifecycle, boot initialization, custom error tracking, and graceful shutdown.
"""

import time

from .ax12 import AX12Interface

from .. import _log as logger


class QuadrupedHardwareError(Exception):
    """
    Exception raised for fatal hardware abnormalities in the Quadruped framework.

    Automatically logs a CRITICAL error message to the logger pipeline
    upon instantiating the exception context.
    """

    def __init__(self, message: str):
        super().__init__(message)
        logger.critical("Hardware Exception Triggered: %s", message)


class QuadrupedWalker:
    """High-level interface for a 12-DOF AX-12A quadruped robot framework."""

    #  Connection & Addressing Defaults
    DEFAULT_PORT = "/dev/ttyUSB0"
    DEFAULT_BAUD = 1_000_000
    EXPECTED_SERVO_IDS = list(range(1, 13))

    # Compliance Margin: Safe dead-zone allowance (1 step)
    COMPLIANCE_MARGIN = 1
    # Compliance Slope: Moderate flexibility under strain to cushion plastic gears
    COMPLIANCE_SLOPE = 64
    # Initial Boot Speed: Safe, slowed velocity limit for startup calibration
    BOOT_SPEED_LIMIT = 400
    # Maximum Holding Torque: Fully engage holding power lines
    MAX_TORQUE_LIMIT = 1023

    # Neutral Stance Position: 512 represents the exact 150° mid-point of the AX-12A
    NEUTRAL_STANCE_POS = 512
    # Transit Speed: Speed used when morphing to the neutral standing posture
    POSTURE_TRANSIT_SPEED = 200
    # Loop Refresh Delay: 0.05 seconds yields a stable 20 Hz gait engine cycle
    GAIT_LOOP_DELAY_SEC = 0.05

    def __init__(self, port: str = DEFAULT_PORT, baudrate: int = DEFAULT_BAUD):
        self._ax12 = AX12Interface(port, baudrate)
        self._is_ready = False

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_exc):
        self.disconnect()

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
                f"Verify logic power distribution lines and serial cables."
            )

        logger.info("All 12 expected servos present. Initializing registers...")
        for servo_id in self.EXPECTED_SERVO_IDS:
            try:
                logger.debug(
                    "Configuring operational profiles on Joint ID: %d", servo_id
                )
                self._ax12.set_led(servo_id, True)
                self._ax12.enable_joint_mode(servo_id)
                self._ax12.set_compliance(
                    servo_id, margin=self.COMPLIANCE_MARGIN, slope=self.COMPLIANCE_SLOPE
                )
                self._ax12.set_moving_speed(servo_id, speed=self.BOOT_SPEED_LIMIT)
                self._ax12.set_torque_limit(servo_id, torque=self.MAX_TORQUE_LIMIT)
                self._ax12.torque_enable(servo_id, enable=True)
            except (TimeoutError, ValueError) as err:
                raise QuadrupedHardwareError(
                    f"Register payload delivery timed out or failed on Servo ID {servo_id}: {err}"
                )

        logger.info("Quadruped framework successfully online and torqued.")
        self._is_ready = True

    def disconnect(self) -> None:
        """Safely deactivates torque and drops visual telemetry across the frame."""
        logger.info("Executing safe hardware shutdown sequence...")
        self._is_ready = False

        for servo_id in self.EXPECTED_SERVO_IDS:
            try:
                logger.debug("Disabling Servo %d (LED and Torque)", servo_id)
                self._ax12.set_led(servo_id, False)
                self._ax12.torque_enable(servo_id, enable=False)
            except (TimeoutError, ValueError) as err:
                logger.warning(
                    "Could not cleanly communicate shutdown to Servo %d: %s",
                    servo_id,
                    err,
                )

        self._ax12.close()
        logger.info("Serial transport engine closed down cleanly.")

    def loop(self) -> None:
        """Main operational execution loop."""
        if not self._is_ready:
            self.connect()

        # Map out our clean, constant-driven home standing posture layout matrix
        stand_pose = {
            servo_id: self.NEUTRAL_STANCE_POS for servo_id in self.EXPECTED_SERVO_IDS
        }

        logger.info("Entering operational kinematic loop...")
        try:
            while self._is_ready:
                logger.debug("Broadcasting Sync Write target updates to all limbs.")
                self._ax12.sync_move(stand_pose, speed=self.POSTURE_TRANSIT_SPEED)

                # Gait calculations go here

                time.sleep(self.GAIT_LOOP_DELAY_SEC)

        except KeyboardInterrupt:
            logger.info("Control loop interrupted by operator via keyboard command.")
        finally:
            self.disconnect()
