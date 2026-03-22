import threading
import time

from .ax12 import AX12Interface
from .config import LegDimensions, BodyDimensions, GaitConfig
from .constants import (
    DEFAULT_PORT,
    DEFAULT_BAUD,
    LegID,
    LEG_SERVO_CONFIG,
    AX12_DEG_TO_UNITS,
    AX12_CENTER,
    GAIT_PHASES,
    GaitType,
)
from .ik import FootTrajectory, LegIK
from .utils import angle_to_raw
from .. import _log


class QuadrupedWalker:
    """Top-level walking controller for RoboKitty."""

    def __init__(
        self,
        leg_dims: LegDimensions = None,
        body_dims: BodyDimensions = None,
        gait_config: GaitConfig = None,
        port: str = DEFAULT_PORT,
        baudrate: int = DEFAULT_BAUD,
        direction_pin: int | None = None,
    ):
        self.leg_dims = leg_dims or LegDimensions()
        self.body_dims = body_dims or BodyDimensions()
        self.gait_cfg = gait_config or GaitConfig()

        self.ik = LegIK(self.leg_dims)
        self.trajectory = FootTrajectory(self.gait_cfg)

        self.gait_type = GaitType.WALK
        self.gait_phases = dict(GAIT_PHASES[self.gait_type])

        self.servos = AX12Interface(port, baudrate, direction_pin)

        # Hip positions relative to body center
        bL = self.body_dims.half_length
        bW = self.body_dims.half_width
        self.hip_positions = {
            LegID.FL: (bL, bW),
            LegID.FR: (bL, -bW),
            LegID.RL: (-bL, bW),
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
        # L1 = self.leg_dims.coxa_length
        foot_lateral = 5.0  # Nearly vertical legs
        h = self.gait_cfg.body_height

        self.neutral_feet = {
            LegID.FL: (0.0, foot_lateral, -h),
            LegID.FR: (0.0, -foot_lateral, -h),
            LegID.RL: (0.0, foot_lateral, -h),
            LegID.RR: (0.0, -foot_lateral, -h),
        }

        # Lock standing coxa angle per leg
        self.locked_coxa_angles = {}
        for leg_id in LegID:
            foot = self.neutral_feet[leg_id]
            angles = self.ik.solve(*foot, is_front=self.leg_is_front[leg_id])
            self.locked_coxa_angles[leg_id] = angles[0]
        self._running = False
        self._walk_thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._speed = 0.0
        self._turn = 0.0
        self._phase = 0.0
        self.on_update = None  # Optional telemetry callback

        # Audit log: stores servo angles every N cycles during walking
        self._audit_log: list[str] = []
        self._audit_cycle = 0
        self._audit_interval = 10  # Log every 10th cycle
        # Build compact column header: FL_s,FL_f,FL_l,FR_s,...
        self._audit_header = "cyc,ph,spd," + ",".join(
            f"{lid.name}_{jn}" for lid in LegID for jn in ["s", "f", "l"]
        )

    def connect(self) -> bool:
        if not self.servos.connect():
            return False
        all_ids = self._all_servo_ids()
        shoulder_ids = [LEG_SERVO_CONFIG[lid][0].servo_id for lid in LegID]
        for sid in all_ids:
            self.servos.enable_torque(sid, True)
            # Zero compliance margin: eliminates dead zone so servos
            # apply force even for sub-unit position changes.
            # Slope 32 = moderate stiffness (good balance of hold vs compliance)
            self.servos.set_compliance(sid, margin=0, slope=32)
            if sid in shoulder_ids:
                self.servos.set_moving_speed(sid, 300)
            else:
                self.servos.set_moving_speed(sid, 0)
            time.sleep(0.003)
        _log.info(
            f"All {len(all_ids)} servos enabled (compliance=0, shoulders=300, legs=max)"
        )
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
            _log.info(f"Audit log saved: {fname} ({len(self._audit_log)} rows)")
        except Exception as e:
            _log.error(f"Failed to save audit log: {e}")

    def _all_servo_ids(self) -> list[int]:
        ids = []
        for leg_id in LegID:
            for cfg in LEG_SERVO_CONFIG[leg_id]:
                ids.append(cfg.servo_id)
        return ids

    def diagnose_servo(self, servo_id: int):
        """Full diagnostic for a servo - reads error register, voltage, temp."""
        _log.info(f"\n  === Diagnosing Servo ID {servo_id} ===")

        # Step 1: Read error register
        _log.info("  1. Reading error register...")
        err = self.servos.read_error(servo_id)
        if err is None:
            _log.error(
                "     FAILED to read - servo not responding on bus!"
                "     Check: wiring, power, servo ID, baud rate"
            )
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
                _log.info("     Error register: 0x00 (no errors)")
            else:
                _log.info(f"     Error register: 0x{err:02X}")
                for bit, name in error_names:
                    if err & (1 << bit):
                        _log.info(f"     ** {name} Error (bit {bit}) **")

        # Step 2: Read voltage
        _log.info("  2. Reading voltage...")
        volts = self.servos.read_voltage(servo_id)
        if volts is not None:
            status = "OK" if 9.0 <= volts <= 12.6 else "WARNING"
            _log.info(f"     Voltage: {volts:.1f}V ({status})")
        else:
            _log.error("     Failed to read voltage")

        # Step 3: Read temperature
        _log.info("  3. Reading temperature...")
        temp = self.servos.read_temperature(servo_id)
        if temp is not None:
            status = "OK" if temp < 65 else "HOT!" if temp < 75 else "CRITICAL!"
            _log.info(f"     Temperature: {temp}C ({status})")
        else:
            _log.error("     Failed to read temperature")

        # Step 4: Read current position
        _log.info("  4. Reading position...")
        pos = self.servos.read_position(servo_id)
        if pos is not None:
            _log.info(f"     Position: {pos} (center=512)")
        else:
            _log.error("     Failed to read position")

        # Step 5: Attempt recovery
        _log.info("  5. Attempting error clear (torque cycle + LED off)...")
        self.servos.clear_error(servo_id)
        time.sleep(0.5)

        # Re-read error
        err2 = self.servos.read_error(servo_id)
        if err2 is not None and err2 == 0:
            _log.info("     Error cleared successfully!")
        elif err2 is not None:
            _log.error(
                f"     Error persists: 0x{err2:02X}"
                "Try: power cycle, check for mechanical bind, reduce load"
            )
        else:
            _log.error("     Still can't communicate with servo")

        # Step 6: Try position command
        _log.info("  6. Setting joint mode and testing movement...")
        self.servos._send_packet(servo_id, 0x03, bytes([6, 0x00, 0x00]))
        time.sleep(0.01)
        self.servos._send_packet(servo_id, 0x03, bytes([8, 0xFF, 0x03]))
        time.sleep(0.01)
        self.servos.enable_torque(servo_id, True)
        self.servos.set_moving_speed(servo_id, 200)
        time.sleep(0.1)

        _log.info("     Moving to center (512)...")
        self.servos.sync_write_positions({servo_id: 512})
        time.sleep(1.5)
        _log.info("     Moving to 350...")
        self.servos.sync_write_positions({servo_id: 350})
        time.sleep(1.5)
        _log.info("     Moving to 650...")
        self.servos.sync_write_positions({servo_id: 650})
        time.sleep(1.5)

        # Return to standing position (not center!)
        _log.info("  7. Returning to standing pose...")
        self.stand()
        time.sleep(1.0)

        _log.info(f"\n  === Diagnosis complete for servo ID {servo_id} ===")

    def scan_all_errors(self):
        """Read error register, voltage, and temperature from every servo."""
        error_names = [
            (0, "Voltage"),
            (1, "AngleLimit"),
            (2, "Overheat"),
            (3, "Range"),
            (4, "Checksum"),
            (5, "Overload"),
            (6, "Instruction"),
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
                _log.info(
                    f"    {leg_id.value:12s} {joint_names[i]:8s} ID{sid:2d}: "
                    f"err={e_str} {v_str} {t_str} {p_str}{status}"
                )

        if not any_error:
            _log.info("    All servos healthy!")

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
            _log.info(f"    RR Shoulder -> {label} (raw={pos})")
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
                _log.info(f"  Flashing: {leg_id.value} {name} (ID {cfg.servo_id})")
                self.servos.identify_servo(cfg.servo_id, flashes=10)
                time.sleep(0.5)

    def read_all_positions(self):
        """
        Read current position of all 12 servos and display as angles.

        Use this with torque OFF to manually pose the robot then capture
        the exact servo positions as a calibration reference.
        """
        _border = "=" * 62
        _log.info(
            f"\n{_border}\n"
            "  READING CURRENT SERVO POSITIONS\n"
            "  (Torque should be OFF - manually pose legs first)\n"
            f"{_border}"
        )

        joint_names = ["Shoulder", "Femur   ", "Leg     "]
        all_raw = {}
        all_angles = {}

        for leg_id in LegID:
            configs = LEG_SERVO_CONFIG[leg_id]
            is_front = self.leg_is_front[leg_id]
            _log.info(f"\n  {leg_id.value} ({'FRONT' if is_front else 'REAR'}):")

            for _, (cfg, name) in enumerate(zip(configs, joint_names)):
                raw = self.servos.read_position(cfg.servo_id)
                if raw is None:
                    _log.error(f"    {name} ID{cfg.servo_id:2d}:  ** READ FAILED **")
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
                _log.info(
                    f"    {name} ID{cfg.servo_id:2d}:  raw={raw:4d}  "
                    f"angle={angle:+7.1f}deg{inv_str}"
                )

        # Summary table for easy copy-paste
        _log.info(
            f"\n{_border}\n"
            "  RAW POSITION SUMMARY (for copy-paste into config):\n"
            f"{_border}"
        )
        for leg_id in LegID:
            configs = LEG_SERVO_CONFIG[leg_id]
            vals = []
            for cfg in configs:
                r = all_raw.get(cfg.servo_id)
                vals.append(f"{r:4d}" if r is not None else " ???")
            _log.info(
                f"  {leg_id.value:12s}:  Shoulder={vals[0]}  "
                f"Femur={vals[1]}  Leg={vals[2]}"
            )

        _log.info(f"\n  ANGLE SUMMARY: \n{_border}")
        for leg_id in LegID:
            configs = LEG_SERVO_CONFIG[leg_id]
            vals = []
            for cfg in configs:
                a = all_angles.get(cfg.servo_id)
                vals.append(f"{a:+7.1f}" if a is not None else "   ???")
            _log.info(
                f"  {leg_id.value:12s}:  Shoulder={vals[0]}  "
                f"Femur={vals[1]}  Leg={vals[2]}"
            )

    def calibrate_standing(self):
        """
        Read all servo positions from manual pose, compare with IK targets,
        and compute the offset_deg needed for each joint to match.

        After running, copy the suggested offsets into LEG_SERVO_CONFIG.
        """
        _border = "=" * 62
        _log.info(f"\n{_border}  READING POSED POSITIONS & COMPUTING OFFSETS{_border}")

        joint_names = ["Shoulder", "Femur   ", "Leg     "]
        suggested_offsets = {}

        for leg_id in LegID:
            configs = LEG_SERVO_CONFIG[leg_id]
            is_front = self.leg_is_front[leg_id]
            foot = self.neutral_feet[leg_id]
            ik_angles = self.ik.solve(*foot, is_front=is_front)

            _log.info(f"\n  {leg_id.value} ({'FRONT' if is_front else 'REAR'}):")

            for i, (cfg, name) in enumerate(zip(configs, joint_names)):
                raw = self.servos.read_position(cfg.servo_id)
                if raw is None:
                    _log.error(f"    {name} ID{cfg.servo_id:2d}:  ** READ FAILED **")
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
                    needed_offset = (
                        -((raw - AX12_CENTER) / AX12_DEG_TO_UNITS) - ik_angle
                    )
                else:
                    needed_offset = ((raw - AX12_CENTER) / AX12_DEG_TO_UNITS) - ik_angle

                current_offset = cfg.offset_deg
                inv_str = " INV" if cfg.inverted else ""
                change = needed_offset - current_offset

                _log.info(
                    f"    {name} ID{cfg.servo_id:2d}:  raw={raw:4d}  "
                    f"actual={actual_angle:+7.1f}  ik_target={ik_angle:+7.1f}  "
                    f"offset: {current_offset:+.1f} -> {needed_offset:+.1f} "
                    f"(change {change:+.1f}){inv_str}"
                )

                suggested_offsets[(leg_id, i)] = needed_offset

        # Print copy-paste config
        _log.info(
            f"\n{_border}\n"
            "  SUGGESTED LEG_SERVO_CONFIG (copy-paste into code):\n"
            f"{_border}"
        )
        for leg_id in LegID:
            configs = LEG_SERVO_CONFIG[leg_id]
            jnames = ["Shoulder", "Femur", "Leg"]
            _log.info(f"    LegID.{leg_id.name}: [")
            for i, (cfg, jn) in enumerate(zip(configs, jnames)):
                off = suggested_offsets.get((leg_id, i), cfg.offset_deg)
                off_rounded = round(off, 1)
                inv_str = "True " if cfg.inverted else "False"
                _log.info(
                    f"        ServoJointConfig(servo_id={cfg.servo_id:<2d}, "
                    f"offset_deg={off_rounded:+6.1f}, inverted={inv_str}),"
                    f"  # {jn}"
                )
            _log.info("    ],")

        _log.info()
        _log.info("  Copy the offsets above into LEG_SERVO_CONFIG in the code.")
        _log.info("  Then run --stand to verify the pose matches.")
        _log.info()

    def test_joints(self):
        """
        Test each joint one at a time.
        Moves each servo slightly from center so you can see direction.
        Press Enter after each to continue.
        """
        joint_names = ["Shoulder", "Femur", "Leg"]

        # First center ALL servos
        _log.info("\n  Centering all servos to 512 (neutral)...")
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

                _log.info(f"\n  {leg_id.value} {joint_names[i]} (ID {cfg.servo_id})")
                _log.info("    Moving POSITIVE 30 degrees from center...")

                # Move this one servo +30 degrees from center
                test_pos = 512 + int(30 * AX12_DEG_TO_UNITS)  # ~614
                self.servos.sync_write_positions({cfg.servo_id: test_pos})
                time.sleep(1.0)

                _log.info(f"    Raw position: 512 -> {test_pos}")
                _log.info(f"    Currently inverted: {cfg.inverted}")
                _log.info("    What did the servo do?")
                _log.info("      Shoulder: should swing FORWARD")
                _log.info("      Femur:    should swing leg DOWN/FORWARD")
                _log.info("      Leg:      should swing foot DOWN/FORWARD")

                input("    Press Enter for next joint...")

                # Return to center
                self.servos.sync_write_positions({cfg.servo_id: 512})
                time.sleep(0.5)

        _log.info("\n  Joint test complete. Report which ones went the wrong way.")
        """Flash each servo LED one at a time with label."""
        for leg_id in LegID:
            configs = LEG_SERVO_CONFIG[leg_id]
            joint_names = ["Shoulder", "Femur", "Leg"]
            for cfg, name in zip(configs, joint_names):
                _log.info(f"  Flashing: {leg_id.value} {name} (ID {cfg.servo_id})")
                self.servos.identify_servo(cfg.servo_id, flashes=10)
                time.sleep(0.5)

    def set_gait(self, gait: GaitType):
        with self._lock:
            self.gait_type = gait
            self.gait_phases = dict(GAIT_PHASES[gait])
            if gait == GaitType.WALK:
                self.gait_cfg.duty_factor = 0.75
                self.gait_cfg.cycle_time = 1.6
                self.gait_cfg.step_length = 80.0
            else:
                self.gait_cfg.duty_factor = 0.5
                self.gait_cfg.cycle_time = 1.2
                self.gait_cfg.step_length = 60.0
        _log.info(f"Gait: {gait.value}")

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
            angles = self.ik.solve(*foot, is_front=self.leg_is_front[leg_id])
            for i, cfg in enumerate(LEG_SERVO_CONFIG[leg_id]):
                positions[cfg.servo_id] = angle_to_raw(angles[i], cfg)
        self.servos.sync_write_positions(positions)
        _log.info("Standing")

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
        _log.info("Walking controller started")

    def stop(self):
        if not self._running:
            return
        self._running = False
        if self._walk_thread:
            self._walk_thread.join(timeout=2.0)
            self._walk_thread = None
        self.smooth_stand(0.8)
        _log.info("Walking controller stopped")

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

            # Compute body shift for weight transfer (walk/creep gait)
            body_dx, body_dy = 0.0, 0.0
            if is_moving and abs(speed) > 0.01:
                body_dx, body_dy = self.trajectory.compute_body_shift(
                    self._phase, speed, phase_offsets, self.hip_positions
                )

            for leg_id in LegID:
                leg_phase = (self._phase + phase_offsets[leg_id]) % 1.0

                if abs(turn) > 0.01 and abs(speed) < 0.01:
                    # Pure turning
                    hip = self.hip_positions[leg_id]
                    dx, dy, dz = self.trajectory.compute_turn(
                        leg_phase, turn, hip[0], hip[1]
                    )
                elif abs(turn) > 0.01:
                    # Combined forward + turning
                    dx_fwd, _, dz_fwd = self.trajectory.compute(leg_phase, speed)
                    hip = self.hip_positions[leg_id]
                    dx_trn, dy_trn, dz_trn = self.trajectory.compute_turn(
                        leg_phase, turn * 0.5, hip[0], hip[1]
                    )
                    dx = dx_fwd + dx_trn
                    dy = dy_trn
                    dz = max(dz_fwd, dz_trn)
                else:
                    # Straight walking
                    dx, dy, dz = self.trajectory.compute(leg_phase, speed)

                if not is_moving:
                    dx, dy, dz = 0.0, 0.0, 0.0

                # Add body shift for weight transfer
                dx += body_dx
                dy += body_dy

                nx, ny, nz = self.neutral_feet[leg_id]
                foot = (nx + dx, ny + dy, nz + dz)

                try:
                    angles = self.ik.solve(*foot, is_front=self.leg_is_front[leg_id])
                    # Override coxa with locked standing angle to prevent drift
                    angles = (self.locked_coxa_angles[leg_id], angles[1], angles[2])
                except Exception as e:
                    _log.error(f"IK fail {leg_id.value}: {e}")
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
                    vals = [
                        str(self._audit_cycle),
                        f"{self._phase:.2f}",
                        f"{speed:.1f}",
                    ]
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
