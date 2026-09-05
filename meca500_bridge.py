#!/usr/bin/env python3
"""
Meca500 VR Bridge — Teleoperate a real Meca500 from VR

Bridges the WebSocket-based VR viewer to a physical Meca500 robot
via the mecademicpy API. Joint angles from the VR viewer become
velocity targets for the real robot, with configurable speed scaling
and automatic safety stop on disconnect.

Architecture:
    VR Headset ←WebSocket→ server.py ←WebSocket→ meca500_bridge.py ←TCP→ Meca500

Usage:
    pip install mecademicpy websockets
    python meca500_bridge.py [--robot-ip 192.168.0.100] [--ws-url ws://localhost:8080/ws]

Gripper:
    An attached MEGP 25E/25LS electric gripper is detected automatically and
    driven from the VR right-hand trigger (analogue) or the viewer panel.
    Gripper moves are throttled before reaching the motion queue — see
    GRIPPER_DEADBAND_MM. Use --no-gripper to ignore the tool entirely.

Safety:
    - Velocity is clamped to --vel-scale fraction of max joint speed (default 25%)
    - SetVelTimeout auto-stops robot if no command arrives within 100 ms
    - Connection watchdog pauses robot if bridge hangs
    - Ctrl+C performs clean shutdown (deactivate + disconnect)
    - Teleoperation must be explicitly enabled (--auto-enable or via UI)
    - E-Stop and collision stop freeze the gripper in place rather than
      releasing, so a held part is not dropped on a fault
"""

import argparse
import asyncio
import json
import logging
import ssl
import sys
import threading
import time

try:
    import mecademicpy.robot as mdr
except ImportError:
    print("mecademicpy not installed. Run: pip install mecademicpy")
    sys.exit(1)

try:
    import websockets
except ImportError:
    print("websockets not installed. Run: pip install websockets")
    sys.exit(1)


logging.basicConfig(level=logging.INFO, format="%(asctime)s  %(message)s")
log = logging.getLogger("meca500-bridge")

# ── Meca500 R3 specifications (from user manual) ────────────────────────────

JOINT_LIMITS = [
    (-175, 175),       # J1 Base
    (-70,   90),       # J2 Shoulder
    (-135,  70),       # J3 Elbow
    (-170, 170),       # J4 Wrist Roll
    (-115, 115),       # J5 Wrist Pitch
    (-36000, 36000),   # J6 Flange (multi-turn)
]

MAX_JOINT_VEL = [150, 150, 180, 300, 300, 500]  # deg/s

# ── MEGP 25E electric gripper ───────────────────────────────────────────────

# Finger opening in mm. GetGripperRange() overrides this once the robot is
# activated and homed; these are the MEGP 25E defaults used until then.
GRIPPER_RANGE_MM = (0.0, 5.6)

# The VR trigger is analogue and streams at the control-loop rate, so gripper
# targets are rate-limited before reaching the motion queue: a MoveGripper per
# control tick would flood it. A move is only issued when the target has moved
# at least GRIPPER_DEADBAND_MM since the last one, and never more often than
# every GRIPPER_MIN_INTERVAL seconds. Explicit open/close bypasses both.
GRIPPER_DEADBAND_MM = 0.15
GRIPPER_MIN_INTERVAL = 0.10

# Tool type IDs reported by GetRtExtToolStatus().physical_tool_type
EXT_TOOL_NONE = 0
EXT_TOOL_MEGP25_SHORT = 10
EXT_TOOL_MEGP25_LONG = 11
GRIPPER_TOOL_TYPES = {EXT_TOOL_MEGP25_SHORT, EXT_TOOL_MEGP25_LONG}

TOOL_NAMES = {
    EXT_TOOL_NONE: "none",
    EXT_TOOL_MEGP25_SHORT: "MEGP 25E",
    EXT_TOOL_MEGP25_LONG: "MEGP 25LS",
    20: "VBOX 2-valve",
}


# ── ANSI colour helpers ─────────────────────────────────────────────────────

def _c(code, text):
    return f"\033[{code}m{text}\033[0m" if sys.stdout.isatty() else text

def _green(t):  return _c("32", t)
def _yellow(t): return _c("33", t)
def _red(t):    return _c("1;31", t)
def _cyan(t):   return _c("36", t)
def _dim(t):    return _c("2", t)


class Meca500Bridge:
    """Bidirectional bridge between VR viewer (WebSocket) and Meca500 (TCP)."""

    def __init__(self, robot_ip, ws_url, session=None,
                 vel_scale=0.25, gain=2.0, update_hz=50,
                 auto_enable=False, sim=False,
                 gripper_force=50, gripper_vel=50, use_gripper=True):
        self.robot_ip = robot_ip
        self.ws_url = ws_url
        self.session = session
        self.vel_scale = max(0.01, min(1.0, vel_scale))
        self.gain = gain
        self.update_hz = update_hz
        self.auto_enable = auto_enable
        self.sim = sim

        self.robot = None
        self.ws = None
        self.running = False
        self.enabled = False
        self.robot_connected = False
        self.robot_homed = False
        self.paused = False

        self.vr_target_joints = None
        self.real_joints = [0.0] * 6
        self.collision_stopped = False
        self._lock = threading.Lock()

        self.max_vel = [v * self.vel_scale for v in MAX_JOINT_VEL]

        # ── Gripper ─────────────────────────────────────────────────────
        self.use_gripper = use_gripper
        self.gripper_present = False
        self.gripper_homed = False
        self.gripper_error = False
        self.tool_type = EXT_TOOL_NONE
        self.gripper_min, self.gripper_max = GRIPPER_RANGE_MM
        self.gripper_force = max(5, min(100, gripper_force))
        self.gripper_vel = max(5, min(100, gripper_vel))
        # Target opening in mm. None until something commands the gripper, so
        # the bridge never actuates it just by connecting.
        self.gripper_target = None
        self.gripper_pos = self.gripper_max
        self.gripper_holding = False
        self._gripper_sent = None
        self._gripper_sent_t = 0.0
        self._gripper_force_send = False

    # ── WebSocket connection ────────────────────────────────────────────

    async def connect_ws(self):
        url = self.ws_url
        sep = "&" if "?" in url else "?"
        url += f"{sep}role=controller"
        if self.session:
            url += f"&session={self.session}"

        # Try the URL as given; if it fails on ws://, retry with wss://
        for attempt_url in self._ws_url_variants(url):
            ssl_ctx = None
            if attempt_url.startswith("wss://"):
                ssl_ctx = ssl.create_default_context()
                ssl_ctx.check_hostname = False
                ssl_ctx.verify_mode = ssl.CERT_NONE

            log.info(f"Connecting to VR viewer at {attempt_url}")
            try:
                self.ws = await websockets.connect(attempt_url, ssl=ssl_ctx)
                log.info(_green("WebSocket connected"))
                return
            except Exception as e:
                if attempt_url == url:
                    log.warning(f"Connection failed: {e}")
                else:
                    raise

    def _ws_url_variants(self, url):
        yield url
        if url.startswith("ws://"):
            yield url.replace("ws://", "wss://", 1)

    # ── Robot connection ────────────────────────────────────────────────

    def connect_robot(self):
        if self.sim:
            log.info(_yellow("SIMULATION MODE — no real robot"))
            self.robot_connected = True
            self.robot_homed = True
            if self.use_gripper:
                self.gripper_present = True
                self.gripper_homed = True
                self.tool_type = EXT_TOOL_MEGP25_SHORT
                log.info(_yellow(f"Simulated gripper: {TOOL_NAMES[self.tool_type]} "
                                 f"({self.gripper_min:.1f}-{self.gripper_max:.1f} mm)"))
            return

        log.info(f"Connecting to Meca500 at {self.robot_ip} ...")
        self.robot = mdr.Robot()
        self.robot.Connect(address=self.robot_ip, disconnect_on_exception=True)
        self.robot_connected = True
        log.info(_green("Robot connected"))

        log.info("Activating robot ...")
        self.robot.ActivateRobot()
        self.robot.WaitActivated()
        log.info(_green("Robot activated"))

        log.info("Homing robot ...")
        self.robot.Home()
        self.robot.WaitHomed()
        self.robot_homed = True
        log.info(_green("Robot homed"))

        self.robot.SetJointVel(self.vel_scale * 100)
        self.robot.SetJointAcc(50)
        self.robot.SetVelTimeout(0.1)
        self.robot.SetMonitoringInterval(0.015)

        rt = self.robot.GetRtTargetJointPos()
        if rt is not None:
            self.real_joints = list(rt)

        self._setup_gripper()

        log.info(f"Ready — joints: [{', '.join(f'{j:.1f}' for j in self.real_joints)}]")

    # ── Gripper setup ───────────────────────────────────────────────────

    def _setup_gripper(self):
        """Detect an attached MEGP gripper and apply force/velocity settings."""
        if not self.use_gripper:
            log.info(_dim("Gripper support disabled (--no-gripper)"))
            return

        try:
            tool = self.robot.GetRtExtToolStatus()
        except Exception as e:
            log.warning(f"Could not read external tool status: {e}")
            return

        self.tool_type = getattr(tool, "physical_tool_type", EXT_TOOL_NONE) or EXT_TOOL_NONE
        name = TOOL_NAMES.get(self.tool_type, f"type {self.tool_type}")

        if self.tool_type not in GRIPPER_TOOL_TYPES:
            log.info(_dim(f"No gripper attached (external tool: {name})"))
            return

        self.gripper_present = True
        self.gripper_homed = bool(getattr(tool, "homing_state", False))
        self.gripper_error = bool(getattr(tool, "error_status", False))

        # Real-time gripper position/force are not streamed by default
        try:
            self.robot.SetRealTimeMonitoring("all")
        except Exception as e:
            log.debug(f"SetRealTimeMonitoring: {e}")

        try:
            lo, hi = self.robot.GetGripperRange()
            if hi > lo:
                self.gripper_min, self.gripper_max = float(lo), float(hi)
        except Exception as e:
            log.debug(f"GetGripperRange unavailable, using defaults: {e}")

        try:
            self.robot.SetGripperForce(self.gripper_force)
            self.robot.SetGripperVel(self.gripper_vel)
        except Exception as e:
            log.warning(f"Gripper force/velocity setup failed: {e}")

        self.gripper_pos = self.gripper_max
        log.info(_green(
            f"Gripper: {name} — range {self.gripper_min:.1f}-{self.gripper_max:.1f} mm, "
            f"force {self.gripper_force}%, speed {self.gripper_vel}%"
        ))
        if not self.gripper_homed:
            log.warning(_yellow("Gripper is not homed — it will home on first move"))
        if self.gripper_error:
            log.warning(_red("Gripper reports an error state"))

    def _clamp_gripper(self, mm):
        return max(self.gripper_min, min(self.gripper_max, float(mm)))

    def _set_gripper_target(self, mm, force_send=False):
        """Request a gripper opening in mm. force_send skips the deadband."""
        if not (self.gripper_present and self.use_gripper):
            return
        with self._lock:
            self.gripper_target = self._clamp_gripper(mm)
            if force_send:
                self._gripper_force_send = True

    # ── Safety helpers ──────────────────────────────────────────────────

    def _clamp_to_limits(self, joints):
        return [
            max(lo, min(hi, j))
            for j, (lo, hi) in zip(joints, JOINT_LIMITS)
        ]

    def _compute_velocity(self, target, current):
        velocities = []
        for i in range(6):
            error = target[i] - current[i]
            vel = self.gain * error
            vel = max(-self.max_vel[i], min(self.max_vel[i], vel))
            if abs(error) < 0.05:
                vel = 0.0
            velocities.append(vel)
        return velocities

    def _stop_robot(self):
        if self.robot and not self.sim:
            try:
                self.robot.MoveJointsVel(0, 0, 0, 0, 0, 0)
            except Exception:
                pass

    def _emergency_stop(self):
        log.warning(_red("EMERGENCY STOP"))
        self.enabled = False
        self.paused = True
        if self.robot and not self.sim:
            try:
                self.robot.PauseMotion()
            except Exception:
                pass

    def _resume(self):
        if not self.paused:
            return
        # In simulation there is no robot to resume, but the paused flag still
        # has to clear or an e-stop leaves the sim permanently frozen.
        if self.sim:
            self.paused = False
            log.info(_green("Motion resumed (sim)"))
            return
        if not self.robot:
            return
        try:
            self.robot.ResumeMotion()
            self.paused = False
            log.info(_green("Motion resumed"))
        except Exception as e:
            log.error(f"Resume failed: {e}")

    def _reset_robot(self):
        log.info(_yellow("Resetting robot ..."))
        self.enabled = False
        self.paused = False
        if self.sim:
            log.info(_green("Reset complete (sim)"))
            return
        if not self.robot:
            return
        try:
            self.robot.DeactivateRobot()
            self.robot.WaitDeactivated(timeout=10)
        except Exception:
            pass
        try:
            self.robot.ResetError()
            self.robot.ResetError()
            self.robot.ActivateRobot()
            self.robot.WaitActivated(timeout=10)
            self.robot.Home()
            self.robot.WaitHomed(timeout=30)
            self.robot.ResumeMotion()
            self.robot.ClearMotion()
            self.robot.SetJointVel(self.vel_scale * 100)
            self.robot.SetJointAcc(50)
            self.robot.SetVelTimeout(0.1)
            rt = self.robot.GetRtTargetJointPos()
            if rt is not None:
                self.real_joints = list(rt)
            log.info(_green("Reset complete — robot ready"))
        except Exception as e:
            log.error(f"Reset failed: {e}")

    # ── WebSocket listener ──────────────────────────────────────────────

    async def ws_listener(self):
        try:
            async for message in self.ws:
                try:
                    data = json.loads(message)
                except json.JSONDecodeError:
                    continue

                msg_type = data.get("type")

                if msg_type == "state":
                    joints = data.get("joints")
                    if joints and len(joints) == 6:
                        with self._lock:
                            self.vr_target_joints = self._clamp_to_limits(joints)

                    collision = data.get("collision", False)
                    if collision and not self.collision_stopped:
                        self.collision_stopped = True
                        self._stop_robot()
                        pairs = data.get("collisions", [])
                        pair_str = ", ".join(f"{p['link']}<->{p['object']}" for p in pairs[:3])
                        log.warning(_red(f"COLLISION DETECTED — robot stopped: {pair_str}"))
                    elif not collision and self.collision_stopped:
                        self.collision_stopped = False
                        log.info(_green("Collision cleared — resuming"))

                elif msg_type == "bridgeCommand":
                    self._handle_bridge_command(data)

        except websockets.ConnectionClosed:
            log.warning("WebSocket disconnected")
            self._stop_robot()
            self.enabled = False

    def _handle_bridge_command(self, data):
        cmd = data.get("cmd")
        if cmd == "enable":
            if self.paused:
                self._resume()
            if self.robot and not self.sim:
                self.robot.ResumeMotion()
            self.enabled = True
            log.info(_green("Teleoperation ENABLED"))
        elif cmd == "disable":
            self.enabled = False
            self._stop_robot()
            log.info(_yellow("Teleoperation DISABLED"))
        elif cmd == "estop":
            self._emergency_stop()
        elif cmd == "resume":
            self._resume()
            self.enabled = True
        elif cmd == "reset":
            self._reset_robot()
        elif cmd == "setVelScale":
            scale = max(0.01, min(1.0, data.get("scale", 0.25)))
            self.vel_scale = scale
            self.max_vel = [v * scale for v in MAX_JOINT_VEL]
            if self.robot and not self.sim:
                self.robot.SetJointVel(scale * 100)
            log.info(f"Velocity scale: {scale * 100:.0f}%")

        # ── Gripper ─────────────────────────────────────────────────────
        elif cmd == "gripperOpen":
            self._set_gripper_target(self.gripper_max, force_send=True)
            log.info("Gripper: OPEN")
        elif cmd == "gripperClose":
            self._set_gripper_target(self.gripper_min, force_send=True)
            log.info("Gripper: CLOSE")
        elif cmd == "setGripper":
            # Accept either an absolute opening in mm or a normalised 0-1
            # fraction, which is what the analogue VR trigger sends.
            if "mm" in data:
                target = data.get("mm", self.gripper_max)
            else:
                frac = max(0.0, min(1.0, float(data.get("opening", 1.0))))
                target = self.gripper_min + frac * (self.gripper_max - self.gripper_min)
            self._set_gripper_target(target, force_send=bool(data.get("immediate")))
        elif cmd == "setGripperForce":
            self.gripper_force = max(5, min(100, int(data.get("force", 50))))
            if self.robot and not self.sim and self.gripper_present:
                try:
                    self.robot.SetGripperForce(self.gripper_force)
                except Exception as e:
                    log.warning(f"SetGripperForce failed: {e}")
            log.info(f"Gripper force: {self.gripper_force}%")
        elif cmd == "setGripperVel":
            self.gripper_vel = max(5, min(100, int(data.get("vel", 50))))
            if self.robot and not self.sim and self.gripper_present:
                try:
                    self.robot.SetGripperVel(self.gripper_vel)
                except Exception as e:
                    log.warning(f"SetGripperVel failed: {e}")
            log.info(f"Gripper speed: {self.gripper_vel}%")

    # ── Control loop (runs in background thread) ────────────────────────

    def control_loop(self):
        period = 1.0 / self.update_hz

        while self.running:
            t0 = time.monotonic()

            try:
                if not self.sim and self.robot:
                    rt = self.robot.GetRtTargetJointPos()
                    if rt is not None:
                        self.real_joints = list(rt)

                self._gripper_tick(period)

                if self.enabled and not self.paused and not self.collision_stopped:
                    with self._lock:
                        target = self.vr_target_joints

                    if target is not None:
                        if self.sim:
                            alpha = min(1.0, self.gain * period)
                            self.real_joints = [
                                c + alpha * (t - c)
                                for c, t in zip(self.real_joints, target)
                            ]
                        else:
                            vel = self._compute_velocity(target, self.real_joints)
                            if any(abs(v) > 0.01 for v in vel):
                                self.robot.MoveJointsVel(*vel)
                            else:
                                self.robot.MoveJointsVel(0, 0, 0, 0, 0, 0)

            except Exception as e:
                log.error(f"Control loop: {e}")
                self._stop_robot()
                self.enabled = False

            elapsed = time.monotonic() - t0
            remaining = period - elapsed
            if remaining > 0:
                time.sleep(remaining)

    # ── Gripper dispatch (called from the control loop) ──────────────────

    def _gripper_tick(self, period):
        """Push the pending gripper target to the robot and read state back.

        Gripper moves go into the robot's motion queue, so the streamed
        analogue target is throttled by a deadband and a minimum interval
        rather than being sent every tick.
        """
        if not (self.gripper_present and self.use_gripper):
            return

        # Read actual position/state back first, so the UI still updates while
        # motion is stopped.
        if not self.sim and self.robot:
            try:
                rt_data = self.robot.GetRobotRtData()
                pos = getattr(rt_data, "rt_gripper_pos", None)
                if pos is not None and getattr(pos, "data", None):
                    self.gripper_pos = float(pos.data[0])
                state = self.robot.GetRtGripperState()
                if state is not None:
                    self.gripper_holding = bool(getattr(state, "holding_part", False))
            except Exception as e:
                log.debug(f"Gripper readback: {e}")

        # An e-stop or collision stop must leave the gripper exactly where it
        # is — dropping a held part on a fault would make things worse.
        if self.paused or self.collision_stopped:
            return

        with self._lock:
            target = self.gripper_target
            forced = self._gripper_force_send
            self._gripper_force_send = False

        if target is None:
            return

        if self.sim:
            # Ease toward the target every tick at the configured gripper
            # speed, so simulated motion is smooth rather than stepped.
            span = max(1e-6, self.gripper_max - self.gripper_min)
            rate = span * (self.gripper_vel / 100.0) * 2.0   # mm/s
            step = rate * period
            delta = target - self.gripper_pos
            self.gripper_pos += max(-step, min(step, delta))
            self.gripper_holding = False
            self._gripper_sent = target
            return

        now = time.monotonic()
        moved = self._gripper_sent is None or abs(target - self._gripper_sent) >= GRIPPER_DEADBAND_MM
        due = (now - self._gripper_sent_t) >= GRIPPER_MIN_INTERVAL

        if not forced and not (moved and due):
            return

        try:
            self.robot.MoveGripper(target)
        except Exception as e:
            log.warning(f"MoveGripper failed: {e}")
            return

        self._gripper_sent = target
        self._gripper_sent_t = now

    # ── Status sender ───────────────────────────────────────────────────

    async def status_sender(self):
        send_count = 0
        while self.running:
            try:
                at_target = False
                with self._lock:
                    target = self.vr_target_joints
                if target is not None:
                    at_target = all(
                        abs(t - r) < 0.5
                        for t, r in zip(target, self.real_joints)
                    )

                status = {
                    "cmd": "bridgeStatus",
                    "robotConnected": self.robot_connected,
                    "robotHomed": self.robot_homed,
                    "enabled": self.enabled,
                    "paused": self.paused,
                    "collisionStopped": self.collision_stopped,
                    "realJoints": [round(j, 2) for j in self.real_joints],
                    "velScale": round(self.vel_scale, 2),
                    "atTarget": at_target,
                    "sim": self.sim,
                    "gripper": {
                        "present": self.gripper_present,
                        "tool": TOOL_NAMES.get(self.tool_type, str(self.tool_type)),
                        "homed": self.gripper_homed,
                        "error": self.gripper_error,
                        "pos": round(self.gripper_pos, 2),
                        "min": round(self.gripper_min, 2),
                        "max": round(self.gripper_max, 2),
                        "force": self.gripper_force,
                        "vel": self.gripper_vel,
                        "holding": self.gripper_holding,
                    },
                }
                if self.ws:
                    await self.ws.send(json.dumps(status))
                    send_count += 1
                    if send_count <= 3 or send_count % 50 == 0:
                        log.info(f"bridgeStatus sent (#{send_count})")
            except Exception as e:
                log.warning(f"status_sender error: {e}")
            await asyncio.sleep(0.1)

    # ── State poller ────────────────────────────────────────────────────

    async def state_poller(self):
        await asyncio.sleep(1.0)
        while self.running:
            try:
                if self.ws:
                    await self.ws.send(json.dumps({"cmd": "getState"}))
            except Exception as e:
                log.warning(f"state_poller error: {e}")
            await asyncio.sleep(0.5)

    # ── Main entry ──────────────────────────────────────────────────────

    async def run(self):
        self.running = True

        self.connect_robot()

        if self.auto_enable:
            self.enabled = True
            log.info(_green("Teleoperation auto-enabled"))

        control_thread = threading.Thread(target=self.control_loop, daemon=True)
        control_thread.start()

        self._print_controls()

        try:
            while self.running:
                try:
                    await self.connect_ws()
                except Exception as e:
                    log.warning(f"WebSocket connect failed: {e} — retrying in 3s")
                    await asyncio.sleep(3)
                    continue

                try:
                    await asyncio.gather(
                        self.ws_listener(),
                        self.status_sender(),
                        self.state_poller(),
                    )
                except (websockets.ConnectionClosed, ConnectionError):
                    log.warning(_yellow("WebSocket lost — reconnecting in 3s"))
                    self._stop_robot()
                    await asyncio.sleep(3)
        except asyncio.CancelledError:
            pass
        finally:
            self.running = False
            self._stop_robot()
            self._shutdown_robot()
            if self.ws:
                await self.ws.close()

    def _shutdown_robot(self):
        if not self.robot or self.sim:
            return
        log.info("Shutting down robot ...")
        try:
            self.robot.MoveJointsVel(0, 0, 0, 0, 0, 0)
            time.sleep(0.5)
            self.robot.DeactivateRobot()
            self.robot.WaitDeactivated(timeout=10)
            self.robot.Disconnect()
            log.info(_green("Robot deactivated and disconnected"))
        except Exception as e:
            log.error(f"Shutdown error: {e}")

    def _print_controls(self):
        mode = _yellow("SIMULATION") if self.sim else _green("LIVE")
        print()
        print(f"  ╔══════════════════════════════════════════╗")
        print(f"  ║      Meca500 VR Bridge — {mode:<20s}║")
        print(f"  ╠══════════════════════════════════════════╣")
        print(f"  ║  Robot IP:   {self.robot_ip:<28s}║")
        print(f"  ║  Vel scale:  {self.vel_scale * 100:>5.0f}% of max               ║")
        print(f"  ║  Control Hz: {self.update_hz:>5d}                      ║")
        print(f"  ║  Enabled:    {'YES' if self.enabled else 'NO ':<28s}║")
        grip = (f"{TOOL_NAMES.get(self.tool_type, '?')} "
                f"({self.gripper_min:.1f}-{self.gripper_max:.1f} mm)"
                if self.gripper_present else "none")
        print(f"  ║  Gripper:    {grip:<28s}║")
        print(f"  ╠══════════════════════════════════════════╣")
        print(f"  ║  Enable/disable from VR panel or send    ║")
        print(f"  ║  bridgeCommand via WebSocket.             ║")
        print(f"  ║  Ctrl+C = clean shutdown                  ║")
        print(f"  ╚══════════════════════════════════════════╝")
        print()


def main():
    parser = argparse.ArgumentParser(
        description="Meca500 VR Bridge — teleoperate a real Meca500 from VR",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
examples:
  %(prog)s --sim                          Simulation (no robot needed)
  %(prog)s --robot-ip 192.168.0.100       Connect to robot at default IP
  %(prog)s --vel-scale 0.1 --gain 1.5     Slower, gentler control
  %(prog)s --session abc123               Target specific VR session
""")
    parser.add_argument("--robot-ip", default="192.168.0.100",
                        help="Meca500 IP address (default: 192.168.0.100)")
    parser.add_argument("--ws-url", default="ws://localhost:8080/ws",
                        help="VR viewer WebSocket URL (default: ws://localhost:8080/ws)")
    parser.add_argument("--session", default=None,
                        help="Target a specific viewer session ID")
    parser.add_argument("--vel-scale", type=float, default=0.25,
                        help="Velocity scaling 0-1 (default: 0.25 = 25%%)")
    parser.add_argument("--gain", type=float, default=2.0,
                        help="Proportional control gain (default: 2.0)")
    parser.add_argument("--hz", type=int, default=50,
                        help="Control loop frequency in Hz (default: 50)")
    parser.add_argument("--auto-enable", action="store_true",
                        help="Enable teleoperation immediately on start")
    parser.add_argument("--sim", action="store_true",
                        help="Simulation mode (no real robot)")
    parser.add_argument("--gripper-force", type=int, default=50,
                        help="MEGP 25E gripper force in percent, 5-100 (default: 50)")
    parser.add_argument("--gripper-vel", type=int, default=50,
                        help="MEGP 25E gripper speed in percent, 5-100 (default: 50)")
    parser.add_argument("--no-gripper", action="store_true",
                        help="Ignore any attached gripper")

    args = parser.parse_args()

    bridge = Meca500Bridge(
        robot_ip=args.robot_ip,
        ws_url=args.ws_url,
        session=args.session,
        vel_scale=args.vel_scale,
        gain=args.gain,
        update_hz=args.hz,
        auto_enable=args.auto_enable,
        sim=args.sim,
        gripper_force=args.gripper_force,
        gripper_vel=args.gripper_vel,
        use_gripper=not args.no_gripper,
    )

    try:
        asyncio.run(bridge.run())
    except KeyboardInterrupt:
        print(f"\n  {_yellow('Interrupted')} — shutting down")


if __name__ == "__main__":
    main()
