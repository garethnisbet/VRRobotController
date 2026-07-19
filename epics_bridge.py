#!/usr/bin/env python3
"""
VR-to-EPICS Bridge — Teleoperate a Meca500 through the EPICS IOC

Bridges the WebSocket-based VR viewer to a physical Meca500 via the
pvAccess IOC, using velocity-mode PVs. Joint angles from VR become
velocity targets written to CMD:VEL_JOINTS; feedback comes from the
IOC's JOINT:J* PVs rather than direct TCP to the robot.

Architecture:
    VR Headset ←WebSocket→ server.py ←WebSocket→ epics_bridge.py ←pvAccess→ EPICS IOC ←TCP→ Meca500

Compared to the direct meca500_bridge.py, routing through EPICS adds
pvAccess round-trip latency but gives PV monitoring, archiving, and
interlock integration. The robot's SetVelTimeout hardware deadman is
active in both paths.

Usage:
    pip install p4p websockets
    python epics_bridge.py [--prefix MECA500] [--ws-url ws://localhost:8080/ws]
"""

import argparse
import asyncio
import json
import logging
import ssl
import sys
import threading
import time

import numpy as np

try:
    from p4p.client.thread import Context
except ImportError:
    print("p4p not installed. Run: pip install p4p")
    sys.exit(1)

try:
    import websockets
except ImportError:
    print("websockets not installed. Run: pip install websockets")
    sys.exit(1)

logging.basicConfig(level=logging.INFO, format="%(asctime)s  %(message)s")
log = logging.getLogger("vr-epics-bridge")

JOINT_LIMITS = [
    (-175, 175),
    (-70,   90),
    (-135,  70),
    (-170, 170),
    (-115, 115),
    (-36000, 36000),
]

MAX_JOINT_VEL = [150, 150, 180, 300, 300, 500]


def _c(code, text):
    return f"\033[{code}m{text}\033[0m" if sys.stdout.isatty() else text

def _green(t):  return _c("32", t)
def _yellow(t): return _c("33", t)
def _red(t):    return _c("1;31", t)


class VREpicsBridge:
    """Bidirectional bridge between VR viewer (WebSocket) and Meca500 (pvAccess IOC)."""

    def __init__(self, prefix, ws_url, session=None,
                 vel_scale=0.25, gain=2.0, update_hz=50,
                 auto_enable=False, sim=False):
        self.prefix = prefix
        self.ws_url = ws_url
        self.session = session
        self.vel_scale = max(0.01, min(1.0, vel_scale))
        self.gain = gain
        self.update_hz = update_hz
        self.auto_enable = auto_enable
        self.sim = sim

        self._ctx: Context | None = None
        self.ws = None
        self.running = False
        self.enabled = False
        self.robot_connected = False
        self.robot_homed = False
        self.vel_mode_active = False
        self.paused = False

        self.vr_target_joints = None
        self.real_joints = [0.0] * 6
        self.collision_stopped = False
        self._lock = threading.Lock()

        self.max_vel = [v * self.vel_scale for v in MAX_JOINT_VEL]

    def _pv(self, suffix: str) -> str:
        return f"{self.prefix}:{suffix}"

    # ── pvAccess connection ────────────────────────────────────────────

    def connect_ioc(self):
        if self.sim:
            log.info(_yellow("SIMULATION MODE — no EPICS IOC"))
            self.robot_connected = True
            self.robot_homed = True
            return

        log.info(f"Connecting to EPICS IOC (prefix={self.prefix}) ...")
        self._ctx = Context("pva")

        for attempt in range(30):
            try:
                activated = self._ctx.get(self._pv("STATUS:ACTIVATED"))
                break
            except TimeoutError:
                if attempt == 0:
                    log.info("Waiting for IOC to come online ...")
                time.sleep(1.0)
        else:
            log.error("IOC not reachable after 30s. Is it running?")
            sys.exit(1)

        log.info(_green("IOC connected"))

        activated = int(self._ctx.get(self._pv("STATUS:ACTIVATED")))
        homed = int(self._ctx.get(self._pv("STATUS:HOMED")))

        if not activated:
            log.info("Activating robot via IOC ...")
            self._ctx.put(self._pv("CMD:ACTIVATE"), 1)
            self._wait_for_pv("STATUS:ACTIVATED", 1, timeout=15)
            log.info(_green("Robot activated"))

        if not homed:
            log.info("Homing robot via IOC ...")
            self._ctx.put(self._pv("CMD:HOME"), 1)
            self._wait_for_pv("STATUS:HOMED", 1, timeout=30)
            log.info(_green("Robot homed"))

        self.robot_connected = True
        self.robot_homed = True

        self._read_joints()
        log.info(f"Ready — joints: [{', '.join(f'{j:.1f}' for j in self.real_joints)}]")

    def _wait_for_pv(self, suffix: str, target: int, timeout: float = 10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            val = int(self._ctx.get(self._pv(suffix)))
            if val == target:
                return
            time.sleep(0.2)
        raise TimeoutError(f"{suffix} did not reach {target} within {timeout}s")

    def _read_joints(self):
        if self.sim:
            return
        joints = []
        for i in range(1, 7):
            joints.append(float(self._ctx.get(self._pv(f"JOINT:J{i}"))))
        self.real_joints = joints

    def _enable_vel_mode(self):
        if self.sim or self.vel_mode_active:
            return
        self._ctx.put(self._pv("CMD:VEL_ENABLE"), 1)
        self._wait_for_pv("STATUS:VEL_MODE", 1, timeout=5)
        self.vel_mode_active = True
        log.info(_green("Velocity mode enabled via IOC"))

    def _disable_vel_mode(self):
        if self.sim or not self.vel_mode_active:
            return
        self._ctx.put(self._pv("CMD:VEL_DISABLE"), 1)
        self.vel_mode_active = False
        log.info("Velocity mode disabled via IOC")

    def _send_velocity(self, velocities: list[float]):
        if self.sim:
            return
        self._ctx.put(self._pv("CMD:VEL_JOINTS"), np.array(velocities, dtype="float64"))

    # ── WebSocket connection ───────────────────────────────────────────

    async def connect_ws(self):
        url = self.ws_url
        sep = "&" if "?" in url else "?"
        url += f"{sep}role=controller"
        if self.session:
            url += f"&session={self.session}"

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

    # ── Safety helpers ─────────────────────────────────────────────────

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
        if not self.sim and self.vel_mode_active:
            try:
                self._send_velocity([0, 0, 0, 0, 0, 0])
            except Exception:
                pass

    def _emergency_stop(self):
        log.warning(_red("EMERGENCY STOP"))
        self.enabled = False
        self.paused = True
        if not self.sim:
            try:
                self._ctx.put(self._pv("CMD:PAUSE"), 1)
            except Exception:
                pass

    def _resume(self):
        if self.paused and not self.sim:
            try:
                self._ctx.put(self._pv("CMD:RESUME"), 1)
                self.paused = False
                log.info(_green("Motion resumed"))
            except Exception as e:
                log.error(f"Resume failed: {e}")

    def _reset_robot(self):
        log.info(_yellow("Resetting robot ..."))
        self.enabled = False
        self.paused = False
        self._disable_vel_mode()
        if self.sim:
            log.info(_green("Reset complete (sim)"))
            return
        if not self._ctx:
            return
        try:
            self._ctx.put(self._pv("CMD:DEACTIVATE"), 1)
            time.sleep(2)
            self._ctx.put(self._pv("CMD:RESET_ERROR"), 1)
            time.sleep(1)
            self._ctx.put(self._pv("CMD:ACTIVATE"), 1)
            self._wait_for_pv("STATUS:ACTIVATED", 1, timeout=15)
            self._ctx.put(self._pv("CMD:HOME"), 1)
            self._wait_for_pv("STATUS:HOMED", 1, timeout=30)
            self._read_joints()
            log.info(_green("Reset complete — robot ready"))
        except Exception as e:
            log.error(f"Reset failed: {e}")

    # ── WebSocket listener ─────────────────────────────────────────────

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
            self._enable_vel_mode()
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
            self._enable_vel_mode()
            self.enabled = True
        elif cmd == "reset":
            self._reset_robot()
        elif cmd == "setVelScale":
            scale = max(0.01, min(1.0, data.get("scale", 0.25)))
            self.vel_scale = scale
            self.max_vel = [v * scale for v in MAX_JOINT_VEL]
            log.info(f"Velocity scale: {scale * 100:.0f}%")

    # ── Control loop ───────────────────────────────────────────────────

    def control_loop(self):
        period = 1.0 / self.update_hz

        while self.running:
            t0 = time.monotonic()

            try:
                if not self.sim:
                    self._read_joints()

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
                                self._send_velocity(vel)
                            else:
                                self._send_velocity([0, 0, 0, 0, 0, 0])

            except Exception as e:
                log.error(f"Control loop: {e}")
                self._stop_robot()
                self.enabled = False

            elapsed = time.monotonic() - t0
            remaining = period - elapsed
            if remaining > 0:
                time.sleep(remaining)

    # ── Status sender ──────────────────────────────────────────────────

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
                }
                if self.ws:
                    await self.ws.send(json.dumps(status))
                    send_count += 1
                    if send_count <= 3 or send_count % 50 == 0:
                        log.info(f"bridgeStatus sent (#{send_count})")
            except Exception as e:
                log.warning(f"status_sender error: {e}")
            await asyncio.sleep(0.1)

    # ── State poller ───────────────────────────────────────────────────

    async def state_poller(self):
        await asyncio.sleep(1.0)
        while self.running:
            try:
                if self.ws:
                    await self.ws.send(json.dumps({"cmd": "getState"}))
            except Exception as e:
                log.warning(f"state_poller error: {e}")
            await asyncio.sleep(0.5)

    # ── Main entry ─────────────────────────────────────────────────────

    async def run(self):
        self.running = True

        self.connect_ioc()

        if self.auto_enable:
            self._enable_vel_mode()
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
            self._shutdown()
            if self.ws:
                await self.ws.close()

    def _shutdown(self):
        self._disable_vel_mode()
        if self._ctx:
            self._ctx.close()
            self._ctx = None
        log.info(_green("Bridge shut down"))

    def _print_controls(self):
        mode = _yellow("SIMULATION") if self.sim else _green("LIVE via EPICS")
        print()
        print(f"  ╔══════════════════════════════════════════╗")
        print(f"  ║   VR → EPICS Bridge — {mode:<20s}║")
        print(f"  ╠══════════════════════════════════════════╣")
        print(f"  ║  PV Prefix:  {self.prefix:<28s}║")
        print(f"  ║  Vel scale:  {self.vel_scale * 100:>5.0f}% of max               ║")
        print(f"  ║  Control Hz: {self.update_hz:>5d}                      ║")
        print(f"  ║  Enabled:    {'YES' if self.enabled else 'NO ':<28s}║")
        print(f"  ╠══════════════════════════════════════════╣")
        print(f"  ║  Enable/disable from VR panel or send    ║")
        print(f"  ║  bridgeCommand via WebSocket.             ║")
        print(f"  ║  Ctrl+C = clean shutdown                  ║")
        print(f"  ╚══════════════════════════════════════════╝")
        print()


def main():
    parser = argparse.ArgumentParser(
        description="VR-to-EPICS Bridge — teleoperate a Meca500 through the EPICS IOC",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
examples:
  %(prog)s --sim                          Simulation (no IOC needed)
  %(prog)s --prefix MECA500               Connect to IOC with default prefix
  %(prog)s --vel-scale 0.1 --gain 1.5     Slower, gentler control
  %(prog)s --session abc123               Target specific VR session
""")
    parser.add_argument("--prefix", default="MECA500",
                        help="EPICS PV prefix (default: MECA500)")
    parser.add_argument("--ws-url", default="ws://localhost:8443/ws",
                        help="VR viewer WebSocket URL (default: ws://localhost:8443/ws)")
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
                        help="Simulation mode (no EPICS IOC needed)")

    args = parser.parse_args()

    bridge = VREpicsBridge(
        prefix=args.prefix,
        ws_url=args.ws_url,
        session=args.session,
        vel_scale=args.vel_scale,
        gain=args.gain,
        update_hz=args.hz,
        auto_enable=args.auto_enable,
        sim=args.sim,
    )

    try:
        asyncio.run(bridge.run())
    except KeyboardInterrupt:
        print(f"\n  {_yellow('Interrupted')} — shutting down")


if __name__ == "__main__":
    main()
