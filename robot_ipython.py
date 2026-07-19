#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot/Device — IPython Remote Control Terminal

Interactive IPython client for controlling the viewer via WebSocket API.
Works with any device config (Meca500, i16 diffractometer, etc.).
Exposes a `robot` object with methods for all commands — use full Python
syntax (loops, variables, etc.) alongside robot control.

Usage:
    pip install websockets ipython
    python robot_ipython.py [--url ws://localhost:8080/ws] [--config meca500_config.json]
"""

import argparse
import asyncio
import atexit
import itertools
import json
import math
import os
import ssl
import sys
import threading
import time
import urllib.request
import websockets
import numpy as np
from GNKinematics import kinematics
from RobotDefinitions import Meca500_kin, GP225_kin, GP180_120_kin, GP280_kin, MotoMini_kin


# ── ANSI colour helpers ──────────────────────────────────────────────────────

_COLORS = os.getenv("NO_COLOR") is None and sys.stdout.isatty()

def _c(code, text):
    return f"\033[{code}m{text}\033[0m" if _COLORS else text

def _bold(t):     return _c("1", t)
def _dim(t):      return _c("2", t)
def _red(t):      return _c("31", t)
def _green(t):    return _c("32", t)
def _yellow(t):   return _c("33", t)
def _blue(t):     return _c("34", t)
def _magenta(t):  return _c("35", t)
def _cyan(t):     return _c("36", t)
def _white(t):    return _c("37", t)
def _bred(t):     return _c("1;31", t)
def _bgreen(t):   return _c("1;32", t)
def _byellow(t):  return _c("1;33", t)
def _bcyan(t):    return _c("1;36", t)


# ── Config loading ───────────────────────────────────────────────────────────

_http_base_url = None

def _ws_url_to_http(ws_url):
    from urllib.parse import urlparse
    p = urlparse(ws_url)
    scheme = "https" if p.scheme == "wss" else "http"
    return f"{scheme}://{p.hostname}:{p.port}" if p.port else f"{scheme}://{p.hostname}"

def _load_config(config_path):
    try:
        with open(config_path) as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        pass
    if _http_base_url:
        filename = os.path.basename(config_path)
        url = f"{_http_base_url}/{filename}"
        try:
            ctx = ssl.create_default_context()
            if url.startswith("https://"):
                ctx.check_hostname = False
                ctx.verify_mode = ssl.CERT_NONE
            with urllib.request.urlopen(url, timeout=5, context=ctx) as resp:
                return json.load(resp)
        except Exception as e:
            print(f"  {_yellow('Warning')}: Could not fetch config {filename} from {url}: {e}")
            return None
    print(f"  {_yellow('Warning')}: Could not load config {config_path}")
    return None

def _get_movable_joints(config):
    if not config:
        return []
    movable = []
    si = 0
    for j in config["joints"]:
        if not j.get("fixed"):
            movable.append((si, j["name"]))
            si += 1
    return movable

# ── Path planning helpers ────────────────────────────────────────────────────

def _densify_path(path, step_deg):
    dense = [path[0]]
    for i in range(len(path) - 1):
        q0 = path[i]
        q1 = path[i + 1]
        max_delta = max(abs(a - b) for a, b in zip(q0, q1))
        n_steps = max(1, math.ceil(max_delta / step_deg))
        for k in range(1, n_steps + 1):
            t = k / n_steps
            dense.append([a + t * (b - a) for a, b in zip(q0, q1)])
    return dense

def _build_axis_vals(start, end, step):
    step = abs(step)
    direction = 1.0 if end >= start else -1.0
    n_steps = int(math.floor(abs(end - start) / step)) + 1
    vals = []
    for i in range(n_steps):
        val = start + i * step * direction
        if direction > 0 and val > end:
            val = end
        elif direction < 0 and val < end:
            val = end
        vals.append(val)
    if not vals or vals[-1] != end:
        vals.append(end)
    return vals

def _is_array_like(obj):
    """Return True if obj looks like a 2D array (list of lists, numpy array, etc.)."""
    if callable(obj):
        return False
    if isinstance(obj, str):
        return False
    try:
        iter(obj)
        return True
    except TypeError:
        return False


def _resolve_axis_for_device(name, movable_joints):
    nl = name.lower()
    for mi, (_, jname) in enumerate(movable_joints):
        if jname.lower() == nl:
            return mi
        if jname.lower().split()[0] == nl:
            return mi
    try:
        n = int(name)
        if 1 <= n <= len(movable_joints):
            return n - 1
    except ValueError:
        pass
    return None

# ── Object reference parsing ─────────────────────────────────────────────────

def _parse_obj_ref(token):
    if isinstance(token, int):
        return {"index": token}
    token = str(token)
    if token.startswith("#") and token[1:].isdigit():
        return {"index": int(token[1:])}
    return {"object": token}


# ── Frame conversions (API Z-up ↔ Three.js Y-up) ─────────────────────────────
#
# The viewer reports positions in mm and orientations as Euler angles (deg)
# built from a Three.js Quaternion with order 'XYZ' (intrinsic). Axes are
# swapped at the boundary so API space has Z-up while Three uses Y-up:
#   API [x, y, z] ↔ Three [x, z, y]

def _api_to_three_vec(v):
    # API Z-up (X right, Y into scene, Z up) → Three.js Y-up (X right, Y up, Z toward viewer)
    # API Y = -Three Z (opposite directions: into-scene vs toward-viewer)
    return np.array([float(v[0]), float(v[2]), -float(v[1])])

def _three_to_api_vec(v):
    return np.array([float(v[0]), -float(v[2]), float(v[1])])

def _api_euler_to_three(rot_deg):
    # Rotation around API Y (into scene) = rotation around -Three Z (toward viewer) → negate ry.
    return (np.radians(rot_deg[0]),    # Three ex = rx_api
            np.radians(rot_deg[2]),    # Three ey = rz_api  (API Z = Three Y)
            -np.radians(rot_deg[1]))   # Three ez = -ry_api (API Y = -Three Z)

def _three_euler_to_api(ex, ey, ez):
    return [float(np.degrees(ex)), float(np.degrees(-ez)), float(np.degrees(ey))]

def _rot_xyz_three(ex, ey, ez):
    """Three.js Euler 'XYZ' intrinsic: R = Rx(ex) · Ry(ey) · Rz(ez)."""
    cx, sx = np.cos(ex), np.sin(ex)
    cy, sy = np.cos(ey), np.sin(ey)
    cz, sz = np.cos(ez), np.sin(ez)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rx @ Ry @ Rz

def _euler_xyz_from_matrix(R):
    """Inverse of _rot_xyz_three (Three.js Euler 'XYZ' convention)."""
    ey = np.arcsin(max(-1.0, min(1.0, R[0, 2])))
    if abs(R[0, 2]) < 0.9999999:
        ex = np.arctan2(-R[1, 2], R[2, 2])
        ez = np.arctan2(-R[0, 1], R[0, 0])
    else:
        ex = np.arctan2(R[2, 1], R[1, 1])
        ez = 0.0
    return ex, ey, ez


# ═══════════════════════════════════════════════════════════════════════════════
#  RobotClient — the main API class
# ═══════════════════════════════════════════════════════════════════════════════

class RobotClient:
    """Interactive robot/device controller for IPython.

    All methods are synchronous — they submit async work to a background
    event loop thread and block for the result. No ``await`` needed.
    """

    def __init__(self, url="ws://localhost:8080/ws", config="meca500_config.json",
                 session=None, connect=True):
        global _http_base_url
        self._url = url
        _http_base_url = _ws_url_to_http(url)
        self._config_path = config
        self._session = session

        self._ssl_ctx = None
        if self._url.startswith("wss://"):
            self._ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
            self._ssl_ctx.check_hostname = False
            self._ssl_ctx.verify_mode = ssl.CERT_NONE

        # Background event loop
        self._loop = None
        self._thread = None
        self._ws = None
        self._listener_task = None
        self._ws_pending = {}

        # Print control
        self._print_state = True

        # Device state
        self._config = _load_config(config)
        self._device_name = self._config["name"] if self._config else "Robot"
        self._movable_joints = _get_movable_joints(self._config)
        self._n_movable = len(self._movable_joints)
        self._joint_name_to_idx = {name.lower(): idx for idx, name in self._movable_joints}
        self._device_names_cache = []

        if connect:
            self.connect()

    # ── Background loop management ───────────────────────────────────────

    def _start_loop(self):
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()

    def _run(self, coro, timeout=5.0):
        """Submit a coroutine to the background loop and block for the result."""
        if not self._loop or not self._loop.is_running():
            raise RuntimeError("Background event loop not running. Call connect() first.")
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout=timeout)

    def _send(self, msg):
        """Send a JSON message over the WebSocket (fire-and-forget)."""
        async def _impl():
            if self._ws:
                await self._ws.send(json.dumps(msg))
        self._run(_impl(), timeout=3.0)

    def _send_and_wait(self, msg, response_type, timeout=3.0):
        """Send a message and wait for a specific response type."""
        if not self._ws:
            print(f"  {_bred('Error')}: not connected. Call robot.connect() first.")
            return None
        # State echoes stream continuously during a scan (one per setJoints),
        # each tagged with its device. When this request targets a device, only
        # accept a reply for that same device so a stale or wrong-device echo
        # can't satisfy the wait with the wrong pose.
        want_device = msg.get("device") if response_type == "state" else None
        async def _impl():
            event = asyncio.Event()
            self._ws_pending[response_type] = {"event": event, "data": None,
                                               "device": want_device}
            try:
                await self._ws.send(json.dumps(msg))
                await asyncio.wait_for(event.wait(), timeout=timeout)
                return self._ws_pending[response_type]["data"]
            except asyncio.TimeoutError:
                return None
            finally:
                self._ws_pending.pop(response_type, None)
        return self._run(_impl(), timeout=timeout + 2.0)

    # ── Connection lifecycle ─────────────────────────────────────────────

    def connect(self):
        """Connect to the WebSocket server."""
        if self._loop is None:
            self._start_loop()

        async def _impl():
            self._ws = await websockets.connect(self._url, ssl=self._ssl_ctx)
            self._listener_task = asyncio.ensure_future(self._print_incoming())

        print(f"  Connecting to {_dim(self._url)} ...")
        try:
            self._run(_impl(), timeout=10.0)
        except Exception as e:
            print(f"  {_bred('Connection failed')}: {e}")
            print(f"  Is the server running?  python server.py --config {self._config_path}")
            return
        print(f"  {_bgreen('Connected!')}")
        atexit.register(self.disconnect)

    def disconnect(self):
        """Close the WebSocket connection and stop the background thread."""
        if self._ws:
            try:
                self._run(self._ws.close(), timeout=3.0)
            except Exception:
                pass
            self._ws = None
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._loop = None
        self._thread = None
        print(f"  {_dim('Disconnected.')}")

    def reconnect(self):
        """Disconnect and reconnect."""
        if self._ws:
            try:
                self._run(self._ws.close(), timeout=3.0)
            except Exception:
                pass
            self._ws = None
        if self._listener_task:
            self._listener_task.cancel()
            self._listener_task = None

        async def _impl():
            self._ws = await websockets.connect(self._url, ssl=self._ssl_ctx)
            self._listener_task = asyncio.ensure_future(self._print_incoming())

        print(f"  Reconnecting to {_dim(self._url)} ...")
        try:
            self._run(_impl(), timeout=10.0)
            print(f"  {_bgreen('Connected!')}")
        except Exception as e:
            print(f"  {_bred('Connection failed')}: {e}")

    # ── Background listener ──────────────────────────────────────────────

    async def _print_incoming(self):
        """Background task: listen for messages and print/dispatch them."""
        try:
            async for message in self._ws:
                data = json.loads(message)
                resp_type = data.get("type")

                # Check pending responses first. If the waiter is bound to a
                # device, ignore echoes for other devices (and let them fall
                # through to normal handling) instead of resolving with them.
                pending = self._ws_pending.get(resp_type)
                if pending and (pending.get("device") is None
                                or data.get("device") == pending["device"]):
                    pending["data"] = data
                    pending["event"].set()
                    continue

                if not self._print_state:
                    continue

                output = self._format_message(data)
                if output:
                    print(output, flush=True)
        except websockets.ConnectionClosed:
            print(f"\n  {_bred('Connection lost.')}", flush=True)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            print(f"\n  {_bred('Listener error')}: {e}", flush=True)

    def _format_message(self, data):
        """Format a WebSocket message for display. Returns a string or None."""
        msg_type = data.get("type")
        lines = []

        if msg_type == "state":
            if data.get("deviceType") == "hexapod":
                pose = data.get("platformPose", [0]*6)
                legs = data.get("legLengths", [])
                lines.append(f"  {_bold('STATE')} | {_bgreen('hexapod')}")
                lines.append(
                    f"         pose: x={_cyan(f'{pose[0]:.1f}')} y={_cyan(f'{pose[1]:.1f}')} z={_cyan(f'{pose[2]:.1f}')}mm  "
                    f"rx={_magenta(f'{pose[3]:.2f}')} ry={_magenta(f'{pose[4]:.2f}')} rz={_magenta(f'{pose[5]:.2f}')}deg"
                )
                if legs:
                    leg_str = ", ".join(f"{l:.2f}" for l in legs)
                    lines.append(f"         legs: {_white(leg_str)}mm")
            else:
                all_joints = data["joints"]
                ee = data["eePosition"]
                ori = data.get("eeOrientation", [0, 0, 0])
                mode = data.get("mode", "?")
                err = data.get("ikError")

                if self._movable_joints:
                    j_parts = []
                    for (si, name), val in zip(self._movable_joints, all_joints):
                        j_parts.append(f"{name}={val:.1f}")
                    j_str = ", ".join(j_parts)
                else:
                    j_str = ", ".join(f"{a:7.1f}" for a in all_joints)

                mode_c = _bgreen(mode) if mode == "FK" else _bcyan(mode)
                lines.append(f"  {_bold('STATE')} | mode={mode_c}")
                lines.append(f"         joints: {_white(j_str)}")
                lines.append(
                    f"         ee=({_cyan(f'{ee[0]:.1f}')}, {_cyan(f'{ee[1]:.1f}')}, {_cyan(f'{ee[2]:.1f}')})mm  "
                    f"ori=({_magenta(f'{ori[0]:.1f}')}, {_magenta(f'{ori[1]:.1f}')}, {_magenta(f'{ori[2]:.1f}')})deg  "
                    f"err={_yellow(f'{err:.2f}mm') if err is not None else _dim('-')}"
                )
            coll_on = data.get("collisionEnabled", False)
            collisions = data.get("collisions", [])
            if coll_on:
                if collisions:
                    pairs = ", ".join(f"{_bred(c['link'])}<>{_bred(c['object'])}" for c in collisions)
                    lines.append(f"         collision: {_bred('YES')} [{pairs}]")
                else:
                    lines.append(f"         collision: {_green('none')}")

        elif msg_type == "hexapodFK":
            pose = data.get("pose", [0]*6)
            legs = data.get("legLengths", [])
            lines.append(f"  {_bold('HEXAPOD FK')} (pose → leg lengths)")
            lines.append(
                f"    pose: x={_cyan(f'{pose[0]:.1f}')} y={_cyan(f'{pose[1]:.1f}')} z={_cyan(f'{pose[2]:.1f}')}mm  "
                f"rx={_magenta(f'{pose[3]:.2f}')} ry={_magenta(f'{pose[4]:.2f}')} rz={_magenta(f'{pose[5]:.2f}')}deg"
            )
            leg_str = ", ".join(f"{l:.2f}" for l in legs)
            lines.append(f"    legs: {_white(leg_str)}mm")

        elif msg_type == "hexapodIK":
            pose = data.get("pose", [0]*6)
            legs = data.get("legLengths", [])
            lines.append(f"  {_bold('HEXAPOD IK')} (leg lengths → pose)")
            leg_str = ", ".join(f"{l:.2f}" for l in legs)
            lines.append(f"    legs: {_white(leg_str)}mm")
            lines.append(
                f"    pose: x={_cyan(f'{pose[0]:.1f}')} y={_cyan(f'{pose[1]:.1f}')} z={_cyan(f'{pose[2]:.1f}')}mm  "
                f"rx={_magenta(f'{pose[3]:.2f}')} ry={_magenta(f'{pose[4]:.2f}')} rz={_magenta(f'{pose[5]:.2f}')}deg"
            )

        elif msg_type == "legLengths":
            pose = data.get("platformPose", [0]*6)
            legs = data.get("legLengths", [])
            lines.append(f"  {_bold('LEG LENGTHS')}")
            lines.append(
                f"    pose: x={_cyan(f'{pose[0]:.1f}')} y={_cyan(f'{pose[1]:.1f}')} z={_cyan(f'{pose[2]:.1f}')}mm  "
                f"rx={_magenta(f'{pose[3]:.2f}')} ry={_magenta(f'{pose[4]:.2f}')} rz={_magenta(f'{pose[5]:.2f}')}deg"
            )
            for i, l in enumerate(legs):
                lines.append(f"    leg {i+1}: {_white(f'{l:.2f}')}mm")

        elif msg_type == "collisions":
            enabled = data.get("enabled", False)
            pairs = data.get("pairs", [])
            status = _bgreen("ON") if enabled else _dim("OFF")
            lines.append(f"  {_bold('COLLISION')} detection: {status}")
            if enabled:
                if pairs:
                    for p in pairs:
                        lines.append(f"    {_red(p['link'])} <> {_red(p['object'])}")
                else:
                    lines.append(f"    {_green('No collisions')}")

        elif msg_type == "objects":
            objs = data.get("objects", [])
            lines.append(f"  {_bold('OBJECTS')} ({len(objs)} imported)")
            for o in objs:
                p, r, s = o["position"], o["rotation"], o["scale"]
                vis = _green("visible") if o.get("visible", True) else _dim("hidden")
                idx = _dim(f"[{o['index']}]")
                lines.append(f"    {idx} {_bold(o['name'])}  {vis}")
                lines.append(
                    f"        pos=({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})mm  "
                    f"rot=({r[0]:.1f}, {r[1]:.1f}, {r[2]:.1f})deg  "
                    f"scale=({s[0]:.3f}, {s[1]:.3f}, {s[2]:.3f})"
                )

        elif msg_type == "object":
            o = data
            p, r, s = o["position"], o["rotation"], o["scale"]
            vis = _green("visible") if o.get("visible", True) else _dim("hidden")
            idx_str = o["index"]
            lines.append(f"  {_dim(f'[{idx_str}]')} {_bold(o['name'])}  {vis}")
            lines.append(f"    pos  = ({_cyan(f'{p[0]:.1f}')}, {_cyan(f'{p[1]:.1f}')}, {_cyan(f'{p[2]:.1f}')}) mm")
            lines.append(f"    rot  = ({_magenta(f'{r[0]:.1f}')}, {_magenta(f'{r[1]:.1f}')}, {_magenta(f'{r[2]:.1f}')}) deg")
            lines.append(f"    scale= ({s[0]:.3f}, {s[1]:.3f}, {s[2]:.3f})")

        elif msg_type == "devices":
            devs = data.get("devices", [])
            self._device_names_cache = [d["name"] for d in devs]
            lines.append(f"  {_bold('DEVICES')} ({len(devs)} loaded)")
            for d in devs:
                active = d.get("active", False)
                marker = _bgreen("*") if active else " "
                dname = _bold(d["name"]) if active else d["name"]
                cfg = _dim(d.get("config", "?"))
                kappa = f" {_dim('[kappa]')}" if d.get("isKappa") else ""
                lines.append(
                    f"    {marker} {dname}  {cfg}  "
                    f"{d.get('numJoints', '?')} joints  {d.get('mode', '?')}{kappa}"
                )

        elif msg_type == "device":
            d = data
            kappa = f" {_dim('[kappa]')}" if d.get("isKappa") else ""
            lines.append(
                f"  {_bold('DEVICE')} {_bold(d.get('name', '?'))}  "
                f"{_dim(d.get('config', '?'))}  "
                f"{d.get('numJoints', '?')} joints  mode={d.get('mode', '?')}{kappa}"
            )

        elif msg_type == "error" or "error" in data:
            lines.append(f"  {_bred('ERROR')}: {_red(data.get('error', 'unknown error'))}")

        return "\n".join(lines) if lines else None

    # ── Properties ───────────────────────────────────────────────────────

    @property
    def name(self):
        """Current device name."""
        return self._device_name

    @property
    def joint_names(self):
        """List of movable joint names."""
        return [name for _, name in self._movable_joints]

    @property
    def url(self):
        """WebSocket URL."""
        return self._url

    @property
    def connected(self):
        """Whether WebSocket is connected."""
        return self._ws is not None and self._ws.protocol is not None

    @property
    def quiet(self):
        """Whether background state printing is suppressed."""
        return not self._print_state

    @quiet.setter
    def quiet(self, value):
        self._print_state = not value

    # ── Axis name resolution ─────────────────────────────────────────────

    def _resolve_axis_name(self, name):
        nl = name.lower()
        for mi, (_, jname) in enumerate(self._movable_joints):
            if jname.lower() == nl:
                return mi
            if jname.lower().split()[0] == nl:
                return mi
        try:
            n = int(name)
            if 1 <= n <= self._n_movable:
                return n - 1
        except ValueError:
            pass
        return None

    def _expand_vector_device_names(self, axis_names):
        """Expand bare device names into per-joint axis names for array scans.

        Returns the expanded list, or None on error.
        """
        expanded = []
        for name in axis_names:
            if ':' in name or self._parse_virtual_axis(name) is not None:
                expanded.append(name)
                continue
            if self._resolve_axis_name(name) is not None:
                expanded.append(name)
                continue
            # Not a known axis — try as a device name
            if name == self._device_name:
                for _, jname in self._movable_joints:
                    expanded.append(jname.split()[0])
                continue
            dc = self._fetch_device_configs([name])
            if dc is not None:
                for _, jname in dc[name]["movable_joints"]:
                    expanded.append(f"{name}:{jname.split()[0]}")
                continue
            names = ", ".join(n for _, n in self._movable_joints)
            print(f"  {_yellow('Error')}: {name!r} is not a known axis or device. Available axes: {names}")
            return None
        return expanded

    def _angles_from_spec(self, spec):
        """Convert an angle spec (list or dict) to a flat list of floats."""
        if isinstance(spec, (list, tuple)):
            return [float(x) for x in spec]
        if isinstance(spec, dict):
            vals = [0.0] * self._n_movable
            for name, val in spec.items():
                idx = self._resolve_axis_name(str(name))
                if idx is None:
                    names = ", ".join(n for _, n in self._movable_joints)
                    raise ValueError(f"Unknown axis {name!r}. Available: {names}")
                vals[idx] = float(val)
            return vals
        raise TypeError(f"Expected list, tuple, or dict, got {type(spec).__name__}")

    # ── Device state update ──────────────────────────────────────────────

    def _update_device_state(self, new_config, new_config_path, new_name):
        self._config = new_config
        self._config_path = new_config_path
        self._device_name = new_name
        self._movable_joints = _get_movable_joints(new_config)
        self._n_movable = len(self._movable_joints)
        self._joint_name_to_idx = {name.lower(): idx for idx, name in self._movable_joints}

    # ═════════════════════════════════════════════════════════════════════
    #  PUBLIC API — State & Mode
    # ═════════════════════════════════════════════════════════════════════

    def state(self):
        """Request current device state. Prints it and returns the dict."""
        data = self._send_and_wait({"cmd": "getState"}, "state")
        if data:
            output = self._format_message(data)
            if output:
                print(output)
        return data

    def home(self):
        """Move all joints to 0 degrees."""
        self._send({"cmd": "home"})

    def fk(self):
        """Switch to Forward Kinematics mode."""
        self._send({"cmd": "setMode", "mode": "FK"})

    def ik(self):
        """Switch to Inverse Kinematics mode."""
        self._send({"cmd": "setMode", "mode": "IK"})

    # ═════════════════════════════════════════════════════════════════════
    #  PUBLIC API — Position Queries
    # ═════════════════════════════════════════════════════════════════════

    def _get_state_silent(self):
        """Fetch state without printing."""
        old = self._print_state
        self._print_state = False
        try:
            return self._send_and_wait({"cmd": "getState"}, "state")
        finally:
            self._print_state = old

    @property
    def pos(self):
        """Current end-effector position [x, y, z] in mm.

        Usage: robot.pos        -> [190.0, 0.0, 308.0]
               x, y, z = robot.pos
        """
        data = self._get_state_silent()
        return data["eePosition"] if data else None

    @property
    def ori(self):
        """Current end-effector orientation [a, b, g] in degrees (ZYX Euler).

        Usage: robot.ori        -> [0.0, 0.0, 0.0]
        """
        data = self._get_state_silent()
        return data.get("eeOrientation", [0, 0, 0]) if data else None

    @property
    def angles(self):
        """Current joint angles as a list of floats (degrees).

        Usage: robot.angles     -> [0.0, -30.0, 60.0, 0.0, 45.0, 90.0]
        """
        data = self._get_state_silent()
        return data["joints"] if data else None

    def get_joint(self, name):
        """Get the current angle of a single joint by name.

        Usage: robot.get_joint('J1')       -> 45.0
               robot.get_joint('J1 Base')  -> 45.0
        """
        data = self._get_state_silent()
        if not data:
            return None
        nl = name.lower()
        if nl not in self._joint_name_to_idx:
            matches = [(n, idx) for n, idx in self._joint_name_to_idx.items() if n.startswith(nl)]
            if len(matches) == 1:
                nl = matches[0][0]
            else:
                print(f"  {_yellow('Unknown joint')}: {name}")
                print(f"  Available: {', '.join(n for _, n in self._movable_joints)}")
                return None
        joint_idx = self._joint_name_to_idx[nl]
        return data["joints"][joint_idx]

    @property
    def mode(self):
        """Current mode ('FK' or 'IK')."""
        data = self._get_state_silent()
        return data.get("mode") if data else None

    def get_device_pos(self, name=None):
        """Get position/rotation/joints for any device.

        Returns dict with 'position', 'rotation', 'joints', 'eePosition',
        'eeOrientation', 'mode', etc.

        Usage: robot.get_device_pos()          # active device
               robot.get_device_pos('GP225')   # specific device
        """
        old = self._print_state
        self._print_state = False
        try:
            # Get device origin info
            msg = {"cmd": "getDevice"}
            if name:
                msg["device"] = name
            dev_data = self._send_and_wait(msg, "device", timeout=3.0)

            # Get EE position via state
            state_msg = {"cmd": "getState"}
            if name:
                state_msg["device"] = name
            state_data = self._send_and_wait(state_msg, "state", timeout=3.0)
        finally:
            self._print_state = old

        if not dev_data and not state_data:
            return None

        result = {}
        if dev_data:
            result["name"] = dev_data.get("name")
            result["position"] = dev_data.get("position")
            result["rotation"] = dev_data.get("rotation")
            result["worldPosition"] = dev_data.get("worldPosition")
            result["worldRotation"] = dev_data.get("worldRotation")
            result["joints"] = dev_data.get("joints")
            result["jointNames"] = dev_data.get("jointNames")
            result["mode"] = dev_data.get("mode")
            result["parent"] = dev_data.get("parent")
            result["links"] = dev_data.get("links")
        if state_data:
            result["eePosition"] = state_data.get("eePosition")
            result["eeOrientation"] = state_data.get("eeOrientation")
            result["joints"] = state_data.get("joints")
            result["mode"] = state_data.get("mode")
        return result

    def get_obj_pos(self, name_or_index):
        """Get position/rotation/scale for any scene object.

        Returns dict with 'position', 'rotation', 'scale', 'visible',
        'worldBB', etc.

        Usage: robot.get_obj_pos('cube_1')
               robot.get_obj_pos(0)
               robot.get_obj_pos('#0')
        """
        old = self._print_state
        self._print_state = False
        try:
            ref = _parse_obj_ref(name_or_index)
            data = self._send_and_wait({"cmd": "getObject", **ref}, "object", timeout=3.0)
        finally:
            self._print_state = old
        return data

    def dev_pose(self, device=None, space='world'):
        """Get a device's origin pose as (position, orientation).

        Parameters:
            device: device name (default: this client's device)
            space:  'world' (default) or 'local' — frame to report the
                    pose in. Local = relative to the device's parent.

        Returns:
            (position, orientation) with position in mm [x, y, z] and
            orientation in degrees [rx, ry, rz] (XYZ-intrinsic Euler).
            Returns (None, None) if the device can't be queried.

        Usage:
            p, o = robot.dev_pose()
            p, o = robot.dev_pose('GP225')
            p, o = robot.dev_pose('GP225', space='local')
        """
        if device is None:
            device = self._device_name
        info = self.get_device_pos(device)
        if not info:
            return None, None
        if space == 'local':
            return info.get('position'), info.get('rotation')
        pos = info.get('worldPosition') or info.get('position')
        rot = info.get('worldRotation') or info.get('rotation')
        return pos, rot

    def dev_pos(self, device=None, space='world'):
        """Device origin position [x, y, z] in mm. See ``dev_pose``."""
        return self.dev_pose(device, space)[0]

    def dev_ori(self, device=None, space='world'):
        """Device origin orientation [rx, ry, rz] in degrees. See ``dev_pose``."""
        return self.dev_pose(device, space)[1]

    def worldToLocal(self, pose_or_position=None, orientation=None, device=None):
        """Transform a world-frame pose into the device's local frame.

        Input uses the viewer convention (matching readback):
          positions [x, y, z] in mm, orientations [alpha, beta, gamma] degrees.
        Output uses the kinematic convention for setEulerTarget:
          positions [x, y, z] in mm, orientations [alpha, beta, gamma] degrees.

        Parameters:
            pose_or_position: [x, y, z, alpha, beta, gamma] (6-element), or
                              [x, y, z] (3-element, with orientation param)
            orientation:      [alpha, beta, gamma] in degrees, or None
            device:           device name (default: this client's device)

        Returns:
            [x, y, z, alpha, beta, gamma] for 6-element input, or
            (local_position, local_orientation) for separate args.
        """
        six_form = pose_or_position is not None and len(pose_or_position) == 6 and orientation is None
        if six_form:
            position = list(pose_or_position[:3])
            orientation = list(pose_or_position[3:])
        else:
            position = pose_or_position

        if device is None:
            device = self._device_name
        info = self.get_device_pos(device)
        if not info:
            print(f"  worldToLocal: no data returned for device '{device}'")
            return None if six_form else (None, None)
        dev_pos = info.get("worldPosition") or info.get("position")
        dev_rot = info.get("worldRotation") or info.get("rotation")
        if dev_pos is None or dev_rot is None:
            print(f"  worldToLocal: device '{device}' has no position/rotation (pos={dev_pos}, rot={dev_rot})")
            return None if six_form else (None, None)

        local_pos, local_ori = self._world_to_local_core(position, orientation, dev_pos, dev_rot)

        if six_form:
            return local_pos + local_ori
        return local_pos, local_ori

    @staticmethod
    def _world_to_local_core(position, orientation, dev_pos, dev_rot):
        """Pure-math world→local transform given a device's world pose.

        Shared by ``worldToLocal`` and the Cartesian world-frame scan so the
        device pose can be fetched once per scan rather than per waypoint.
        Returns (local_pos, local_ori); either may be None if its input was.
        """
        # Device rotation in Three.js Y-up: viewer [rx,ry,rz] → Three.js Euler (rx,rz,ry).
        R_dev_three = _rot_xyz_three(np.radians(dev_rot[0]),
                                     np.radians(dev_rot[2]),
                                     np.radians(dev_rot[1]))
        # Basis change to kinematic Z-up via W = Rx(90°): R_kin = W · R_three · W^T.
        _W = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)
        R_dev_kin = _W @ R_dev_three @ _W.T

        local_pos = None
        if position is not None:
            # Viewer → kinematic world coords: negate Y (viewer Y = -kin Y).
            p_kin = np.array([position[0], -position[1], position[2]], dtype=float)
            o_kin = np.array([dev_pos[0], -dev_pos[1], dev_pos[2]], dtype=float)
            local_pos = (R_dev_kin.T @ (p_kin - o_kin)).tolist()

        local_ori = None
        if orientation is not None:
            # pyEuler: em = Rx(α)·Ry(β)·Rz(γ).  Local em = R_dev_kin^T · em_world
            # (the Ry(±90°) factors in pyEuler↔quat cancel in the conjugation).
            em_world = _rot_xyz_three(np.radians(orientation[0]),
                                      np.radians(orientation[1]),
                                      np.radians(orientation[2]))
            em_local = R_dev_kin.T @ em_world
            ex, ey, ez = _euler_xyz_from_matrix(em_local)
            local_ori = [float(np.degrees(ex)), float(np.degrees(ey)), float(np.degrees(ez))]

        return local_pos, local_ori

    # ═════════════════════════════════════════════════════════════════════
    #  PUBLIC API — Joint Control
    # ═════════════════════════════════════════════════════════════════════

    def joints(self, *angles):
        """Set all movable joint angles in degrees.

        Usage: robot.joints(0, 30, 60, 0, 45, 90)
        """
        if len(angles) == 1 and isinstance(angles[0], (list, tuple)):
            angles = angles[0]
        if len(angles) != self._n_movable:
            names = ", ".join(f"<{n}>" for _, n in self._movable_joints)
            print(f"  {_yellow('Expected')} {_bold(str(self._n_movable))} values ({names}), got {len(angles)}")
            return
        self._send({"cmd": "setJoints", "angles": [float(a) for a in angles]})

    def joint(self, name, angle):
        """Set a single joint by name.

        Usage: robot.joint('J1', 45)
               robot.joint('J1 Base', 45)
        """
        nl = name.lower()
        if nl not in self._joint_name_to_idx:
            matches = [(n, idx) for n, idx in self._joint_name_to_idx.items() if n.startswith(nl)]
            if len(matches) == 1:
                nl = matches[0][0]
            else:
                print(f"  {_yellow('Unknown joint')}: {name}")
                print(f"  Available: {', '.join(n for _, n in self._movable_joints)}")
                return
        joint_idx = self._joint_name_to_idx[nl]
        self._send({"cmd": "setSingleJoint", "index": joint_idx, "angle": float(angle)})

    _VIRTUAL_AXES = ('chi', 'theta', 'phi')

    @staticmethod
    def _parse_virtual_axis(name):
        """If name is 'v:<axis>' (case-insensitive) with axis in {chi,theta,phi},
        return the canonical virtual axis name; else None."""
        if not isinstance(name, str):
            return None
        s = name.strip().lower()
        if s.startswith('v:') and s[2:] in RobotClient._VIRTUAL_AXES:
            return s[2:]
        return None

    # End-effector Cartesian axes: index into the pose vector
    # [x, y, z, alpha, beta, gamma] (mm, mm, mm, deg, deg, deg, ZYX Euler).
    _EE_AXES = ('x', 'y', 'z', 'a', 'b', 'g')

    # Active-arm name → GNKinematics solver (used for Cartesian 'ee:' scans).
    _EE_KIN_MAP = {
        'meca500':   Meca500_kin,
        'gp180_120': GP180_120_kin,
        'gp225':     GP225_kin,
        'gp280':     GP280_kin,
        'motomini':  MotoMini_kin,
    }

    @staticmethod
    def _parse_ee_axis(name):
        """If name is 'ee:<axis>' (case-insensitive) with axis in
        {x,y,z,a,b,g}, return its index 0..5 into the pose vector; else None."""
        if not isinstance(name, str):
            return None
        s = name.strip().lower()
        if s.startswith('ee:') and s[3:] in RobotClient._EE_AXES:
            return RobotClient._EE_AXES.index(s[3:])
        return None

    @staticmethod
    def _parse_ee_axis_spec(name):
        """Parse an EE axis with an optional device prefix.

        Accepts 'ee:<axis>' or '<Device>:ee:<axis>'. Returns (device, comp)
        where device is the device name (preserving case) or None for the
        active device, and comp is the pose index 0..5; or None if not an
        EE axis.
        """
        if not isinstance(name, str):
            return None
        s = name.strip()
        comp = RobotClient._parse_ee_axis(s)
        if comp is not None:
            return (None, comp)
        if ':' in s:
            dev, rest = s.split(':', 1)
            comp = RobotClient._parse_ee_axis(rest)
            if comp is not None:
                return (dev, comp)
        return None

    def _kinematics_for_device(self, name=None):
        """Return the GNKinematics solver for a device, or None (with a
        printed error) if no Python IK model is available for it."""
        dev = name or self._device_name
        key = dev.strip().lower()
        kin = self._EE_KIN_MAP.get(key)
        if kin is None:
            # Tolerate renamed/duplicated devices, e.g. 'GP225_2', 'Meca500.001'.
            for k, v in self._EE_KIN_MAP.items():
                if key.startswith(k):
                    kin = v
                    break
        if kin is None:
            known = ", ".join(sorted({'Meca500', 'GP180_120', 'GP225', 'GP280', 'MotoMini'}))
            print(f"  {_bred('Error')}: no Python IK model for device {dev!r}. "
                  f"Cartesian (ee:) scans require one of: {known}")
        return kin

    def _fetch_virtual_angles(self, device):
        """Return {'chi':.., 'theta':.., 'phi':..} for a kappa device, or None."""
        old = self._print_state
        self._print_state = False
        try:
            msg = {"cmd": "getVirtualAngles"}
            if device is not None:
                msg["device"] = str(device)
            resp = self._send_and_wait(msg, "virtualAngles", timeout=3.0)
        finally:
            self._print_state = old
        if not resp or 'chi' not in resp:
            return None
        return {
            'chi':   float(resp['chi']),
            'theta': float(resp['theta']),
            'phi':   float(resp['phi']),
        }

    def _fetch_device_joints(self, device):
        """Return (current_joints, joint_names) for a device, or (None, None)."""
        old = self._print_state
        self._print_state = False
        try:
            state = self._send_and_wait(
                {"cmd": "getState", "device": str(device)}, "state", timeout=3.0
            )
            dev = self._send_and_wait(
                {"cmd": "getDevice", "device": str(device)}, "device", timeout=3.0
            )
        finally:
            self._print_state = old
        if not state or "joints" not in state:
            return None, None
        current = [float(a) for a in state["joints"]]
        names = (dev or {}).get("jointNames") or []
        return current, list(names)

    @staticmethod
    def _resolve_axis(key, n, names):
        """Resolve a dict key to a joint index. Accepts int, or str (name/index)."""
        if isinstance(key, bool):
            raise ValueError(f"invalid axis key: {key!r}")
        if isinstance(key, int):
            idx = key
        elif isinstance(key, str):
            s = key.strip()
            if s.lstrip('-').isdigit():
                idx = int(s)
            else:
                lower = [str(nm).lower() for nm in names]
                sl = s.lower()
                if sl in lower:
                    idx = lower.index(sl)
                else:
                    matches = [i for i, nm in enumerate(lower) if nm.startswith(sl)]
                    if len(matches) == 1:
                        idx = matches[0]
                    else:
                        raise ValueError(
                            f"unknown joint {key!r}; available: {names}"
                        )
        else:
            raise ValueError(f"invalid axis key type: {type(key).__name__}")
        if idx < 0:
            idx += n
        if not (0 <= idx < n):
            raise ValueError(f"axis index {key!r} out of range (device has {n} joints)")
        return idx

    def set_pos(self, device, value):
        """Set joint angles (degrees) for a named device.

        Usage: robot.set_pos('meca500', [0, 0, 0, 0, 0, 0])
               robot.set_pos('meca500', np.zeros(6))
               robot.set_pos('meca500', my_pose_func)     # callable, called with no args
               robot.set_pos('meca500', my_pose_func())   # or pass result directly

        Partial update by axis (only named axes are changed):
               robot.set_pos('meca500', {2: 45})                 # axis index
               robot.set_pos('meca500', {'J4': 10, 'J6': -30})   # axis name

        Virtual axes (kappa diffractometers only), prefix with 'v:':
               robot.set_pos('i19', {'v:chi': 45})
               robot.set_pos('i19', {'v:chi': 45, 'v:theta': 30})
        """
        if callable(value):
            value = value()
        if isinstance(value, dict):
            virtual, physical = {}, {}
            for k, v in value.items():
                va = self._parse_virtual_axis(k)
                if va:
                    virtual[va] = v
                else:
                    physical[k] = v
            if virtual and physical:
                print(f"  {_yellow('Error')}: cannot mix virtual (v:) and physical axes "
                      "in one call; issue them separately")
                return
            if virtual:
                try:
                    msg = {"cmd": "setVirtualAngles", "device": str(device)}
                    for name, val in virtual.items():
                        msg[name] = float(val)
                except (ValueError, TypeError) as e:
                    print(f"  {_yellow('Error')}: {e}")
                    return
                self._send(msg)
                return
            current, names = self._fetch_device_joints(device)
            if current is None:
                print(f"  {_bred('Error')}: could not read current joints for device {device!r}")
                return
            new_angles = list(current)
            try:
                for k, v in physical.items():
                    idx = self._resolve_axis(k, len(current), names)
                    new_angles[idx] = float(v)
            except (ValueError, TypeError) as e:
                print(f"  {_yellow('Error')}: {e}")
                return
            self._send({"cmd": "setJoints", "device": str(device),
                        "angles": [round(a, 4) for a in new_angles]})
            return
        try:
            angles = [float(a) for a in value]
        except TypeError:
            print(f"  {_yellow('Error')}: value must be iterable, dict, or callable, got {type(value).__name__}")
            return
        self._send({"cmd": "setJoints", "device": str(device),
                    "angles": [round(a, 4) for a in angles]})

    def inc_pos(self, device, value):
        """Increment joint angles (degrees) for a named device, relative to current.

        Usage: robot.inc_pos('meca500', [0, 0, 0, 0, 0, 10])
               robot.inc_pos('meca500', np.array([1, -1, 0, 0, 0, 0]))
               robot.inc_pos('meca500', my_delta_func)   # callable, called with no args

        Partial increment by axis (only named axes are moved):
               robot.inc_pos('meca500', {5: 10})                 # axis index
               robot.inc_pos('meca500', {'J6': 10, 'J4': -5})    # axis name

        Virtual axes (kappa diffractometers only), prefix with 'v:':
               robot.inc_pos('i19', {'v:chi': 5})
               robot.inc_pos('i19', {'v:chi': 5, 'v:theta': 2})
        """
        if callable(value):
            value = value()
        if isinstance(value, dict):
            virtual, physical = {}, {}
            for k, v in value.items():
                va = self._parse_virtual_axis(k)
                if va:
                    virtual[va] = v
                else:
                    physical[k] = v
            if virtual and physical:
                print(f"  {_yellow('Error')}: cannot mix virtual (v:) and physical axes "
                      "in one call; issue them separately")
                return
            if virtual:
                current_v = self._fetch_virtual_angles(device)
                if current_v is None:
                    print(f"  {_bred('Error')}: could not read virtual angles "
                          f"(device {device!r} is not a kappa geometry?)")
                    return
                try:
                    msg = {"cmd": "setVirtualAngles", "device": str(device)}
                    for name, delta in virtual.items():
                        msg[name] = current_v[name] + float(delta)
                except (ValueError, TypeError) as e:
                    print(f"  {_yellow('Error')}: {e}")
                    return
                self._send(msg)
                return
            current, names = self._fetch_device_joints(device)
            if current is None:
                print(f"  {_bred('Error')}: could not read current joints for device {device!r}")
                return
            new_angles = list(current)
            try:
                for k, v in physical.items():
                    idx = self._resolve_axis(k, len(current), names)
                    new_angles[idx] = current[idx] + float(v)
            except (ValueError, TypeError) as e:
                print(f"  {_yellow('Error')}: {e}")
                return
            self._send({"cmd": "setJoints", "device": str(device),
                        "angles": [round(a, 4) for a in new_angles]})
            return
        current, names = self._fetch_device_joints(device)
        if current is None:
            print(f"  {_bred('Error')}: could not read current joints for device {device!r}")
            return
        try:
            deltas = [float(a) for a in value]
        except TypeError:
            print(f"  {_yellow('Error')}: value must be iterable, dict, or callable, got {type(value).__name__}")
            return
        if len(deltas) != len(current):
            print(f"  {_yellow('Error')}: expected {len(current)} values for "
                  f"{device!r}, got {len(deltas)}")
            return
        new_angles = [c + d for c, d in zip(current, deltas)]
        self._send({"cmd": "setJoints", "device": str(device),
                    "angles": [round(a, 4) for a in new_angles]})

    # ═════════════════════════════════════════════════════════════════════
    #  PUBLIC API — IK
    # ═════════════════════════════════════════════════════════════════════

    def move(self, x, y, z, a=0, b=0, g=0):
        """Switch to IK and move to position (mm) with orientation (degrees).

        Usage: robot.move(150, 100, 300)
               robot.move(150, 100, 300, 45, 0, 0)
        """
        self._send({"cmd": "moveTo",
                     "position": [float(x), float(y), float(z)],
                     "orientation": [float(a), float(b), float(g)]})

    def target(self, x, y, z, a=0, b=0, g=0):
        """Set IK target position/orientation without switching mode."""
        self._send({"cmd": "setIKTarget",
                     "position": [float(x), float(y), float(z)],
                     "orientation": [float(a), float(b), float(g)]})

    @staticmethod
    def _resolve_ee_key(key):
        """Map an EE-pose key to its index 0..5 in [x, y, z, a, b, g].

        Accepts an int index, a bare letter ('x'..'g'), or an 'ee:'-prefixed
        name ('ee:z'). Raises ValueError on anything else.
        """
        if isinstance(key, (int, np.integer)):
            if 0 <= int(key) < 6:
                return int(key)
            raise ValueError(f"axis index {key} out of range 0..5")
        s = str(key).strip().lower()
        if s.startswith('ee:'):
            s = s[3:]
        if s in RobotClient._EE_AXES:
            return RobotClient._EE_AXES.index(s)
        raise ValueError(f"unknown EE axis {key!r}; use x,y,z,a,b,g or 0..5")

    def _ik_goto(self, value, device, space, incremental):
        """Solve one Cartesian EE pose with the Python IK and stream setJoints.

        Shared core of eepos (absolute) and eeinc (incremental). Unspecified
        components hold the device's current pose. Returns the solved joint
        angles (degrees, list) or None on error / unreachable target.
        """
        if callable(value):
            value = value()
        base = self._cartesian_base(space, device=device)
        if base is None:
            return None
        kin, base_joints, base_pose, to_local = base
        pose = list(base_pose)
        if isinstance(value, dict):
            try:
                for k, v in value.items():
                    idx = self._resolve_ee_key(k)
                    pose[idx] = (pose[idx] + float(v)) if incremental else float(v)
            except (ValueError, TypeError) as e:
                print(f"  {_yellow('Error')}: {e}")
                return None
        else:
            try:
                vals = [float(a) for a in value]
            except TypeError:
                print(f"  {_yellow('Error')}: value must be iterable, dict, or "
                      f"callable, got {type(value).__name__}")
                return None
            if len(vals) > 6:
                print(f"  {_yellow('Error')}: pose has at most 6 components "
                      f"[x,y,z,a,b,g], got {len(vals)}")
                return None
            for i, v in enumerate(vals):
                pose[i] = (pose[i] + v) if incremental else v

        kin.storeCurrentPosition(np.array(base_joints, dtype=float))
        with np.errstate(invalid='ignore'):
            sol = np.asarray(kin.setEulerTarget(to_local(pose)), dtype=float)
        if np.any(np.isnan(sol)):
            dev = device or self._device_name
            frame = 'world' if space == 'world' else 'base'
            print(f"  {_bred('IK FAILED')} — no solution for {_bold(dev)} at "
                  f"{frame}-frame pose [x={pose[0]:.1f} y={pose[1]:.1f} z={pose[2]:.1f} "
                  f"a={pose[3]:.1f} b={pose[4]:.1f} g={pose[5]:.1f}]")
            print(f"  {_yellow('Robot did NOT move')} (target unreachable). Nothing sent.")
            return None
        msg = {"cmd": "setJoints", "angles": [round(a, 4) for a in sol.tolist()]}
        if device is not None:
            msg["device"] = str(device)
        self._send(msg)
        return sol.tolist()

    def eepos(self, value, device=None, space='local'):
        """Move an end-effector to an ABSOLUTE Cartesian pose via the Python IK.

        Solves the pose with GNKinematics (the same solver as the Cartesian
        scan) and streams the result as setJoints, so the on-screen pose matches
        the Python solution. Pose is [x, y, z, a, b, g] (mm, ZYX-Euler degrees);
        unspecified components hold the device's current pose. Mirrors set_pos.

        space='local' (default) = robot base frame; space='world' = viewer
        world frame (converted via worldToLocal). device=None = active device.

        Usage: robot.eepos([200, 0, 400, 0, 90, 0])
               robot.eepos({'z': 450})              # only Z, rest hold
               robot.eepos({'x': 300, 'a': 45})
               robot.eepos(my_pose_func)            # callable, called with no args
               robot.eepos([4334, 0, 500], device='GP180_120', space='world')

        Returns the solved joint angles (list, degrees) or None on error.
        """
        return self._ik_goto(value, device, space, incremental=False)

    def eeinc(self, value, device=None, space='local'):
        """Like eepos, but ADDS deltas to the device's current EE pose.

        Usage: robot.eeinc([0, 0, 50])              # +50 mm in Z
               robot.eeinc({'y': -100})             # jog Y by -100 mm
               robot.eeinc({'a': 10, 'z': 20})
               robot.eeinc([0, 0, 0, 0, 5, 0], device='GP180_120', space='world')

        Returns the solved joint angles (list, degrees) or None on error.
        """
        return self._ik_goto(value, device, space, incremental=True)

    # ═════════════════════════════════════════════════════════════════════
    #  PUBLIC API — Hexapod
    # ═════════════════════════════════════════════════════════════════════

    def platform(self, pose):
        """Set hexapod platform pose [x,y,z,rx,ry,rz] (mm, degrees).

        Usage: robot.platform([0, 0, 10, 0, 0, 0])
               robot.platform([5, 0, 0, 3, 0, 0])
        """
        if len(pose) != 6:
            print(f"  {_yellow('Expected 6-element pose')} [x,y,z,rx,ry,rz], got {len(pose)}")
            return
        self._send({"cmd": "setPlatformPose",
                     "pose": [float(v) for v in pose]})

    @property
    def platform_pose(self):
        """Current hexapod platform pose [x,y,z,rx,ry,rz] (mm, deg).

        Usage: robot.platform_pose  -> [0.0, 0.0, 10.0, 3.0, 0.0, 0.0]
        """
        data = self._get_state_silent()
        return data.get("platformPose") if data else None

    @property
    def leg_lengths(self):
        """Current hexapod leg lengths [l1..l6] in mm.

        Usage: robot.leg_lengths  -> [150.12, 150.12, 150.12, 150.12, 150.12, 150.12]
        """
        data = self._get_state_silent()
        return data.get("legLengths") if data else None

    def hexapod_fk(self, pose=None):
        """Forward kinematics: platform pose → leg lengths.

        If pose is omitted, uses the current platform pose.
        Returns dict with 'pose' and 'legLengths' (mm).

        Usage: robot.hexapod_fk()                         # current pose
               robot.hexapod_fk([5, 0, 10, 2, 0, 0])     # specific pose
        """
        msg = {"cmd": "hexapodFK"}
        if pose is not None:
            msg["pose"] = [float(v) for v in pose]
        data = self._send_and_wait(msg, "hexapodFK")
        if data:
            output = self._format_message(data)
            if output:
                print(output)
        return data

    def hexapod_ik(self, leg_lengths):
        """Inverse kinematics: leg lengths → platform pose.

        Returns dict with 'pose' and 'legLengths' (mm).

        Usage: robot.hexapod_ik([150.1, 150.1, 150.1, 150.1, 150.1, 150.1])
        """
        data = self._send_and_wait(
            {"cmd": "hexapodIK", "legLengths": [float(l) for l in leg_lengths]},
            "hexapodIK")
        if data:
            output = self._format_message(data)
            if output:
                print(output)
        return data

    def get_leg_lengths(self):
        """Get current hexapod leg lengths with per-leg detail.

        Usage: robot.get_leg_lengths()
        """
        data = self._send_and_wait({"cmd": "getLegLengths"}, "legLengths")
        if data:
            output = self._format_message(data)
            if output:
                print(output)
        return data

    def set_leg_lengths(self, *lengths):
        """Set hexapod pose by specifying desired leg lengths in mm.

        Solves IK to find the platform pose that produces the given lengths,
        then applies it.

        Usage: robot.set_leg_lengths(150.1, 150.1, 150.1, 150.1, 150.1, 150.1)
               robot.set_leg_lengths([150.1, 150.1, 150.1, 150.1, 150.1, 150.1])
        """
        if len(lengths) == 1 and isinstance(lengths[0], (list, tuple)):
            lengths = lengths[0]
        if len(lengths) != 6:
            print(f"  {_yellow('Expected 6 leg lengths')}, got {len(lengths)}")
            return
        self._send({"cmd": "setLegLengths",
                     "legLengths": [float(l) for l in lengths]})

    # ═════════════════════════════════════════════════════════════════════
    #  PUBLIC API — Collision
    # ═════════════════════════════════════════════════════════════════════

    def collision(self, enabled=None):
        """Toggle or set collision detection.

        Usage: robot.collision()       # toggle
               robot.collision(True)   # enable
               robot.collision(False)  # disable
        """
        msg = {"cmd": "setCollision"}
        if enabled is not None:
            msg["enabled"] = bool(enabled)
        self._send(msg)

    def collisions(self):
        """Get current collision pairs."""
        self._send({"cmd": "getCollisions"})

    # ═════════════════════════════════════════════════════════════════════
    #  PUBLIC API — Objects
    # ═════════════════════════════════════════════════════════════════════

    def objects(self):
        """List all imported mesh objects."""
        self._send({"cmd": "listObjects"})

    def obj(self, name_or_index):
        """Get details for an object by name or index.

        Usage: robot.obj('cube_1')  or  robot.obj(0)  or  robot.obj('#0')
        """
        ref = _parse_obj_ref(name_or_index)
        self._send({"cmd": "getObject", **ref})

    def objpos(self, name_or_index, x, y, z):
        """Set object position in mm."""
        ref = _parse_obj_ref(name_or_index)
        self._send({"cmd": "setObject", **ref, "position": [float(x), float(y), float(z)]})

    def objrot(self, name_or_index, rx, ry, rz):
        """Set object rotation in degrees."""
        ref = _parse_obj_ref(name_or_index)
        self._send({"cmd": "setObject", **ref, "rotation": [float(rx), float(ry), float(rz)]})

    def objscale(self, name_or_index, *args):
        """Set object scale. Single value for uniform, three for per-axis.

        Usage: robot.objscale('cube_1', 2.0)
               robot.objscale('cube_1', 1.0, 2.0, 0.5)
        """
        ref = _parse_obj_ref(name_or_index)
        if len(args) == 1:
            self._send({"cmd": "setObject", **ref, "scale": float(args[0])})
        elif len(args) >= 3:
            self._send({"cmd": "setObject", **ref, "scale": [float(a) for a in args[:3]]})
        else:
            print(f"  {_yellow('Usage')}: robot.objscale(name, s) or robot.objscale(name, sx, sy, sz)")

    def objvis(self, name_or_index, visible):
        """Show or hide an object.

        Usage: robot.objvis('cube_1', True)
               robot.objvis('cube_1', False)
        """
        ref = _parse_obj_ref(name_or_index)
        self._send({"cmd": "setObject", **ref, "visible": bool(visible)})

    def objresetrot(self, name_or_index):
        """Reset object rotation to (0, 0, 0)."""
        ref = _parse_obj_ref(name_or_index)
        self._send({"cmd": "resetObjectRotation", **ref})

    def objresetscale(self, name_or_index):
        """Reset object scale to (1, 1, 1)."""
        ref = _parse_obj_ref(name_or_index)
        self._send({"cmd": "resetObjectScale", **ref})

    def objtranslate(self, name_or_index, dx, dy, dz, space='parent'):
        """Translate object by (dx, dy, dz) mm in the given reference frame.

        space: 'parent' — parent object's axes (default)
               'local'  — object's own rotated axes
               'world'  — world/scene axes

        Usage: robot.objtranslate('Cube', 10, 0, 0)                # +10mm X in parent frame
               robot.objtranslate('Cube', 0, 0, 50, space='local') # +50mm Z along object's own Z
               robot.objtranslate('Cube', 0, 100, 0, space='world')
        """
        ref = _parse_obj_ref(name_or_index)
        self._send({"cmd": "translateObject", **ref, "delta": [float(dx), float(dy), float(dz)], "space": space})

    def objrotate(self, name_or_index, rx, ry, rz, space='parent'):
        """Rotate object by (rx, ry, rz) degrees in the given reference frame.

        space: 'parent' — parent object's axes (default)
               'local'  — object's own rotated axes
               'world'  — world/scene axes

        Usage: robot.objrotate('Cube', 0, 0, 45)                    # +45° around parent's Z
               robot.objrotate('Cube', 90, 0, 0, space='local')     # +90° around object's own X
               robot.objrotate('Cube', 0, 0, 30, space='world')
        """
        ref = _parse_obj_ref(name_or_index)
        self._send({"cmd": "rotateObject", **ref, "delta": [float(rx), float(ry), float(rz)], "space": space})

    def devtranslate(self, dx, dy, dz, space='parent', device=None):
        """Translate device origin by (dx, dy, dz) mm in the given reference frame.

        space: 'parent' — parent object's axes (default)
               'local'  — device's own rotated axes
               'world'  — world/scene axes

        Usage: robot.devtranslate(100, 0, 0)                         # +100mm X in parent frame
               robot.devtranslate(0, 0, 50, space='local')           # +50mm along device's own Z
               robot.devtranslate(0, 0, 200, space='world')
               robot.devtranslate(50, 0, 0, device='GP225')          # specific device
        """
        msg = {"cmd": "translateDevice", "delta": [float(dx), float(dy), float(dz)], "space": space}
        if device:
            msg["device"] = device
        self._send(msg)

    def devrotate(self, rx, ry, rz, space='parent', device=None):
        """Rotate device origin by (rx, ry, rz) degrees in the given reference frame.

        space: 'parent' — parent object's axes (default)
               'local'  — device's own rotated axes
               'world'  — world/scene axes

        Usage: robot.devrotate(0, 0, 45)                             # +45° around parent's Z
               robot.devrotate(90, 0, 0, space='local')              # +90° around device's own X
               robot.devrotate(0, 0, 30, space='world')
               robot.devrotate(0, 0, 90, device='GP225')             # specific device
        """
        msg = {"cmd": "rotateDevice", "delta": [float(rx), float(ry), float(rz)], "space": space}
        if device:
            msg["device"] = device
        self._send(msg)

    def devpose(self, position=None, rotation=None, device=None):
        """Set the device origin position and/or rotation (parent-local frame).

        position: [x, y, z] in mm, or [x, y, z, rx, ry, rz] for full pose
        rotation: [rx, ry, rz] in degrees

        Usage: r.devpose([100, 0, 0])                                # position only
               r.devpose([100, 0, 0], [0, 0, 90])                   # position + rotation
               r.devpose([100, 0, 0, 0, 0, 90])                     # full pose as one list
               r.devpose(rotation=[0, 0, 90])                       # rotation only
               r.devpose([0, 0, 0], device='GP225')                 # specific device
        """
        if position is not None and len(position) == 6:
            position, rotation = position[:3], position[3:]
        msg = {"cmd": "setDeviceOrigin"}
        if position is not None:
            msg["position"] = [float(v) for v in position]
        if rotation is not None:
            msg["rotation"] = [float(v) for v in rotation]
        if device:
            msg["device"] = device
        self._send(msg)

    # ═════════════════════════════════════════════════════════════════════
    #  PUBLIC API — Devices & Sessions
    # ═════════════════════════════════════════════════════════════════════

    def devices(self):
        """List all devices loaded in the viewer."""
        self._send({"cmd": "listDevices"})

    def device(self, name):
        """Switch active device by name. Updates prompt and joint names.

        Usage: robot.device('GP225')
        """
        # Switch active device and wait for state confirmation
        state_data = self._send_and_wait(
            {"cmd": "setActiveDevice", "device": name}, "state", timeout=2.0
        )
        if not state_data:
            print(f"  {_yellow('No response')} — is the device loaded?")
            return

        new_name = state_data.get("device", name)

        # Fetch device info to get config path
        dev_data = self._send_and_wait({"cmd": "getDevice"}, "device", timeout=2.0)
        new_config_path = dev_data.get("config") if dev_data else None

        if new_config_path:
            new_config = _load_config(new_config_path)
            if new_config:
                self._update_device_state(new_config, new_config_path, new_name)
                names = ", ".join(n for _, n in self._movable_joints)
                print(f"  {_bgreen('Switched to')} {_bold(new_name)}")
                print(f"  Joints ({self._n_movable}): {_dim(names)}")
            else:
                self._device_name = new_name
                print(f"  {_bgreen('Switched to')} {_bold(new_name)} "
                      f"{_yellow('(config not available locally)')}")
        else:
            self._device_name = new_name
            print(f"  {_bgreen('Switched to')} {_bold(new_name)}")

    def session(self, session_id=None):
        """Switch to a different viewer session by ID.

        Usage: robot.session('ab12cd34')
               robot.session()   # show current session
        """
        if session_id is None:
            print(f"  {_bold('Current session')}: {_bcyan(self._session or 'default')}")
            return

        session_id = str(session_id).strip()
        # Build new URL: strip any existing session param, then append the new one
        base_url = self._url.split("?")[0]
        if session_id:
            self._url = f"{base_url}?session={session_id}"
        else:
            self._url = base_url
        self._session = session_id or None

        # Reconnect with the new URL
        self.reconnect()

    def sessions(self):
        """List active viewer sessions."""
        http_url = self._url.replace("ws://", "http://").replace("wss://", "https://")
        http_url = http_url.split("?")[0]
        http_url = http_url.rsplit("/ws", 1)[0] + "/sessions"
        try:
            with urllib.request.urlopen(http_url, timeout=3) as resp:
                data = json.loads(resp.read())
            if not data:
                print(f"  {_dim('No active viewer sessions.')}")
                return []
            print(f"  {_bold('Active sessions')}:")
            for s in data:
                viewers_label = _dim(f"({s['viewers']} viewer{'s' if s['viewers'] != 1 else ''})")
                print(f"    {_bcyan(s['id'])}  {viewers_label}")
            return data
        except Exception as e:
            print(f"  {_red('Could not fetch sessions')}: {e}")
            return []

    # ═════════════════════════════════════════════════════════════════════
    #  PUBLIC API — Demo
    # ═════════════════════════════════════════════════════════════════════

    def demo(self):
        """Run the demo pose from the config, then return home."""
        is_kappa = self._config and any(
            j.get("name") == "kappa" for j in self._config.get("joints", [])
        )
        if not self._config or (not self._config.get("demoPose") and not is_kappa):
            print(f"  {_yellow('No demoPose defined in config')}")
            return

        poses = [
            ("Home position", {"cmd": "home"}),
            ("Demo pose", {"cmd": "demoPose"}),
        ]
        for i, (desc, cmd) in enumerate(poses, 1):
            print(f"  {_dim(f'[{i}/{len(poses)}]')} {_cyan(desc)}")
            self._send(cmd)
            time.sleep(1.5)
        print(f"  {_bgreen('Demo complete!')}")

    # ═════════════════════════════════════════════════════════════════════
    #  PUBLIC API — Path Planning
    # ═════════════════════════════════════════════════════════════════════

    def plan(self, start, end, stepsize=5.0, steptime=80):
        """Run RRT-Connect path planner and stream waypoints to the viewer.

        Args:
            start: list of angles or dict like {'J1': 30, 'mu': 45}
            end:   list of angles or dict like {'J1': 60, 'mu': 90}
            stepsize: RRT step size in degrees (default 5.0)
            steptime: delay between waypoints in ms (default 80)

        Usage:
            robot.plan([0,0,0,0,0,0], [30,-45,60,0,30,0])
            robot.plan({'J1': 0}, {'J1': 90, 'J2': -45})
        """
        try:
            from planner import RobotPlanner
        except ImportError:
            print(f"  {_bred('Error')}: planner.py not found")
            return

        start_vals = self._angles_from_spec(start)
        end_vals = self._angles_from_spec(end)

        if len(start_vals) != self._n_movable or len(end_vals) != self._n_movable:
            print(f"  {_yellow('Error')}: expected {self._n_movable} axis values, "
                  f"got start={len(start_vals)} end={len(end_vals)}")
            return

        # Fetch scene objects for collision obstacles
        objects_data = self._send_and_wait({"cmd": "listObjects"}, "objects", timeout=3.0)
        obj_list = objects_data.get("objects", []) if objects_data else []

        planner = RobotPlanner(self._config_path, step_deg=stepsize)
        n_obs = planner.sync_from_viewer_objects(obj_list)
        if n_obs:
            print(f"  {_dim(f'Using {n_obs} scene object(s) as collision obstacles')}")

        msg = f'Planning... (step size {stepsize}\u00b0)'
        print(f"  {_dim(msg)}")
        path = planner.plan(start_vals, end_vals)

        if path is None:
            print(f"  {_bred('No path found.')}")
            return

        dense = _densify_path(path, stepsize)
        detail = f'{steptime} ms/step, {stepsize}\u00b0 resolution'
        print(f"  {_bgreen('Executing')} {len(dense)} steps "
              f"({_dim(detail)})  "
              f"{_dim('Ctrl+C to stop')}")

        try:
            for wp in dense:
                self._send({"cmd": "setJoints", "angles": [round(a, 4) for a in wp]})
                time.sleep(steptime / 1000.0)
        except KeyboardInterrupt:
            print(f"\n  {_yellow('Motion stopped.')}")
        else:
            print(f"  {_bgreen('Done.')}")

    # ═════════════════════════════════════════════════════════════════════
    #  PUBLIC API — Scanning
    # ═════════════════════════════════════════════════════════════════════

    def scan(self, *axis_specs, steptime=80, space='local'):
        """Scan one or more axes (joints and/or object transforms).

        Each axis_spec is a tuple:
          (axis_name, start, end, step) — grid scan
          (axis_name, start, step)      — coupled scan (lockstep with primary)

        The first spec must always have 4 values (start, end, step).
        Supports Device:Axis syntax for multi-device scans.

        Object axes use '@' prefix — components: tx,ty,tz (mm), rx,ry,rz (deg):
            ('@Cube:tx', 0, 100, 10)   — scan object X translation
            ('@Cube:rz', 0, 360, 10)   — scan object Z rotation

        space parameter (for object axes):
            'local'  — parent-relative coordinates (default)
            'world'  — world coordinates

        Array scan: pass axis names (strings) followed by a callable or 2D array.
        A device name can be used in place of listing all its joints:
            robot.scan('GP180_120', waypoints)                   # full-vector array scan
            robot.scan('Meca500', 'GP225', waypoints)            # multi-device vector (N, 12)
            robot.scan('GP180_120', 'Meca500', pts)              # cols = 6+6 joints

        Virtual axes (kappa diffractometers) use a 'v:' prefix — chi/theta/phi:
            robot.scan(('v:chi', 0, 90, 5))                             # 1D virtual
            robot.scan(('v:chi', 0, 90, 5), ('v:theta', 0, 2))          # coupled
            robot.scan(('v:chi', 0, 45, 5), ('v:phi', 0, 30, 5))        # grid
            robot.scan('v:chi', 'v:theta', my_func)                     # array virtual

        Cartesian end-effector axes use an 'ee:' prefix — x,y,z (mm),
        a,b,g (ZYX Euler deg) in the robot base frame. Each target pose is
        solved to joints with the Python IK (GNKinematics), so the on-screen
        pose matches the analytical solution. The current EE pose is the scan
        origin; only the listed axes vary. Supported on Meca500, GP180_120,
        GP225, GP280 and MotoMini. Start/end/step are absolute coordinates;
        unlisted axes hold their current value. The space parameter chooses
        the frame: 'local' (default) = robot base frame; 'world' = world frame
        (converted per waypoint via worldToLocal, so the EE tracks world axes
        regardless of how the device is mounted/rotated):
            robot.scan(('ee:x', 150, 250, 10))                          # sweep EE X (base)
            robot.scan(('ee:z', 200, 400, 10))                          # sweep EE Z
            robot.scan(('ee:x', 150, 250, 5), ('ee:y', -50, 50, 5))     # grid in XY
            robot.scan(('ee:x', 150, 250, 5), ('ee:z', 300, 1))         # coupled
            robot.scan(('ee:z', 200, 400, 10), space='world')           # sweep world Z
            robot.scan('ee:x', 'ee:y', 'ee:z', waypoints)               # array of poses

        Prefix the axis with a device name to target a specific (non-active)
        device, and combine several devices in one scan — each is solved with
        its own IK (axes without a prefix use the active device). EE axes may
        also be combined with ordinary joint axes on other devices (one kind
        per device); object/virtual axes cannot be mixed in:
            robot.scan(('GP180_120:ee:z', 200, 400, 10))                       # one named device
            robot.scan(('GP180_120:ee:z', 200, 400, 10), ('Meca500:ee:x', 150, 250, 10))  # multi-device grid
            robot.scan(('GP180_120:ee:y', 354, 400, 10), ('I16_diff:delta', 0, 120, 10))   # Cartesian + joint

        Usage:
            robot.scan(('J1', 0, 90, 5))
            robot.scan(('J1', 0, 90, 5), ('J2', 0, 45, 5))             # grid
            robot.scan(('J1', 0, 90, 5), ('J2', 0, 2))                  # coupled
            robot.scan(('Meca500:J1', 0, 90, 5), ('GP225:J2', 0, 45, 5))  # multi-device
            robot.scan('J1', 'J2', my_func)                              # array scan
            robot.scan(('@Cube:tx', 0, 100, 10))                         # object scan
            robot.scan(('@Cube:tx', 0, 100, 10), space='world')          # world coords
            robot.scan(('J1', 0, 90, 5), ('@Cube:tz', 0, 50, 5))        # mixed
        """
        if not axis_specs:
            first = self._movable_joints[0][1].split()[0].lower() if self._movable_joints else "J1"
            second = self._movable_joints[1][1].split()[0].lower() if len(self._movable_joints) > 1 else "J2"
            ex1 = f"robot.scan(('{first}', 0, 50, 5))"
            ex2 = f"robot.scan(('{first}', 10, 20, 1), ('{second}', 20, 2))"
            ex3 = f"robot.scan(('{first}', 0, 20, 1), ('{second}', 0, 30, 2))"
            ex4 = f"robot.scan('{first}', '{second}', my_array_func)"
            ex5 = "robot.scan(('@Cube:tx', 0, 100, 10), space='world')"
            ex6 = f"robot.scan('{self._device_name}', my_array)"
            ex7 = "robot.scan(('ee:x', 0, 100, 10))"
            print(f"  {_yellow('Usage')}: robot.scan((axis, start, end, step), ...)")
            print(f"  1D:        {_dim(ex1)}")
            print(f"  Coupled:   {_dim(ex2)}")
            print(f"  Grid:      {_dim(ex3)}")
            print(f"  Array:     {_dim(ex4)}")
            print(f"  Vector:    {_dim(ex6)}")
            print(f"  Object:    {_dim(ex5)}")
            print(f"  Cartesian: {_dim(ex7)}")
            return

        # Detect array scan: last arg is callable or array-like, preceding args are strings
        last = axis_specs[-1]
        is_array_scan = (
            len(axis_specs) >= 2
            and (callable(last) or _is_array_like(last))
            and all(isinstance(a, str) for a in axis_specs[:-1])
        )

        if is_array_scan:
            axis_names = list(axis_specs[:-1])
            data = last() if callable(last) else last
            # Cartesian end-effector array scan (Python IK) via 'ee:' prefix
            ee_specs = [self._parse_ee_axis_spec(n) for n in axis_names]
            ee_flags = [e is not None for e in ee_specs]
            if any(ee_flags) and not all(ee_flags):
                print(f"  {_yellow('Error')}: cannot mix Cartesian (ee:) and other axes in one scan")
                return
            if all(ee_flags):
                if any(dev is not None for dev, _ in ee_specs):
                    print(f"  {_yellow('Error')}: device-prefixed EE axes (Device:ee:) are only "
                          f"supported in the range form (scan Dev:ee:x start end step). "
                          f"For an array, switch to the device first: robot.device('Dev')")
                    return
                self._scan_cartesian_array(axis_names, data, steptime, space)
                return
            axis_names = self._expand_vector_device_names(axis_names)
            if axis_names is None:
                return
            virt_flags = [self._parse_virtual_axis(n) is not None for n in axis_names]
            if any(virt_flags) and not all(virt_flags):
                print(f"  {_yellow('Error')}: cannot mix virtual (v:) and physical axes in one scan")
                return
            if all(virt_flags):
                self._scan_from_array_virtual(axis_names, data, steptime)
                return
            self._scan_from_array(axis_names, data, steptime)
            return

        if len(axis_specs[0]) != 4:
            print(f"  {_yellow('Error')}: first axis needs 4 values (axis, start, end, step)")
            return

        # Virtual-axis scan (chi/theta/phi on kappa device) via 'v:' prefix
        virt_flags = [self._parse_virtual_axis(str(s[0])) is not None for s in axis_specs]
        if any(virt_flags) and not all(virt_flags):
            print(f"  {_yellow('Error')}: cannot mix virtual (v:) and physical axes in one scan")
            return
        if all(virt_flags):
            self._scan_virtual_single(axis_specs, steptime)
            return

        # Cartesian end-effector scan (solved with the Python IK) via 'ee:' prefix,
        # optionally device-qualified as 'Device:ee:<axis>'. May be combined with
        # ordinary joint axes (but not object axes) for a mixed multi-device scan.
        ee_specs = [self._parse_ee_axis_spec(str(s[0])) for s in axis_specs]
        ee_flags = [e is not None for e in ee_specs]
        if any(ee_flags):
            if any(str(s[0]).startswith('@') for s in axis_specs):
                print(f"  {_yellow('Error')}: cannot mix Cartesian (ee:) and object (@) axes in one scan")
                return
            if all(ee_flags) and not any(dev is not None for dev, _ in ee_specs):
                self._scan_cartesian(axis_specs, steptime, space)
            else:
                # device-qualified ee:, and/or mixed with joint axes → combined
                self._scan_cartesian_multi(axis_specs, steptime, space)
            return

        # Separate joint axes and object axes
        has_objects = any(str(s[0]).startswith('@') for s in axis_specs)

        if has_objects:
            self._scan_with_objects(axis_specs, steptime, space)
            return

        # Check for multi-device syntax
        is_multi = any(":" in str(s[0]) for s in axis_specs)

        if is_multi:
            self._scan_multi_device(axis_specs, steptime)
        else:
            self._scan_single_device(axis_specs, steptime)

    def _scan_single_device(self, axis_specs, step_ms):
        """Run a single-device scan (grid or coupled)."""
        # Fetch current joint state
        state_data = self._send_and_wait({"cmd": "getState"}, "state", timeout=3.0)
        base = list(state_data["joints"]) if state_data else [0.0] * self._n_movable

        grid_axes = []
        coupled_axes = []
        for gi, spec in enumerate(axis_specs):
            axis_name = str(spec[0])
            nums = [float(x) for x in spec[1:]]
            axis_idx = self._resolve_axis_name(axis_name)
            if axis_idx is None:
                names = ", ".join(n for _, n in self._movable_joints)
                print(f"  {_yellow('Error')}: unknown axis {axis_name!r}. Available: {names}")
                return
            if gi == 0 or len(nums) == 3:
                if len(nums) != 3:
                    print(f"  {_yellow('Error')}: axis {axis_name!r} needs 3 values (start, end, step)")
                    return
                grid_axes.append((axis_idx, nums[0], nums[1], nums[2]))
            elif len(nums) == 2:
                coupled_axes.append((axis_idx, nums[0], nums[1]))
            else:
                print(f"  {_yellow('Error')}: axis {axis_name!r} needs 3 values "
                      f"(start, end, step) or 2 (start, step), got {len(nums)}")
                return

        # Build per-axis value ranges
        grid_ranges = []
        for axis_idx, start, end, step in grid_axes:
            if abs(step) < 1e-6:
                print(f"  {_bred('Error')}: step size must be > 0")
                return
            vals = _build_axis_vals(start, end, step)
            actual_joint_idx = self._movable_joints[axis_idx][0]
            grid_ranges.append((actual_joint_idx, vals))

        # Build waypoints
        if coupled_axes:
            primary_vals = grid_ranges[0][1]
            n_points = len(primary_vals)
            coupled_ranges = []
            for axis_idx, c_start, c_step in coupled_axes:
                actual_joint_idx = self._movable_joints[axis_idx][0]
                vals = [c_start + i * c_step for i in range(n_points)]
                coupled_ranges.append((actual_joint_idx, vals))
            all_ranges = grid_ranges + coupled_ranges
            waypoints = []
            for i in range(n_points):
                angles = list(base)
                for joint_idx, vals in all_ranges:
                    angles[joint_idx] = vals[i]
                waypoints.append(angles)
        else:
            all_val_lists = [vals for _, vals in grid_ranges]
            joint_indices = [idx for idx, _ in grid_ranges]
            waypoints = []
            for combo in itertools.product(*all_val_lists):
                angles = list(base)
                for joint_idx, val in zip(joint_indices, combo):
                    angles[joint_idx] = val
                waypoints.append(angles)

        # Print summary
        axis_descs = []
        for axis_idx, start, end, step in grid_axes:
            name = self._movable_joints[axis_idx][1]
            axis_descs.append(f"{_bold(name)}: {start} \u2192 {end} (step {abs(step)})")
        for axis_idx, c_start, c_step in coupled_axes:
            name = self._movable_joints[axis_idx][1]
            n_points = len(grid_ranges[0][1])
            c_end = c_start + (n_points - 1) * c_step
            axis_descs.append(f"{_bold(name)}: {c_start} \u2192 {c_end} (step {c_step})")
        mode_str = _dim("coupled") if coupled_axes else (_dim("grid") if len(grid_axes) > 1 else "")
        mode_suffix = f"  {mode_str}" if mode_str else ""
        print(f"  {_bgreen('Scanning')} {', '.join(axis_descs)}{mode_suffix}")
        print(f"  {len(waypoints)} points, {step_ms} ms/step  {_dim('Ctrl+C to stop')}")

        self._run_waypoints(waypoints, step_ms)

    def _scan_multi_device(self, axis_specs, step_ms):
        """Run a multi-device scan."""
        parsed = []
        for spec in axis_specs:
            axis_name = str(spec[0])
            nums = [float(x) for x in spec[1:]]
            if ":" in axis_name:
                dp, ap = axis_name.split(":", 1)
            else:
                dp, ap = self._device_name, axis_name
            parsed.append((dp, ap, nums))

        dev_names_used = list(dict.fromkeys(g[0] for g in parsed))
        dev_configs = self._fetch_device_configs(dev_names_used)
        if dev_configs is None:
            print(f"  {_bred('Error')}: could not load config for one or more devices.")
            return

        device_axes = []
        for gi, (dp, ap, nums) in enumerate(parsed):
            dc = dev_configs[dp]
            mi = _resolve_axis_for_device(ap, dc["movable_joints"])
            if mi is None:
                names = ", ".join(n for _, n in dc["movable_joints"])
                print(f"  {_yellow('Error')}: unknown axis {ap!r} on {dp}. Available: {names}")
                return
            jidx = dc["movable_joints"][mi][0]
            label = f"{dp}:{dc['movable_joints'][mi][1]}"
            if gi == 0 or len(nums) == 3:
                device_axes.append((dp, jidx, "grid", nums[0], nums[1], nums[2], label))
            elif len(nums) == 2:
                device_axes.append((dp, jidx, "coupled", nums[0], nums[1], label))
            else:
                print(f"  {_yellow('Error')}: axis {ap!r} needs 3 or 2 values, got {len(nums)}")
                return

        # Fetch current state for each device
        bases = {}
        for dname in dev_names_used:
            state_data = self._send_and_wait(
                {"cmd": "getState", "device": dname}, "state", timeout=3.0
            )
            n_movable = len(dev_configs[dname]["movable_joints"])
            bases[dname] = list(state_data["joints"]) if state_data else [0.0] * n_movable

        # Separate grid and coupled
        grid_ranges = []
        coupled_info = []
        for a in device_axes:
            dname, joint_idx, mode = a[0], a[1], a[2]
            if mode == "grid":
                start, end, step, label = a[3], a[4], a[5], a[6]
                if abs(step) < 1e-6:
                    print(f"  {_bred('Error')}: step size must be > 0")
                    return
                vals = _build_axis_vals(start, end, step)
                grid_ranges.append((dname, joint_idx, vals, label))
            else:
                coupled_info.append((dname, joint_idx, a[3], a[4], a[5]))

        # Build waypoints
        if coupled_info:
            primary_vals = grid_ranges[0][2]
            n_points = len(primary_vals)
            coupled_ranges = []
            for dname, joint_idx, c_start, c_step, _lbl in coupled_info:
                vals = [c_start + i * c_step for i in range(n_points)]
                coupled_ranges.append((dname, joint_idx, vals))
            all_ranges = [(d, j, v) for d, j, v, _ in grid_ranges] + coupled_ranges
            waypoints = []
            for i in range(n_points):
                wp = {d: list(bases[d]) for d in dev_names_used}
                for dname, joint_idx, vals in all_ranges:
                    wp[dname][joint_idx] = vals[i]
                waypoints.append(wp)
        else:
            all_val_lists = [v for _, _, v, _ in grid_ranges]
            waypoints = []
            for combo in itertools.product(*all_val_lists):
                wp = {d: list(bases[d]) for d in dev_names_used}
                for (dname, joint_idx, _, _), val in zip(grid_ranges, combo):
                    wp[dname][joint_idx] = val
                waypoints.append(wp)

        # Print summary
        axis_descs = []
        for a in device_axes:
            if a[2] == "grid":
                label, start, end, step = a[6], a[3], a[4], a[5]
                axis_descs.append(f"{_bold(label)}: {start} \u2192 {end} (step {abs(step)})")
            else:
                label, c_start, c_step = a[5], a[3], a[4]
                n_points = len(grid_ranges[0][2])
                c_end = c_start + (n_points - 1) * c_step
                axis_descs.append(f"{_bold(label)}: {c_start} \u2192 {c_end} (step {c_step})")
        mode_str = _dim("coupled") if coupled_info else (_dim("grid") if len(grid_ranges) > 1 else "")
        mode_suffix = f"  {mode_str}" if mode_str else ""
        print(f"  {_bgreen('Scanning')} {', '.join(axis_descs)}  {_dim('multi-device')}{mode_suffix}")
        print(f"  {len(waypoints)} points, {step_ms} ms/step  {_dim('Ctrl+C to stop')}")

        self._run_waypoints_multi(waypoints, step_ms)

    def _scan_from_array(self, axis_names, data, step_ms):
        """Run a scan from a 2D array of waypoint values.

        axis_names: list of axis name strings (may include Device:Axis syntax).
        data: 2D iterable — each row has one value per axis.
        """
        # Convert to list of lists
        try:
            rows = [list(row) for row in data]
        except TypeError:
            print(f"  {_bred('Error')}: array data must be a 2D iterable (list of lists, numpy array, etc.)")
            return

        if not rows:
            print(f"  {_yellow('Error')}: array is empty")
            return

        n_axes = len(axis_names)
        for ri, row in enumerate(rows):
            if len(row) != n_axes:
                print(f"  {_yellow('Error')}: row {ri} has {len(row)} values, expected {n_axes} (one per axis)")
                return

        is_multi = any(":" in name for name in axis_names)

        if is_multi:
            self._scan_from_array_multi(axis_names, rows, step_ms)
        else:
            self._scan_from_array_single(axis_names, rows, step_ms)

    def _scan_from_array_single(self, axis_names, rows, step_ms):
        """Array scan for a single device."""
        state_data = self._send_and_wait({"cmd": "getState"}, "state", timeout=3.0)
        base = list(state_data["joints"]) if state_data else [0.0] * self._n_movable

        # Resolve axis names to joint indices
        resolved = []
        for name in axis_names:
            idx = self._resolve_axis_name(name)
            if idx is None:
                names = ", ".join(n for _, n in self._movable_joints)
                print(f"  {_yellow('Error')}: unknown axis {name!r}. Available: {names}")
                return
            resolved.append((idx, self._movable_joints[idx][0], name))

        # Build waypoints
        waypoints = []
        for row in rows:
            angles = list(base)
            for (mi, joint_idx, _), val in zip(resolved, row):
                angles[joint_idx] = float(val)
            waypoints.append(angles)

        # Print summary
        labels = [_bold(self._movable_joints[mi][1]) for mi, _, _ in resolved]
        print(f"  {_bgreen('Array scan')} {', '.join(labels)}")
        print(f"  {len(waypoints)} points, {step_ms} ms/step  {_dim('Ctrl+C to stop')}")

        self._run_waypoints(waypoints, step_ms)

    def _scan_from_array_multi(self, axis_names, rows, step_ms):
        """Array scan across multiple devices."""
        # Parse Device:Axis pairs
        parsed = []
        for name in axis_names:
            if ":" in name:
                dp, ap = name.split(":", 1)
            else:
                dp, ap = self._device_name, name
            parsed.append((dp, ap))

        dev_names_used = list(dict.fromkeys(dp for dp, _ in parsed))
        dev_configs = self._fetch_device_configs(dev_names_used)
        if dev_configs is None:
            print(f"  {_bred('Error')}: could not load config for one or more devices.")
            return

        # Resolve each axis
        resolved = []
        for dp, ap in parsed:
            dc = dev_configs[dp]
            mi = _resolve_axis_for_device(ap, dc["movable_joints"])
            if mi is None:
                names = ", ".join(n for _, n in dc["movable_joints"])
                print(f"  {_yellow('Error')}: unknown axis {ap!r} on {dp}. Available: {names}")
                return
            jidx = dc["movable_joints"][mi][0]
            label = f"{dp}:{dc['movable_joints'][mi][1]}"
            resolved.append((dp, jidx, label))

        # Fetch current state for each device
        bases = {}
        for dname in dev_names_used:
            state_data = self._send_and_wait(
                {"cmd": "getState", "device": dname}, "state", timeout=3.0
            )
            n_movable = len(dev_configs[dname]["movable_joints"])
            bases[dname] = list(state_data["joints"]) if state_data else [0.0] * n_movable

        # Build waypoints
        waypoints = []
        for row in rows:
            wp = {d: list(bases[d]) for d in dev_names_used}
            for (dp, jidx, _), val in zip(resolved, row):
                wp[dp][jidx] = float(val)
            waypoints.append(wp)

        # Print summary
        labels = [_bold(lbl) for _, _, lbl in resolved]
        print(f"  {_bgreen('Array scan')} {', '.join(labels)}  {_dim('multi-device')}")
        print(f"  {len(waypoints)} points, {step_ms} ms/step  {_dim('Ctrl+C to stop')}")

        self._run_waypoints_multi(waypoints, step_ms)

    def _run_waypoints(self, waypoints, step_ms):
        """Stream single-device waypoints with Ctrl+C interrupt."""
        old_ps, self._print_state = self._print_state, False
        try:
            for wp in waypoints:
                self._send({"cmd": "setJoints", "angles": [round(a, 4) for a in wp]})
                time.sleep(step_ms / 1000.0)
        except KeyboardInterrupt:
            print(f"\n  {_yellow('Scan stopped.')}")
        else:
            print(f"  {_bgreen('Scan complete.')}")
        finally:
            self._print_state = old_ps

    def _run_waypoints_multi(self, waypoints, step_ms):
        """Stream multi-device waypoints with Ctrl+C interrupt."""
        old_ps, self._print_state = self._print_state, False
        try:
            for wp in waypoints:
                for dname, angles in wp.items():
                    self._send({"cmd": "setJoints", "device": dname,
                                "angles": [round(a, 4) for a in angles]})
                time.sleep(step_ms / 1000.0)
        except KeyboardInterrupt:
            print(f"\n  {_yellow('Scan stopped.')}")
        else:
            print(f"  {_bgreen('Scan complete.')}")
        finally:
            self._print_state = old_ps

    # ── Cartesian end-effector scans (Python IK) ─────────────────────────

    def _cartesian_base(self, space='local', device=None):
        """Return (kin, base_joints, base_pose, to_local) for a device, or
        None on error. device=None means the active device.

        base_pose is the current end-effector pose [x, y, z, alpha, beta,
        gamma] (mm, deg, ZYX Euler), the scan origin, expressed in:
          - 'local' (default): the robot base frame, from forward kinematics.
          - 'world':           the world frame, from the viewer's EE readback.
        to_local maps a pose in that frame to the local/kinematic pose that
        setEulerTarget expects — identity for 'local', the worldToLocal
        transform (device pose fetched once) for 'world'.
        """
        dev_name = device or self._device_name
        kin = self._kinematics_for_device(device)
        if kin is None:
            return None
        msg = {"cmd": "getState"}
        if device is not None:
            msg["device"] = device
        state_data = self._send_and_wait(msg, "state", timeout=3.0)
        if not state_data or "joints" not in state_data:
            print(f"  {_bred('Error')}: could not read joint state for {dev_name!r}")
            return None
        base_joints = [float(a) for a in state_data["joints"]]
        if len(base_joints) != 6:
            print(f"  {_yellow('Error')}: Cartesian IK scan needs a 6-axis arm "
                  f"({dev_name!r} has {len(base_joints)} joints)")
            return None

        if space == 'world':
            ee_pos = state_data.get("eePosition")
            ee_ori = state_data.get("eeOrientation")
            if ee_pos is None or ee_ori is None:
                print(f"  {_bred('Error')}: viewer did not report an EE world pose "
                      f"for {dev_name!r}")
                return None
            info = self.get_device_pos(dev_name)
            dev_pos = (info.get("worldPosition") or info.get("position")) if info else None
            dev_rot = (info.get("worldRotation") or info.get("rotation")) if info else None
            if dev_pos is None or dev_rot is None:
                print(f"  {_bred('Error')}: could not read world pose of device "
                      f"{dev_name!r}")
                return None
            base_pose = [float(v) for v in ee_pos] + [float(v) for v in ee_ori]

            def to_local(pose):
                lp, lo = self._world_to_local_core(pose[:3], pose[3:], dev_pos, dev_rot)
                return lp + lo
        else:
            fk = kin.f_kinematics(np.array(base_joints, dtype=float))
            base_pose = [float(v) for v in fk[-2]] + [float(np.degrees(v)) for v in fk[-1]]
            to_local = lambda pose: list(pose)

        return kin, base_joints, base_pose, to_local

    def _run_cartesian_poses(self, kin, base_joints, pose_waypoints, step_ms, to_local):
        """Solve a list of target poses to joint angles and stream them.

        Each pose is mapped to the local/kinematic frame with ``to_local``
        before solving. Joint continuity is preserved by seeding the solver's
        'minimum_movement' strategy with the previous reachable solution.
        Unreachable poses (no valid IK) hold the previous joint angles.
        """
        kin.storeCurrentPosition(np.array(base_joints, dtype=float))
        joint_waypoints = []
        last_valid = list(base_joints)
        n_fail = 0
        # The IK candidate search evaluates geometrically-invalid configs
        # (arccos of out-of-range values) before filtering — silence that noise.
        with np.errstate(invalid='ignore'):
            for pose in pose_waypoints:
                sol = np.asarray(kin.setEulerTarget(to_local(pose)), dtype=float)
                if np.any(np.isnan(sol)):
                    n_fail += 1
                    sol = np.array(last_valid, dtype=float)
                else:
                    last_valid = sol.tolist()
                    kin.storeCurrentPosition(sol)
                joint_waypoints.append(sol.tolist())

        if n_fail:
            n = len(pose_waypoints)
            print(f"  {_bred('IK FAILED')} on {_bold(f'{n_fail}/{n}')} target poses "
                  f"({100*n_fail//n}%) — no solution, held previous joints.")
            print(f"  {_yellow('The arm freezes at those points')} (looks like a stall/skip).")
        self._run_waypoints(joint_waypoints, step_ms)

    def _scan_cartesian(self, axis_specs, step_ms, space='local'):
        """Grid/coupled scan of end-effector Cartesian axes, solved in Python.

        EE axes use an 'ee:' prefix — x, y, z (mm) and a, b, g (ZYX Euler
        degrees). With space='local' (default) they are base-frame coords;
        with space='world' they are world coords, converted per waypoint via
        worldToLocal. Each pose is solved to joint angles via the analytical
        IK (GNKinematics) and streamed as setJoints, so the on-screen pose
        matches the Python solution.
        """
        info = self._cartesian_base(space)
        if info is None:
            return
        kin, base_joints, base_pose, to_local = info

        grid_axes, coupled_axes = [], []
        for gi, spec in enumerate(axis_specs):
            axis_name = str(spec[0])
            comp = self._parse_ee_axis(axis_name)
            try:
                nums = [float(x) for x in spec[1:]]
            except (TypeError, ValueError):
                print(f"  {_yellow('Error')}: axis {axis_name!r} has non-numeric values")
                return
            if gi == 0 or len(nums) == 3:
                if len(nums) != 3:
                    print(f"  {_yellow('Error')}: axis {axis_name!r} needs 3 values (start, end, step)")
                    return
                grid_axes.append((comp, nums[0], nums[1], nums[2]))
            elif len(nums) == 2:
                coupled_axes.append((comp, nums[0], nums[1]))
            else:
                print(f"  {_yellow('Error')}: axis {axis_name!r} needs 3 values "
                      f"(start, end, step) or 2 (start, step), got {len(nums)}")
                return

        grid_ranges = []
        for comp, start, end, step in grid_axes:
            if abs(step) < 1e-6:
                print(f"  {_bred('Error')}: step size must be > 0")
                return
            grid_ranges.append((comp, _build_axis_vals(start, end, step)))

        # Build target pose waypoints (each a full [x,y,z,a,b,g])
        pose_waypoints = []
        if coupled_axes:
            n_points = len(grid_ranges[0][1])
            coupled_ranges = [(c, [cs + i * cp for i in range(n_points)])
                              for c, cs, cp in coupled_axes]
            all_ranges = grid_ranges + coupled_ranges
            for i in range(n_points):
                pose = list(base_pose)
                for comp, vals in all_ranges:
                    pose[comp] = vals[i]
                pose_waypoints.append(pose)
        else:
            comps = [c for c, _ in grid_ranges]
            val_lists = [vals for _, vals in grid_ranges]
            for combo in itertools.product(*val_lists):
                pose = list(base_pose)
                for comp, val in zip(comps, combo):
                    pose[comp] = val
                pose_waypoints.append(pose)

        # Summary
        labels = []
        for comp, start, end, step in grid_axes:
            labels.append(f"{_bold('ee:' + self._EE_AXES[comp])}: {start} → {end} (step {abs(step)})")
        for comp, cs, cp in coupled_axes:
            n_points = len(grid_ranges[0][1])
            labels.append(f"{_bold('ee:' + self._EE_AXES[comp])}: {cs} → "
                          f"{cs + (n_points - 1) * cp} (step {cp})")
        mode_parts = []
        if coupled_axes:
            mode_parts.append("coupled")
        elif len(grid_axes) > 1:
            mode_parts.append("grid")
        mode_parts.append("world frame" if space == 'world' else "base frame")
        mode_suffix = f"  {_dim(', '.join(mode_parts))}"
        print(f"  {_bgreen('Cartesian IK scan')} {', '.join(labels)}{mode_suffix}")
        print(f"  {len(pose_waypoints)} points, {step_ms} ms/step  "
              f"{_dim('Python IK · Ctrl+C to stop')}")

        self._run_cartesian_poses(kin, base_joints, pose_waypoints, step_ms, to_local)

    def _scan_cartesian_array(self, axis_names, data, step_ms, space='local'):
        """Array scan of end-effector Cartesian axes, solved in Python.

        axis_names are 'ee:' axes; data is an (N, len(axis_names)) array of
        target values, each row overlaid on the current pose then solved.
        space='local' (default) treats values as base-frame coords;
        space='world' treats them as world coords (converted via worldToLocal).
        """
        info = self._cartesian_base(space)
        if info is None:
            return
        kin, base_joints, base_pose, to_local = info
        comps = [self._parse_ee_axis(n) for n in axis_names]

        arr = np.asarray(data, dtype=float)
        if arr.ndim == 1:
            arr = arr.reshape(-1, 1) if len(comps) == 1 else arr.reshape(1, -1)
        if arr.ndim != 2 or arr.shape[1] != len(comps):
            print(f"  {_yellow('Error')}: array must have {len(comps)} column(s) "
                  f"(one per axis), got shape {arr.shape}")
            return

        pose_waypoints = []
        for row in arr:
            pose = list(base_pose)
            for comp, val in zip(comps, row):
                pose[comp] = float(val)
            pose_waypoints.append(pose)

        names = ", ".join('ee:' + self._EE_AXES[c] for c in comps)
        frame = "world frame" if space == 'world' else "base frame"
        print(f"  {_bgreen('Cartesian IK scan')} {_bold(names)}  {_dim('array, ' + frame)}")
        print(f"  {len(pose_waypoints)} points, {step_ms} ms/step  "
              f"{_dim('Python IK · Ctrl+C to stop')}")
        self._run_cartesian_poses(kin, base_joints, pose_waypoints, step_ms, to_local)

    def _scan_cartesian_multi(self, axis_specs, step_ms, space='local'):
        """Multi-device scan mixing Cartesian 'Device:ee:<axis>' axes and
        ordinary joint axes ('Device:<joint>') in one command.

        Each Cartesian device is solved with its own Python IK; joint axes are
        set directly. The per-step joint vectors for every device are streamed
        together (one setJoints per device per step), mirroring the
        multi-device joint scan. Axes without a device prefix use the active
        device. A single device cannot mix ee: and joint axes.
        """
        # Parse each spec into a typed axis: ('ee'|'joint', device, target, nums)
        parsed = []
        for spec in axis_specs:
            axis_name = str(spec[0])
            try:
                nums = [float(x) for x in spec[1:]]
            except (TypeError, ValueError):
                print(f"  {_yellow('Error')}: axis {axis_name!r} has non-numeric values")
                return
            ee = self._parse_ee_axis_spec(axis_name)
            if ee is not None:
                dev, comp = ee
                parsed.append(('ee', dev or self._device_name, comp, nums, axis_name))
            else:
                if ':' in axis_name:
                    dev, ap = axis_name.split(':', 1)
                else:
                    dev, ap = self._device_name, axis_name
                parsed.append(('joint', dev, ap, nums, axis_name))

        # Classify devices; a device can't be both IK-solved and joint-driven
        dev_kind = {}
        for kind, dev, *_ in parsed:
            if dev_kind.setdefault(dev, kind) != kind:
                print(f"  {_yellow('Error')}: device {dev!r} mixes Cartesian (ee:) and "
                      f"joint axes — use one kind per device")
                return
        dev_names = list(dict.fromkeys(p[1] for p in parsed))
        arm_devs = [d for d in dev_names if dev_kind[d] == 'ee']
        joint_devs = [d for d in dev_names if dev_kind[d] == 'joint']

        # Cartesian bases (kin, base_joints, base_pose, to_local) per arm device
        bases = {}
        for d in arm_devs:
            info = self._cartesian_base(space, device=d)
            if info is None:
                return
            bases[d] = info

        # Configs + current joints for joint-driven devices
        jcfg, jbase = {}, {}
        if joint_devs:
            cfgs = self._fetch_device_configs(joint_devs)
            if cfgs is None:
                print(f"  {_bred('Error')}: could not load config for one or more devices.")
                return
            for d in joint_devs:
                jcfg[d] = cfgs[d]
                st = self._send_and_wait({"cmd": "getState", "device": d}, "state", timeout=3.0)
                n = len(cfgs[d]["movable_joints"])
                jbase[d] = list(st["joints"]) if st and st.get("joints") else [0.0] * n

        # Resolve each axis to (kind, device, target_index, label) + split grid/coupled
        grid_axes, coupled_axes = [], []   # entries: (kind, dev, idx, label, nums)
        for gi, (kind, dev, target, nums, axis_name) in enumerate(parsed):
            if kind == 'ee':
                idx, label = target, f"{dev}:ee:{self._EE_AXES[target]}"
            else:
                mi = _resolve_axis_for_device(target, jcfg[dev]["movable_joints"])
                if mi is None:
                    names = ", ".join(n for _, n in jcfg[dev]["movable_joints"])
                    print(f"  {_yellow('Error')}: unknown axis {target!r} on {dev}. Available: {names}")
                    return
                idx = jcfg[dev]["movable_joints"][mi][0]
                label = f"{dev}:{jcfg[dev]['movable_joints'][mi][1]}"
            if gi == 0 or len(nums) == 3:
                if len(nums) != 3:
                    print(f"  {_yellow('Error')}: axis {axis_name!r} needs 3 values (start, end, step)")
                    return
                if abs(nums[2]) < 1e-6:
                    print(f"  {_bred('Error')}: step size must be > 0")
                    return
                grid_axes.append((kind, dev, idx, label, _build_axis_vals(*nums), nums))
            elif len(nums) == 2:
                coupled_axes.append((kind, dev, idx, label, nums[0], nums[1]))
            else:
                print(f"  {_yellow('Error')}: axis {axis_name!r} needs 3 values "
                      f"(start, end, step) or 2 (start, step), got {len(nums)}")
                return

        def _blank_targets():
            t = {d: list(bases[d][2]) for d in arm_devs}        # poses
            t.update({d: list(jbase[d]) for d in joint_devs})   # joints
            return t

        # Build per-step targets (each: {device: pose-or-joints with overrides})
        target_steps = []
        if coupled_axes:
            n_points = len(grid_axes[0][4])
            seqs = [(d, i, vals) for _k, d, i, _l, vals, _n in grid_axes]
            seqs += [(d, i, [cs + n * cp for n in range(n_points)])
                     for _k, d, i, _l, cs, cp in coupled_axes]
            for n in range(n_points):
                t = _blank_targets()
                for d, i, vals in seqs:
                    t[d][i] = vals[n]
                target_steps.append(t)
        else:
            keys = [(d, i) for _k, d, i, _l, _v, _n in grid_axes]
            val_lists = [vals for _k, _d, _i, _l, vals, _n in grid_axes]
            for combo in itertools.product(*val_lists):
                t = _blank_targets()
                for (d, i), val in zip(keys, combo):
                    t[d][i] = val
                target_steps.append(t)

        # Summary
        labels = [f"{_bold(l)}: {nums[0]} → {nums[1]} (step {abs(nums[2])})"
                  for _k, _d, _i, l, _v, nums in grid_axes]
        for _k, _d, _i, l, cs, cp in coupled_axes:
            n_points = len(grid_axes[0][4])
            labels.append(f"{_bold(l)}: {cs} → {cs + (n_points - 1) * cp} (step {cp})")
        mode_parts = []
        if coupled_axes:
            mode_parts.append("coupled")
        elif len(grid_axes) > 1:
            mode_parts.append("grid")
        if arm_devs:
            mode_parts.append("world frame" if space == 'world' else "base frame")
        kind_note = "IK + joint" if (arm_devs and joint_devs) else ("Python IK" if arm_devs else "joint")
        print(f"  {_bgreen('Multi-device scan')} {', '.join(labels)}  {_dim(', '.join(mode_parts))}")
        print(f"  {len(target_steps)} points × {len(dev_names)} device(s), {step_ms} ms/step  "
              f"{_dim(kind_note + ' · Ctrl+C to stop')}")

        # Seed IK continuity per arm device
        for d in arm_devs:
            bases[d][0].storeCurrentPosition(np.array(bases[d][1], dtype=float))
        last_valid = {d: list(bases[d][1]) for d in arm_devs}
        fail_by_dev = {d: 0 for d in arm_devs}
        joint_steps = []
        with np.errstate(invalid='ignore'):
            for t in target_steps:
                wp = {}
                for d in arm_devs:
                    kin, _, _, to_local = bases[d]
                    sol = np.asarray(kin.setEulerTarget(to_local(t[d])), dtype=float)
                    if np.any(np.isnan(sol)):
                        fail_by_dev[d] += 1
                        sol = np.array(last_valid[d], dtype=float)
                    else:
                        last_valid[d] = sol.tolist()
                        kin.storeCurrentPosition(sol)
                    wp[d] = sol.tolist()
                for d in joint_devs:
                    wp[d] = list(t[d])
                joint_steps.append(wp)

        n = len(target_steps)
        if any(fail_by_dev.values()):
            for d, nf in fail_by_dev.items():
                if nf:
                    print(f"  {_bred('IK FAILED')} for {_bold(d)} on {_bold(f'{nf}/{n}')} "
                          f"target poses ({100*nf//n}%) — no solution, held previous joints.")
            print(f"  {_yellow('The arm freezes at those points')} (looks like a stall/skip).")
        self._run_waypoints_multi(joint_steps, step_ms)

    # ── Virtual-axis (kappa) scans ───────────────────────────────────────

    def _scan_virtual_single(self, axis_specs, step_ms):
        """Scan kappa virtual axes (chi/theta/phi) on the active device."""
        base = self._fetch_virtual_angles(None)
        if base is None:
            print(f"  {_bred('Error')}: active device is not a kappa geometry — "
                  "virtual axes unavailable")
            return

        grid_axes = []
        coupled_axes = []
        for gi, spec in enumerate(axis_specs):
            axis_name = str(spec[0])
            vname = self._parse_virtual_axis(axis_name)
            try:
                nums = [float(x) for x in spec[1:]]
            except (TypeError, ValueError):
                print(f"  {_yellow('Error')}: axis {axis_name!r} has non-numeric values")
                return
            if gi == 0 or len(nums) == 3:
                if len(nums) != 3:
                    print(f"  {_yellow('Error')}: axis {axis_name!r} needs 3 values (start, end, step)")
                    return
                grid_axes.append((vname, nums[0], nums[1], nums[2]))
            elif len(nums) == 2:
                coupled_axes.append((vname, nums[0], nums[1]))
            else:
                print(f"  {_yellow('Error')}: axis {axis_name!r} needs 3 values "
                      f"(start, end, step) or 2 (start, step), got {len(nums)}")
                return

        grid_ranges = []
        for vname, start, end, step in grid_axes:
            if abs(step) < 1e-6:
                print(f"  {_bred('Error')}: step size must be > 0")
                return
            grid_ranges.append((vname, _build_axis_vals(start, end, step)))

        if coupled_axes:
            primary_vals = grid_ranges[0][1]
            n_points = len(primary_vals)
            coupled_ranges = [
                (vn, [cs + i * cp for i in range(n_points)])
                for vn, cs, cp in coupled_axes
            ]
            all_ranges = grid_ranges + coupled_ranges
            waypoints = []
            for i in range(n_points):
                pt = {vn: vals[i] for vn, vals in all_ranges}
                waypoints.append(pt)
        else:
            names_list = [vn for vn, _ in grid_ranges]
            vals_list = [v for _, v in grid_ranges]
            waypoints = [
                {n: v for n, v in zip(names_list, combo)}
                for combo in itertools.product(*vals_list)
            ]

        axis_descs = []
        for vn, start, end, step in grid_axes:
            axis_descs.append(f"{_bold('v:' + vn)}: {start} \u2192 {end} (step {abs(step)})")
        if coupled_axes:
            n_points = len(grid_ranges[0][1])
            for vn, cs, cp in coupled_axes:
                ce = cs + (n_points - 1) * cp
                axis_descs.append(f"{_bold('v:' + vn)}: {cs} \u2192 {ce} (step {cp})")
        mode_str = _dim("coupled") if coupled_axes else (_dim("grid") if len(grid_axes) > 1 else "")
        mode_suffix = f"  {mode_str}" if mode_str else ""
        print(f"  {_bgreen('Scanning')} {', '.join(axis_descs)}  {_dim('virtual')}{mode_suffix}")
        print(f"  {len(waypoints)} points, {step_ms} ms/step  {_dim('Ctrl+C to stop')}")

        self._run_virtual_waypoints(waypoints, step_ms)

    def _scan_from_array_virtual(self, axis_names, data, step_ms):
        """Array scan over kappa virtual axes on the active device."""
        try:
            rows = [list(row) for row in data]
        except TypeError:
            print(f"  {_bred('Error')}: array data must be a 2D iterable")
            return
        if not rows:
            print(f"  {_yellow('Error')}: array is empty")
            return
        vnames = [self._parse_virtual_axis(n) for n in axis_names]
        n_axes = len(vnames)
        for ri, row in enumerate(rows):
            if len(row) != n_axes:
                print(f"  {_yellow('Error')}: row {ri} has {len(row)} values, expected {n_axes}")
                return

        base = self._fetch_virtual_angles(None)
        if base is None:
            print(f"  {_bred('Error')}: active device is not a kappa geometry — "
                  "virtual axes unavailable")
            return

        waypoints = []
        for row in rows:
            pt = {vn: float(val) for vn, val in zip(vnames, row)}
            waypoints.append(pt)

        labels = [_bold('v:' + vn) for vn in vnames]
        print(f"  {_bgreen('Array scan')} {', '.join(labels)}  {_dim('virtual')}")
        print(f"  {len(waypoints)} points, {step_ms} ms/step  {_dim('Ctrl+C to stop')}")
        self._run_virtual_waypoints(waypoints, step_ms)

    def _run_virtual_waypoints(self, waypoints, step_ms, device=None):
        """Stream virtual-angle waypoints with Ctrl+C interrupt."""
        old_ps, self._print_state = self._print_state, False
        try:
            for wp in waypoints:
                msg = {"cmd": "setVirtualAngles"}
                if device is not None:
                    msg["device"] = device
                for k, v in wp.items():
                    msg[k] = float(v)
                self._send(msg)
                time.sleep(step_ms / 1000.0)
        except KeyboardInterrupt:
            print(f"\n  {_yellow('Scan stopped.')}")
        else:
            print(f"  {_bgreen('Scan complete.')}")
        finally:
            self._print_state = old_ps

    def _run_commands(self, steps, step_ms):
        """Stream a list of command-lists with Ctrl+C interrupt."""
        old_ps, self._print_state = self._print_state, False
        try:
            for cmds in steps:
                for cmd in cmds:
                    self._send(cmd)
                time.sleep(step_ms / 1000.0)
        except KeyboardInterrupt:
            print(f"\n  {_yellow('Scan stopped.')}")
        else:
            print(f"  {_bgreen('Scan complete.')}")
        finally:
            self._print_state = old_ps

    # ── Object / mixed scanning ─────────────────────────────────────────

    _OBJ_COMPONENTS = {'tx', 'ty', 'tz', 'rx', 'ry', 'rz'}

    def _parse_obj_axis(self, raw_name):
        """Parse '@ObjName:tx' → (obj_name, component, axis_idx, is_translation) or None."""
        name = raw_name.lstrip('@')
        parts = name.split(':', 1)
        if len(parts) != 2:
            return None
        obj_name, comp = parts[0], parts[1].lower()
        if comp not in self._OBJ_COMPONENTS:
            return None
        return obj_name, comp, {'x': 0, 'y': 1, 'z': 2}[comp[1]], comp[0] == 't'

    def _scan_with_objects(self, axis_specs, step_ms, space):
        """Scan with object axes (possibly mixed with joint axes)."""
        # ── Parse all axes ──────────────────────────────────────────
        joint_axes = []   # (mode, axis_idx, start, end, step, label)
        obj_axes = []     # (mode, obj_name, comp, axis_idx, is_trans, start, end, step, label)
        for gi, spec in enumerate(axis_specs):
            raw = str(spec[0])
            nums = [float(x) for x in spec[1:]]
            is_grid = gi == 0 or len(nums) == 3

            if raw.startswith('@'):
                parsed = self._parse_obj_axis(raw)
                if not parsed:
                    print(f"  {_yellow('Error')}: invalid object axis {raw!r}. "
                          f"Format: @ObjectName:tx|ty|tz|rx|ry|rz")
                    return
                obj_name, comp, ai, is_trans = parsed
                unit = 'mm' if is_trans else '\u00b0'
                label = f"@{obj_name}:{comp}"
                if is_grid:
                    if len(nums) != 3:
                        print(f"  {_yellow('Error')}: {raw} needs (start, end, step)")
                        return
                    obj_axes.append(('grid', obj_name, comp, ai, is_trans,
                                     nums[0], nums[1], nums[2], label))
                else:
                    if len(nums) != 2:
                        print(f"  {_yellow('Error')}: {raw} needs (start, step) for coupled")
                        return
                    obj_axes.append(('coupled', obj_name, comp, ai, is_trans,
                                     nums[0], nums[1], None, label))
            else:
                axis_idx = self._resolve_axis_name(raw)
                if axis_idx is None:
                    names = ", ".join(n for _, n in self._movable_joints)
                    print(f"  {_yellow('Error')}: unknown joint {raw!r}. Available: {names}")
                    return
                jname = self._movable_joints[axis_idx][1]
                joint_idx = self._movable_joints[axis_idx][0]
                if is_grid:
                    if len(nums) != 3:
                        print(f"  {_yellow('Error')}: {raw} needs (start, end, step)")
                        return
                    joint_axes.append(('grid', joint_idx, nums[0], nums[1], nums[2], jname))
                else:
                    if len(nums) != 2:
                        print(f"  {_yellow('Error')}: {raw} needs (start, step) for coupled")
                        return
                    joint_axes.append(('coupled', joint_idx, nums[0], nums[1], None, jname))

        # ── Fetch base state ────────────────────────────────────────
        old_ps = self._print_state
        self._print_state = False
        try:
            base_joints = None
            if joint_axes:
                st = self._send_and_wait({"cmd": "getState"}, "state", timeout=3.0)
                base_joints = list(st["joints"]) if st else [0.0] * self._n_movable

            obj_states = {}
            for oa in obj_axes:
                oname = oa[1]
                if oname not in obj_states:
                    d = self._send_and_wait(
                        {"cmd": "getObject", "object": oname}, "object", timeout=3.0)
                    if not d:
                        print(f"  {_bred('Error')}: object '{oname}' not found")
                        return
                    obj_states[oname] = d
        finally:
            self._print_state = old_ps

        # ── Build value ranges ──────────────────────────────────────
        grid_ranges = []   # (kind, key, vals)   kind='joint'|'obj'
        coupled_info = []

        for ja in joint_axes:
            mode, jidx = ja[0], ja[1]
            if mode == 'grid':
                start, end, step = ja[2], ja[3], ja[4]
                if abs(step) < 1e-6:
                    print(f"  {_bred('Error')}: step size must be > 0")
                    return
                grid_ranges.append(('joint', jidx, _build_axis_vals(start, end, step)))
            else:
                coupled_info.append(('joint', jidx, ja[2], ja[3]))

        for oa in obj_axes:
            mode, oname, comp, ai, is_trans = oa[0], oa[1], oa[2], oa[3], oa[4]
            key = (oname, comp, ai, is_trans)
            if mode == 'grid':
                start, end, step = oa[5], oa[6], oa[7]
                if abs(step) < 1e-6:
                    print(f"  {_bred('Error')}: step size must be > 0")
                    return
                grid_ranges.append(('obj', key, _build_axis_vals(start, end, step)))
            else:
                coupled_info.append(('obj', key, oa[5], oa[6]))

        # ── Determine base object positions/rotations ───────────────
        pos_key = 'worldPosition' if space == 'world' else 'position'
        rot_key = 'worldRotation' if space == 'world' else 'rotation'
        base_pos = {n: list(d.get(pos_key, d['position'])) for n, d in obj_states.items()}
        base_rot = {n: list(d.get(rot_key, d['rotation'])) for n, d in obj_states.items()}

        # ── Build waypoints ─────────────────────────────────────────
        def _make_step(value_map):
            """Build command list from a dict mapping axis keys to values."""
            angles = list(base_joints) if base_joints is not None else None
            pos = {n: list(v) for n, v in base_pos.items()}
            rot = {n: list(v) for n, v in base_rot.items()}
            for key, val in value_map.items():
                if isinstance(key, int):
                    # joint index
                    angles[key] = val
                else:
                    oname, comp, ai, is_trans = key
                    if is_trans:
                        pos[oname][ai] = val
                    else:
                        rot[oname][ai] = val
            cmds = []
            if angles is not None:
                cmds.append({"cmd": "setJoints",
                             "angles": [round(a, 4) for a in angles]})
            for oname in obj_states:
                cmd = {"cmd": "setObject", "object": oname,
                       "position": [round(v, 2) for v in pos[oname]],
                       "rotation": [round(v, 2) for v in rot[oname]]}
                if space == 'world':
                    cmd["space"] = "world"
                cmds.append(cmd)
            return cmds

        # Unpack grid + coupled into value sequences
        all_grid_keys = []
        all_grid_vals = []
        for kind, key, vals in grid_ranges:
            k = key if kind == 'obj' else key   # int for joints, tuple for objects
            all_grid_keys.append(k)
            all_grid_vals.append(vals)

        all_coupled = []
        for kind, key, c_start, c_step in coupled_info:
            k = key if kind == 'obj' else key
            all_coupled.append((k, c_start, c_step))

        steps = []
        if all_coupled:
            primary_vals = all_grid_vals[0]
            n_points = len(primary_vals)
            for i in range(n_points):
                vm = {}
                for k, vals in zip(all_grid_keys, all_grid_vals):
                    vm[k] = vals[i]
                for k, c_start, c_step in all_coupled:
                    vm[k] = c_start + i * c_step
                steps.append(_make_step(vm))
        else:
            for combo in itertools.product(*all_grid_vals):
                vm = dict(zip(all_grid_keys, combo))
                steps.append(_make_step(vm))

        # ── Print summary ───────────────────────────────────────────
        descs = []
        for ja in joint_axes:
            name = ja[5]
            if ja[0] == 'grid':
                descs.append(f"{_bold(name)}: {ja[2]} \u2192 {ja[3]} (step {abs(ja[4])})\u00b0")
            else:
                n = len(all_grid_vals[0])
                c_end = ja[2] + (n - 1) * ja[3]
                descs.append(f"{_bold(name)}: {ja[2]} \u2192 {c_end} (step {ja[3]})\u00b0")
        for oa in obj_axes:
            label = oa[8]
            unit = 'mm' if oa[4] else '\u00b0'
            if oa[0] == 'grid':
                descs.append(f"{_bold(label)}: {oa[5]} \u2192 {oa[6]} (step {abs(oa[7])}){unit}")
            else:
                n = len(all_grid_vals[0])
                c_end = oa[5] + (n - 1) * oa[6]
                descs.append(f"{_bold(label)}: {oa[5]} \u2192 {c_end} (step {oa[6]}){unit}")
        space_str = f"  {_dim(f'[{space}]')}" if obj_axes else ""
        mode_str = ""
        if all_coupled:
            mode_str = f"  {_dim('coupled')}"
        elif len(grid_ranges) > 1:
            mode_str = f"  {_dim('grid')}"
        print(f"  {_bgreen('Scanning')} {', '.join(descs)}{space_str}{mode_str}")
        print(f"  {len(steps)} points, {step_ms} ms/step  {_dim('Ctrl+C to stop')}")

        self._run_commands(steps, step_ms)

    def _fetch_device_configs(self, device_names):
        """Fetch device list and load configs for named devices."""
        resp = self._send_and_wait({"cmd": "listDevices"}, "devices", timeout=3.0)
        if not resp:
            return None
        devs = resp.get("devices", [])
        result = {}
        for dname in device_names:
            dev = next((d for d in devs if d["name"] == dname), None)
            if not dev:
                return None
            cfg_path = dev.get("config")
            if not cfg_path:
                return None
            config = _load_config(cfg_path)
            if not config:
                return None
            result[dname] = {
                "config": config,
                "movable_joints": _get_movable_joints(config),
            }
        return result

    # ═════════════════════════════════════════════════════════════════════
    #  PUBLIC API — Kappa Virtual Angles
    # ═════════════════════════════════════════════════════════════════════

    def virtual_angles(self, chi=None, theta=None, phi=None):
        """Get or set kappa virtual angles.

        Usage: robot.virtual_angles()                    # get
               robot.virtual_angles(chi=45, theta=30)    # set
        """
        if chi is None and theta is None and phi is None:
            self._send({"cmd": "getVirtualAngles"})
        else:
            msg = {"cmd": "setVirtualAngles"}
            if chi is not None:
                msg["chi"] = float(chi)
            if theta is not None:
                msg["theta"] = float(theta)
            if phi is not None:
                msg["phi"] = float(phi)
            self._send(msg)

    def kappa_sign(self, positive):
        """Set kappa sign convention.

        Usage: robot.kappa_sign(True)   # positive
               robot.kappa_sign(False)  # negative
        """
        self._send({"cmd": "setKappaSign", "positive": bool(positive)})

    # ═════════════════════════════════════════════════════════════════════
    #  PUBLIC API — Visualization
    # ═════════════════════════════════════════════════════════════════════

    def labels(self, enabled=True):
        """Show or hide joint labels."""
        self._send({"cmd": "setLabels", "enabled": bool(enabled)})

    def origins(self, enabled=True):
        """Show or hide joint origin axes."""
        self._send({"cmd": "setOrigins", "enabled": bool(enabled)})

    def chain(self, enabled=True, device=None):
        """Show or hide kinematic chain visualization."""
        msg = {"cmd": "setChain", "enabled": bool(enabled)}
        if device:
            msg["device"] = device
        self._send(msg)

    def ortho(self, enabled=True):
        """Switch to orthographic (True) or perspective (False) camera."""
        self._send({"cmd": "setOrtho", "enabled": bool(enabled)})

    # ═════════════════════════════════════════════════════════════════════
    #  PUBLIC API — Camera
    # ═════════════════════════════════════════════════════════════════════

    def camera(self, position=None, target=None):
        """Get or set camera position/target.

        Usage: robot.camera()                                        # get
               robot.camera(position=[500, 500, 500])                # set position
               robot.camera(position=[500,500,500], target=[0,0,0])  # set both
        """
        if position is None and target is None:
            self._send({"cmd": "getCamera"})
        else:
            msg = {"cmd": "setCamera"}
            if position is not None:
                msg["position"] = [float(x) for x in position]
            if target is not None:
                msg["target"] = [float(x) for x in target]
            self._send(msg)

    def snap(self, view):
        """Snap camera to a preset view.

        Views: +X, -X, +Y, -Y, +Z, -Z, top, bottom, front, back, left, right, iso
        """
        self._send({"cmd": "snapCamera", "view": view})

    # ═════════════════════════════════════════════════════════════════════
    #  PUBLIC API — Scene
    # ═════════════════════════════════════════════════════════════════════

    def scene_state(self):
        """Get full scene state (devices, objects, camera)."""
        self._send({"cmd": "getSceneState"})

    def save_scene(self):
        """Trigger browser download of scene JSON."""
        self._send({"cmd": "saveScene"})

    # ═════════════════════════════════════════════════════════════════════
    #  Help
    # ═════════════════════════════════════════════════════════════════════

    def help(self):
        """Show available commands."""
        sections = [
            ("State & Mode", [
                ("robot.state()", "Get current device state"),
                ("robot.home()", "Move all joints to 0\u00b0"),
                ("robot.fk()", "Switch to Forward Kinematics"),
                ("robot.ik()", "Switch to Inverse Kinematics"),
                ("robot.demo()", "Run demo pose sequence"),
            ]),
            ("Joint Control", [
                ("robot.joints(j1, j2, ...)", "Set all movable joint angles (\u00b0)"),
                ("robot.joint('name', angle)", "Set a single joint by name"),
                ("robot.set_pos('dev', [j1, ...])", "Set joints on named device (list/array/callable)"),
                ("robot.set_pos('dev', {axis: v})", "Set only listed axes (axis = index or name)"),
                ("robot.inc_pos('dev', [d1, ...])", "Increment joints on named device (relative)"),
                ("robot.inc_pos('dev', {axis: d})", "Increment only listed axes (axis = index or name)"),
                ("pos <dev> <list|array|dict|callable>", "Magic: pos meca500 [0,0,0,0,0,0] or {2: 45}"),
                ("inc <dev> <list|array|dict|callable>", "Magic: inc meca500 [0,0,0,0,0,10] or {5: 10}"),
            ]),
            ("Inverse Kinematics", [
                ("robot.move(x, y, z [, a, b, g])", "IK move to position (mm) + orientation (\u00b0)"),
                ("robot.target(x, y, z [, a, b, g])", "Set IK target without switching mode"),
            ]),
            ("Hexapod (Stewart Platform)", [
                ("robot.platform([x,y,z,rx,ry,rz])", "Set platform pose (mm, \u00b0)"),
                ("robot.platform_pose", "Current platform pose [x,y,z,rx,ry,rz]"),
                ("robot.leg_lengths", "Current leg lengths [l1..l6] mm"),
                ("robot.hexapod_fk([pose])", "FK: pose \u2192 leg lengths"),
                ("robot.hexapod_ik([l1..l6])", "IK: leg lengths \u2192 pose"),
                ("robot.get_leg_lengths()", "Get current leg lengths (detailed)"),
                ("robot.set_leg_lengths(l1..l6)", "Set pose via leg lengths (mm)"),
            ]),
            ("Collision", [
                ("robot.collision([True|False])", "Toggle or set collision detection"),
                ("robot.collisions()", "Get current collision pairs"),
            ]),
            ("Objects", [
                ("robot.objects()", "List imported mesh objects"),
                ("robot.obj(name_or_idx)", "Get object details"),
                ("robot.objpos(name, x, y, z)", "Set object position (mm)"),
                ("robot.objrot(name, rx, ry, rz)", "Set object rotation (\u00b0)"),
                ("robot.objscale(name, s)", "Set uniform or per-axis scale"),
                ("robot.objvis(name, True|False)", "Show/hide object"),
                ("robot.objtranslate(name, dx,dy,dz, sp)", "Translate in parent|local|world frame"),
                ("robot.objrotate(name, rx,ry,rz, sp)", "Rotate in parent|local|world frame"),
            ]),
            ("Devices & Sessions", [
                ("robot.devices()", "List all loaded devices"),
                ("robot.device('name')", "Switch active device"),
                ("robot.devtranslate(dx, dy, dz, sp)", "Translate device in parent|local|world"),
                ("robot.devrotate(rx, ry, rz, sp)", "Rotate device in parent|local|world"),
                ("robot.sessions()", "List active viewer sessions"),
                ("robot.session('id')", "Switch to a viewer session"),
            ]),
            ("Scanning", [
                ("robot.scan(('J1', 0, 90, 5))", "1D scan"),
                ("robot.scan((...), (...))", "Grid or coupled scan"),
                ("robot.scan(('Dev:J1', ...))", "Multi-device scan"),
                ("robot.scan('J1', 'J2', func)", "Array scan (func/array)"),
                ("robot.scan(('v:chi', 0, 90, 5))", "Kappa virtual-axis scan"),
                ("robot.scan(('ee:x', 0, 100, 10))", "Cartesian EE scan (Python IK)"),
                ("scan ee:x 150 250 10", "Magic: space-separated EE scan"),
                ("scan ee:z 200 400 10 --space world", "Magic: EE scan in world frame"),
                ("scan GP180_120:ee:z 200 400 10", "Magic: EE scan on a named device"),
                ("scan Dev1:ee:x 0 50 5 Dev2:ee:y 0 50 5", "Magic: multi-device EE scan"),
                ("scan Arm:ee:y 0 100 5 Kappa:delta 0 90 5", "Magic: mixed Cartesian + joint scan"),
                ("robot.scan(('@Cube:tx', 0, 100, 10))", "Object transform scan"),
                ("  space='local'|'world'", "Frame for EE (ee:) and object scans"),
            ]),
            ("Path Planning", [
                ("robot.plan(start, end)", "RRT-Connect path planner"),
                ("  start/end: list or dict", "e.g. [0,0,0,0,0,0] or {'J1': 30}"),
            ]),
            ("Visualization", [
                ("robot.labels([True])", "Show/hide joint labels"),
                ("robot.origins([True])", "Show/hide joint origin axes"),
                ("robot.chain([True])", "Show/hide kinematic chain"),
                ("robot.ortho([True])", "Orthographic/perspective camera"),
                ("robot.camera()", "Get/set camera position"),
                ("robot.snap('iso')", "Snap to preset view"),
            ]),
            ("Kappa (Diffractometers)", [
                ("robot.virtual_angles(chi=, theta=, phi=)", "Get/set virtual angles"),
                ("robot.set_pos('dev', {'v:chi': 45})", "Partial set of virtual angles"),
                ("robot.inc_pos('dev', {'v:chi': 5})", "Increment virtual angles"),
                ("robot.scan(('v:chi', 0, 90, 5))", "Scan virtual axes"),
                ("robot.kappa_sign(True|False)", "Set kappa sign convention"),
            ]),
            ("Position Queries", [
                ("robot.pos", "Active device EE position [x, y, z] mm"),
                ("robot.ori", "Active device EE orientation [a, b, g] deg"),
                ("robot.angles", "Active device joint angles (list)"),
                ("robot.get_joint('J1')", "Single joint angle by name"),
                ("robot.mode", "Current mode ('FK' or 'IK')"),
                ("robot.get_device_pos(['name'])", "Any device: pos, rot, joints, EE"),
                ("robot.get_obj_pos(name_or_idx)", "Any object: pos, rot, scale, BB"),
                ("robot.dev_pose([name], space=)", "Device origin (pos, ori), world|local"),
                ("robot.dev_pos([name]) / dev_ori", "Device origin position / orientation"),
                ("robot.worldToLocal(pos, ori)", "World \u2192 device-local pose transform"),
            ]),
            ("Properties", [
                ("robot.name", "Current device name"),
                ("robot.joint_names", "List of movable joint names"),
                ("robot.connected", "Connection status"),
                ("robot.quiet = True", "Suppress background state output"),
            ]),
            ("Connection", [
                ("robot.reconnect()", "Reconnect to server"),
                ("robot.disconnect()", "Close connection"),
            ]),
        ]

        print()
        for heading, cmds in sections:
            print(f"  {_byellow(heading)}")
            for sig, desc in cmds:
                print(f"    {_bold(sig):<45s} {_dim(desc)}")
            print()

    def __repr__(self):
        status = _bgreen("connected") if self.connected else _red("disconnected")
        return f"RobotClient({self._device_name}, {status})"


# ═══════════════════════════════════════════════════════════════════════════════
#  IPython Integration
# ═══════════════════════════════════════════════════════════════════════════════

def _build_banner(robot):
    """Build the startup banner string."""
    name = "DLS Collision Model"
    lines = []
    if _COLORS:
        lines.append(_cyan(f"\n  \u2554{'=' * 42}\u2557"))
        lines.append(_cyan(f"  \u2551") + _bold(f"  {name:^38s}") + _cyan(f"  \u2551"))
        lines.append(_cyan(f"  \u255a{'=' * 42}\u255d"))
    else:
        lines.append(f"\n  {name}")
        lines.append(f"  {'=' * len(name)}")

    lines.append(f"\n  Remote Control Terminal v3.0 (IPython)")
    lines.append(f"  Server: {_bold(robot.url) if _COLORS else robot.url}")

    if robot._movable_joints:
        names = ", ".join(name for _, name in robot._movable_joints)
        lines.append(f"  Joints ({robot._n_movable}): {_dim(names)}")

    lines.append(f"\n  Python: {_bold('robot.<Tab>')}  |  Space-separated: {_bold('home')}, {_bold('joints 0 30 60 0 45 90')}, ...")
    lines.append(f"  Type {_bold('robot.help()')} or {_bold('rhelp')} for full docs.")
    lines.append("")
    return "\n".join(lines)


from IPython.terminal.prompts import Prompts


class _RobotPrompts(Prompts):
    """Custom IPython prompts showing the device name."""

    def __init__(self, shell):
        super().__init__(shell)

    def in_prompt_tokens(self):
        from pygments.token import Token
        robot = self.shell.user_ns.get("robot")
        name = robot.name.lower().replace(" ", "_") if robot else "robot"
        return [
            (Token.Prompt, f"{name} "),
            (Token.Prompt, "["),
            (Token.PromptNum, str(self.shell.execution_count)),
            (Token.Prompt, "]: "),
        ]

    def out_prompt_tokens(self):
        from pygments.token import Token
        return [
            (Token.OutPrompt, "Out"),
            (Token.OutPromptNum, f"[{self.shell.execution_count}]"),
            (Token.OutPrompt, ": "),
        ]

    def continuation_prompt_tokens(self, width=None):
        from pygments.token import Token
        if width is None:
            width = self._width()
        spaces = " " * (width - 5)
        return [
            (Token.Prompt, f"{spaces}...: "),
        ]

    def rewrite_prompt_tokens(self):
        from pygments.token import Token
        return [
            (Token.Prompt, ""),
        ]

    def _width(self):
        robot = self.shell.user_ns.get("robot")
        name = robot.name.lower().replace(" ", "_") if robot else "robot"
        count_str = str(self.shell.execution_count)
        return len(name) + 1 + len(count_str) + 4  # "name [N]: "


# ── Line magics (space-separated syntax) ─────────────────────────────────────

def _register_magics(ipython, robot):
    """Register line magics so the original space-separated syntax works.

    With automagic (on by default), the % prefix is optional:
        joints 0 30 60 0 45 90
        joint J1 45
        pos meca500 [0, 0, 0, 0, 0, 0]
        pos meca500 np.zeros(6)
        inc meca500 [0, 0, 0, 0, 0, 10]
        move 150 100 300
        scan J1 0 90 5
        scan J1 J2 polar_func()
    """
    reg = ipython.register_magic_function

    # ── Simple commands ──────────────────────────────────────────────

    def _m_state(line):
        robot.state()
    reg(_m_state, magic_name='state')

    def _m_home(line):
        robot.home()
    reg(_m_home, magic_name='home')

    def _m_fk(line):
        robot.fk()
    reg(_m_fk, magic_name='fk')

    def _m_ik(line):
        robot.ik()
    reg(_m_ik, magic_name='ik')

    def _m_demo(line):
        robot.demo()
    reg(_m_demo, magic_name='demo')

    # ── Joint commands ───────────────────────────────────────────────

    def _m_joints(line):
        """joints 0 30 60 0 45 90"""
        parts = line.split()
        if not parts:
            robot.state()
            return
        robot.joints(*[float(x) for x in parts])
    reg(_m_joints, magic_name='joints')

    def _m_joint(line):
        """joint J1 45"""
        parts = line.split()
        if len(parts) < 2:
            print(f"  {_yellow('Usage')}: joint <name> <angle>")
            return
        robot.joint(parts[0], float(parts[1]))
    reg(_m_joint, magic_name='joint')

    def _m_pos(line):
        """pos <device> <value>

        value is a Python expression evaluated in the IPython namespace:
        a list/array/callable for the full joint vector, OR a dict
        {axis: angle} to set individual axes (axis is index or joint name).

        Examples:
            pos meca500 [0, 0, 0, 0, 0, 0]
            pos meca500 np.zeros(6)
            pos meca500 my_pose_func()
            pos meca500 my_pose_func        # callable; invoked with no args
            pos meca500 {2: 45}             # set only axis index 2
            pos meca500 {'J4': 10, 'J6': -30}
        """
        stripped = line.strip()
        if not stripped:
            print(f"  {_yellow('Usage')}: pos <device> <list|array|dict|callable>")
            print(f"  Example: {_bold('pos meca500 [0,0,0,0,0,0]')}")
            print(f"  Example: {_bold('pos meca500 {2: 45}')}  # single axis")
            return
        parts = stripped.split(None, 1)
        if len(parts) < 2:
            print(f"  {_yellow('Usage')}: pos <device> <list|array|dict|callable>")
            return
        device, expr = parts[0], parts[1]
        from IPython import get_ipython
        ip = get_ipython()
        try:
            value = ip.ev(expr)
        except Exception as e:
            print(f"  {_bred('Error')}: failed to evaluate {expr!r}: {e}")
            return
        robot.set_pos(device, value)
    reg(_m_pos, magic_name='pos')

    def _m_inc(line):
        """inc <device> <value>

        Like pos, but adds the given deltas to the device's current joint angles.
        Accepts a list/array/callable for all axes, or a dict {axis: delta}
        to increment only individual axes (axis is index or joint name).

        Examples:
            inc meca500 [0, 0, 0, 0, 0, 10]
            inc meca500 np.array([1, -1, 0, 0, 0, 0])
            inc meca500 my_delta_func()
            inc meca500 {5: 10}              # move only axis index 5
            inc meca500 {'J6': 10, 'J4': -5}
        """
        stripped = line.strip()
        if not stripped:
            print(f"  {_yellow('Usage')}: inc <device> <list|array|dict|callable>")
            print(f"  Example: {_bold('inc meca500 [0,0,0,0,0,10]')}")
            print(f"  Example: {_bold('inc meca500 {5: 10}')}  # single axis")
            return
        parts = stripped.split(None, 1)
        if len(parts) < 2:
            print(f"  {_yellow('Usage')}: inc <device> <list|array|dict|callable>")
            return
        device, expr = parts[0], parts[1]
        from IPython import get_ipython
        ip = get_ipython()
        try:
            value = ip.ev(expr)
        except Exception as e:
            print(f"  {_bred('Error')}: failed to evaluate {expr!r}: {e}")
            return
        robot.inc_pos(device, value)
    reg(_m_inc, magic_name='inc')

    def _ee_move_magic(line, incremental):
        """Shared parser for eepos/eeinc: <expr> [--device name] [--space frame].

        Like pos/inc but Cartesian: the value is an [x,y,z,a,b,g] list/array,
        a {axis: value} dict (axis x,y,z,a,b,g), or a callable, solved with the
        Python IK. Targets the active device unless --device is given.
        """
        name = 'eeinc' if incremental else 'eepos'
        usage = f"{name} <list|dict|callable> [--device name] [--space world]"
        raw = line.split()
        space, device = 'local', None
        for flag, setter in (('--space', 'space'), ('--device', 'device')):
            if flag in raw:
                i = raw.index(flag)
                if i + 1 >= len(raw):
                    print(f"  {_yellow('Usage')}: {usage}")
                    return
                if setter == 'space':
                    space = raw[i + 1]
                else:
                    device = raw[i + 1]
                raw = raw[:i] + raw[i + 2:]
        if not raw:
            print(f"  {_yellow('Usage')}: {usage}")
            print(f"  Example: {_bold(name + ' [200,0,400,0,90,0]')}")
            print(f"  Example: {_bold(name + ' {' + chr(39) + 'z' + chr(39) + ': 450} --device GP180_120 --space world')}")
            return
        expr = ' '.join(raw)
        from IPython import get_ipython
        ip = get_ipython()
        try:
            value = ip.ev(expr)
        except Exception as e:
            print(f"  {_bred('Error')}: failed to evaluate {expr!r}: {e}")
            return
        (robot.eeinc if incremental else robot.eepos)(value, device=device, space=space)

    def _m_eepos(line):
        """eepos <list|dict|callable> [--device name] [--space local|world]

        Move an end-effector to an ABSOLUTE Cartesian pose [x,y,z,a,b,g]
        (mm, ZYX-Euler deg) via the Python IK. Unspecified components hold.

        Examples:
            eepos [200, 0, 400, 0, 90, 0]
            eepos {'z': 450}
            eepos {'x': 4334} --device GP180_120 --space world
            eepos my_pose_func()
        """
        _ee_move_magic(line, incremental=False)
    reg(_m_eepos, magic_name='eepos')

    def _m_eeinc(line):
        """eeinc <list|dict|callable> [--device name] [--space local|world]

        Like eepos, but ADD the values to the current EE pose (jog).

        Examples:
            eeinc [0, 0, 50]
            eeinc {'y': -100}
            eeinc {'a': 10} --device GP180_120 --space world
        """
        _ee_move_magic(line, incremental=True)
    reg(_m_eeinc, magic_name='eeinc')

    # ── IK commands ──────────────────────────────────────────────────

    def _m_move(line):
        """move x y z [a b g]"""
        parts = line.split()
        if len(parts) < 3:
            print(f"  {_yellow('Usage')}: move x y z [a b g]")
            return
        robot.move(*[float(x) for x in parts[:6]])
    reg(_m_move, magic_name='move')

    def _m_target(line):
        """target x y z [a b g]"""
        parts = line.split()
        if len(parts) < 3:
            print(f"  {_yellow('Usage')}: target x y z [a b g]")
            return
        robot.target(*[float(x) for x in parts[:6]])
    reg(_m_target, magic_name='target')

    # ── Collision ────────────────────────────────────────────────────

    def _m_collision(line):
        """collision [on|off]"""
        arg = line.strip().lower()
        if arg == "on":
            robot.collision(True)
        elif arg == "off":
            robot.collision(False)
        else:
            robot.collision()
    reg(_m_collision, magic_name='collision')

    def _m_collisions(line):
        robot.collisions()
    reg(_m_collisions, magic_name='collisions')

    # ── Object commands ──────────────────────────────────────────────

    def _m_objects(line):
        robot.objects()
    reg(_m_objects, magic_name='objects')

    def _m_obj(line):
        """obj <name|#idx>"""
        arg = line.strip()
        if not arg:
            print(f"  {_yellow('Usage')}: obj <name|#idx>")
            return
        robot.obj(arg)
    reg(_m_obj, magic_name='obj')

    def _m_objpos(line):
        """objpos <name|#idx> x y z"""
        parts = line.split()
        if len(parts) < 4:
            print(f"  {_yellow('Usage')}: objpos <name|#idx> x y z")
            return
        robot.objpos(parts[0], float(parts[1]), float(parts[2]), float(parts[3]))
    reg(_m_objpos, magic_name='objpos')

    def _m_objrot(line):
        """objrot <name|#idx> rx ry rz"""
        parts = line.split()
        if len(parts) < 4:
            print(f"  {_yellow('Usage')}: objrot <name|#idx> rx ry rz")
            return
        robot.objrot(parts[0], float(parts[1]), float(parts[2]), float(parts[3]))
    reg(_m_objrot, magic_name='objrot')

    def _m_objscale(line):
        """objscale <name|#idx> s [sy sz]"""
        parts = line.split()
        if len(parts) < 2:
            print(f"  {_yellow('Usage')}: objscale <name|#idx> s  or  <name|#idx> sx sy sz")
            return
        robot.objscale(parts[0], *[float(x) for x in parts[1:]])
    reg(_m_objscale, magic_name='objscale')

    def _m_objvis(line):
        """objvis <name|#idx> on|off"""
        parts = line.split()
        if len(parts) < 2 or parts[1].lower() not in ("on", "off"):
            print(f"  {_yellow('Usage')}: objvis <name|#idx> on|off")
            return
        robot.objvis(parts[0], parts[1].lower() == "on")
    reg(_m_objvis, magic_name='objvis')

    def _m_objresetrot(line):
        """objresetrot <name|#idx>"""
        arg = line.strip()
        if not arg:
            print(f"  {_yellow('Usage')}: objresetrot <name|#idx>")
            return
        robot.objresetrot(arg)
    reg(_m_objresetrot, magic_name='objresetrot')

    def _m_objresetscale(line):
        """objresetscale <name|#idx>"""
        arg = line.strip()
        if not arg:
            print(f"  {_yellow('Usage')}: objresetscale <name|#idx>")
            return
        robot.objresetscale(arg)
    reg(_m_objresetscale, magic_name='objresetscale')

    # ── Device / session commands ────────────────────────────────────

    def _m_devices(line):
        robot.devices()
    reg(_m_devices, magic_name='devices')

    def _m_device(line):
        """device <name>"""
        arg = line.strip()
        if not arg:
            print(f"  {_yellow('Usage')}: device <name>")
            print(f"  Type {_bold('devices')} to list available devices.")
            return
        robot.device(arg)
    reg(_m_device, magic_name='device')

    def _m_session(line):
        """session [id]"""
        arg = line.strip()
        if not arg:
            robot.session()
        else:
            robot.session(arg)
    reg(_m_session, magic_name='session')

    def _m_sessions(line):
        robot.sessions()
    reg(_m_sessions, magic_name='sessions')

    # ── Plan ─────────────────────────────────────────────────────────

    def _m_plan(line):
        """plan --start <axes> --end <axes> [--stepsize d] [--steptime ms]"""
        raw = line.split()
        if not raw:
            names_eg = ' '.join(
                f'{n.split()[0].lower()}=0' for _, n in robot._movable_joints[:3]
            )
            print(f"  {_yellow('Usage')}: plan "
                  f"{_cyan('--start')} {_cyan('<axes>')} "
                  f"{_cyan('--end')} {_cyan('<axes>')} "
                  f"[{_cyan('--stepsize')} {_cyan('<deg>')}] "
                  f"[{_cyan('--steptime')} {_cyan('<ms>')}]")
            print(f"  Positional: {_dim(' '.join(['0'] * robot._n_movable))}")
            print(f"  Named:      {_dim(names_eg + ' ...')}")
            return

        def _get_flag(flag, default=None):
            nonlocal raw
            if flag in raw:
                i = raw.index(flag)
                val = raw[i + 1]
                raw = raw[:i] + raw[i+2:]
                return val
            return default

        def _get_angle_list(flag):
            nonlocal raw
            if flag not in raw:
                return None
            i = raw.index(flag)
            tokens = []
            j = i + 1
            while j < len(raw) and not raw[j].startswith('--'):
                tokens.append(raw[j])
                j += 1
            raw = raw[:i] + raw[j:]
            if not tokens:
                return None
            if any('=' in t for t in tokens):
                vals = [0.0] * robot._n_movable
                for t in tokens:
                    if '=' not in t:
                        raise ValueError(f"Mix of positional and named axes: {t!r}")
                    name, val = t.split('=', 1)
                    idx = robot._resolve_axis_name(name.strip())
                    if idx is None:
                        names = ', '.join(n for _, n in robot._movable_joints)
                        raise ValueError(f"Unknown axis {name!r}. Available: {names}")
                    vals[idx] = float(val)
                return vals
            else:
                return [float(t) for t in tokens]

        try:
            stepsize_str = _get_flag('--stepsize')
            steptime_str = _get_flag('--steptime')
            start_vals = _get_angle_list('--start')
            end_vals = _get_angle_list('--end')
        except ValueError as e:
            print(f"  {_bred('Error')}: {e}")
            return

        if start_vals is None or end_vals is None:
            print(f"  {_yellow('Error')}: --start and --end are required")
            return

        stepsize = float(stepsize_str) if stepsize_str is not None else 5.0
        steptime = int(steptime_str) if steptime_str is not None else 80
        robot.plan(start_vals, end_vals, stepsize=stepsize, steptime=steptime)
    reg(_m_plan, magic_name='plan')

    # ── Scan ─────────────────────────────────────────────────────────

    def _m_scan(line):
        """scan <axis> <start> <end> <step> [<axis> ...] [--steptime ms] [--space local|world]
        scan <axis> [<axis> ...] func_or_expr() [--steptime ms] [--space local|world]
        scan <device> array_var [--steptime ms]

        Kappa virtual axes use a 'v:' prefix, e.g.:
            scan v:chi 0 90 5
            scan v:chi 0 90 5 v:theta 0 2

        Cartesian end-effector axes use an 'ee:' prefix (x,y,z mm; a,b,g deg),
        solved with the Python IK. --space picks the frame (default local).
        Prefix the axis with a device name (Device:ee:<axis>) to target a
        specific device or combine several in one scan:
            scan ee:x 150 250 10
            scan ee:x 150 250 5 ee:y -50 50 5
            scan ee:z 200 400 10 --space world
            scan GP180_120:ee:z 200 400 10
            scan GP180_120:ee:z 200 400 10 Meca500:ee:x 150 250 10
            scan GP180_120:ee:y 354 400 10 I16_diff:delta 0 120 10   # Cartesian + joint

        Vector scan with device name:
            scan GP180_120 scanpoints
            scan GP180_120 Meca500 combined_pts
            scan robot1 robot2 my_func()
        """
        raw = line.split()
        if not raw:
            robot.scan()
            return

        steptime = 80
        if '--steptime' in raw:
            i = raw.index('--steptime')
            steptime = int(raw[i + 1])
            raw = raw[:i] + raw[i+2:]

        space = 'local'
        if '--space' in raw:
            i = raw.index('--space')
            space = raw[i + 1]
            raw = raw[:i] + raw[i+2:]

        def _is_number(s):
            try:
                float(s)
                return True
            except ValueError:
                return False

        # Detect array scan: last token ends with "()" or "(args)"
        # e.g. "scan delta gamma polar_func()" or "scan delta gamma my_func(10)"
        last_token = raw[-1] if raw else ""
        paren_idx = last_token.find("(")
        if paren_idx > 0 and last_token.endswith(")"):
            # Array scan mode: tokens before the func call are axis names
            axis_names = raw[:-1]
            func_expr = last_token
            if len(axis_names) < 1:
                print(f"  {_yellow('Error')}: array scan needs at least one axis name before the function")
                return
            # Evaluate the function expression in the IPython namespace
            from IPython import get_ipython
            ip = get_ipython()
            try:
                data = ip.ev(func_expr)
            except Exception as e:
                print(f"  {_bred('Error')}: failed to evaluate {func_expr!r}: {e}")
                return
            robot.scan(*axis_names, data, steptime=steptime, space=space)
            return

        # Detect vector/array scan: last token is a variable that evaluates
        # to an array-like or callable, preceding tokens are axis or device names.
        # e.g. "scan GP180_120 scanpoints" or "scan J1 J2 my_array"
        if len(raw) >= 2 and not _is_number(raw[-1]):
            from IPython import get_ipython
            ip = get_ipython()
            try:
                data = ip.ev(raw[-1])
                if _is_array_like(data) or callable(data):
                    axis_names = raw[:-1]
                    robot.scan(*axis_names, data, steptime=steptime, space=space)
                    return
            except Exception:
                pass

        if len(raw) < 4:
            robot.scan()
            return

        groups = []
        i = 0
        while i < len(raw):
            axis_name = raw[i]
            i += 1
            nums = []
            while i < len(raw) and _is_number(raw[i]):
                nums.append(float(raw[i]))
                i += 1
            groups.append(tuple([axis_name] + nums))

        robot.scan(*groups, steptime=steptime, space=space)
    reg(_m_scan, magic_name='scan')

    # ── Help ─────────────────────────────────────────────────────────

    def _m_rhelp(line):
        robot.help()
    reg(_m_rhelp, magic_name='rhelp')


# ── Entry point ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Robot/Device IPython Remote Control Terminal",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Examples:\n"
               "  python robot_ipython.py\n"
               "  python robot_ipython.py --config i16_config.json\n"
               "  python robot_ipython.py --url ws://192.168.1.100:8080/ws --config meca500_config.json\n"
               "  python robot_ipython.py --session ab12cd34\n",
    )
    parser.add_argument("--url", default="ws://localhost:8080/ws",
                        help="WebSocket URL (default: ws://localhost:8080/ws)")
    parser.add_argument("--config", default="meca500_config.json",
                        help="Path to device config JSON (default: meca500_config.json)")
    parser.add_argument("--session", default=None,
                        help="Session ID of the viewer instance to connect to.")
    args = parser.parse_args()

    url = args.url
    if args.session:
        sep = "&" if "?" in url else "?"
        url = f"{url}{sep}session={args.session}"

    robot = RobotClient(url=url, config=args.config)

    banner = _build_banner(robot)

    import IPython

    from traitlets.config import Config

    c = Config()
    c.TerminalInteractiveShell.banner1 = banner
    c.TerminalInteractiveShell.banner2 = ""
    c.TerminalInteractiveShell.confirm_exit = False
    c.TerminalInteractiveShell.prompts_class = _RobotPrompts
    c.InteractiveShellApp.exec_lines = [
        "_register_magics(get_ipython(), robot)",
    ]

    IPython.start_ipython(
        argv=[],
        user_ns={
            "robot": robot,
            "r": robot,
            "time": time,
            "np": np,
            "kinematics": kinematics,
            "Meca500_kin": Meca500_kin,
            "GP225_kin": GP225_kin,
            "GP180_120_kin": GP180_120_kin,
            "GP280_kin": GP280_kin,
            "MotoMini_kin": MotoMini_kin,
            "_register_magics": _register_magics,
        },
        config=c,
    )

    robot.disconnect()


if __name__ == "__main__":
    main()
