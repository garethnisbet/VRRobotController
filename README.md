# Robot & Device Visualisation

Interactive 3D visualisation and control for robots and scientific instruments. Config-driven — works with any device defined by a JSON config and GLB model. Includes forward and inverse kinematics, mesh import, collision detection, VR support (Meta Quest), real robot teleoperation, and a remote control API.

## Supported Devices

| Device | Config | Type | Description |
|--------|--------|------|-------------|
| Meca500 R3 | `meca500_config.json` | 6-DOF serial | Compact industrial manipulator |
| Hexapod | `hexapod_config.json` | Stewart platform | 6-DOF parallel kinematic platform |
| i16 Diffractometer | `i16_config.json` | Branching | Diamond Light Source 6-circle diffractometer |
| i19 Kappa Diffractometer | `i19_config.json` | Branching | Diamond Light Source kappa diffractometer |
| Yaskawa GP225 | `gp225_config.json` | 6-DOF serial | Heavy-payload industrial robot |
| Yaskawa GP280 | `gp280_config.json` | 6-DOF serial | Heavy-payload industrial robot |
| Yaskawa GP180-120 | `gp180_config.json` | 6-DOF serial | Heavy-payload industrial robot |
| Yaskawa MotoMini | `motomini_config.json` | 6-DOF serial | Compact industrial robot |

New devices can be added from Blender scenes using `import_robot.py` (see [Adding New Devices](#adding-new-devices)). Hexapod/Stewart platforms use `import_hexapod.py`.

## Features

- **Config-driven viewer** — a single generic `threejs_scene.html` viewer loads any device via JSON config
- **Multi-device scene** — load multiple devices simultaneously from the add-device dropdown; click a device in the list or click its mesh to switch active device
- **Serial and parallel kinematics** — supports serial manipulators (FK/IK via Jacobian) and Stewart platform hexapods (6-DOF parallel FK/IK)
- **Device renaming** — double-click a device name in the device list to rename it
- **Device origin transform** — move and rotate device origins with translate/rotate gizmo modes and numeric input fields; World/Local space toggle
- **Device parenting** — parent a device to a link on another device so it follows the kinematic chain
- **Auto-fit camera** — camera automatically frames the loaded model on startup
- **Forward Kinematics** — joint angle sliders for serial devices, platform pose sliders for hexapods
- **Inverse Kinematics** — 6-DOF damped least-squares solver (serial), Newton-Raphson leg-length solver (hexapod)
- **Branching kinematic chains** — supports devices with multiple independent chains and sub-branches
- **Draggable IK target** — move the green sphere with the gizmo or use XYZ / alpha-beta-gamma sliders
- **Orientation gizmo** — visual end-effector orientation indicator showing the current tool frame axes
- **Double-click to type** — double-click any slider value label to enter a number directly
- **Numeric input fields** — direct position/rotation/scale entry for devices and objects
- **Mesh labels toggle** — show/hide object name labels on all meshes
- **Mesh import** — load external STL, OBJ, PLY, and GLB/GLTF files into the scene with auto-scaling, labels, and per-object colour
- **Primitive objects** — add cube, sphere, and cylinder primitives directly from the toolbar
- **Object duplication** — duplicate any imported or primitive object with a single click
- **Persistent objects** — imported and primitive objects are automatically saved to IndexedDB and restored on page reload
- **Object manipulation** — click objects to select, then move, rotate, or scale with transform gizmos (keyboard: T/R/S, Escape to deselect); World/Local space toggle for gizmo axis alignment
- **Parent-child linking** — attach objects to device links so they follow the kinematic chain
- **Self-collision detection** — BVH-accelerated triangle-level intersection testing between device links, using kinematic adjacency to skip physically connected parts
- **Collision detection** — intersection testing between device links and imported scene objects, with red highlight on colliding meshes
- **Screenshot** — one-click PNG capture of the WebGL view composited with the control panel overlay
- **Unified Euler convention** — viewer, WebSocket state, and the Python `GNKinematics` library all report end-effector orientation as the same ZYX Euler triple
- **VR support** — WebXR-based VR with Meta Quest controller interaction, passthrough toggle, persistent anchors for drift correction, and hexapod platform grab
- **Real robot bridge** — teleoperate a physical Meca500 from the viewer or VR with velocity-scaled joint control, E-Stop, and speed adjustment
- **Remote control API** — two-way WebSocket API for controlling any device from Python or any WebSocket client
- **Session routing** — each browser tab gets a unique session ID; controllers can target a specific tab or broadcast to all
- **Connection info panel** — click the API status indicator (top-left) to see the session ID, connection command, and download `RemoteAPI.zip`
- **HTTPS / WSS support** — self-signed certificate generation for secure connections (required for WebXR on non-localhost)

## Running

### 1. Visualisation Server (required)

The server serves the viewer and provides the WebSocket API that all other components connect to.

```bash
pip install aiohttp
python server.py
```

This starts an HTTP server on port 8080. Open `http://localhost:8080` in a browser.

**Options:**

| Flag | Default | Description |
|------|---------|-------------|
| `--port PORT` | `8080` | HTTP/HTTPS port |
| `--host HOST` | `0.0.0.0` | Bind address |
| `--config FILE` | `meca500_config.json` | Default robot config to load |
| `--ssl` | off | Enable HTTPS with auto-generated self-signed certificate |
| `--cert FILE` | — | Path to SSL certificate (implies HTTPS) |
| `--key FILE` | — | Path to SSL private key |

**Examples:**

```bash
# Basic — HTTP on default port
python server.py

# Different robot config
python server.py --config hexapod_config.json

# HTTPS (required for VR on Meta Quest over network)
python server.py --ssl --port 8443

# HTTPS with your own certificate
python server.py --cert /path/to/cert.pem --key /path/to/key.pem
```

Once running, the viewer URL and WebSocket endpoint are printed to the console.

### 2. IPython Remote Control Client (optional)

An interactive terminal for controlling the viewer programmatically.

```bash
pip install websockets ipython
python robot_ipython.py
```

**Options:**

| Flag | Default | Description |
|------|---------|-------------|
| `--url URL` | `ws://localhost:8080/ws` | WebSocket URL of the server |
| `--config FILE` | `meca500_config.json` | Robot config (sets prompt name and joint info) |
| `--session ID` | — | Target a specific viewer tab by session ID |

**Examples:**

```bash
# Broadcast to all viewer tabs
python robot_ipython.py

# Target a specific viewer tab
python robot_ipython.py --session ab12cd34

# Connect to remote server with HTTPS
python robot_ipython.py --url wss://192.168.1.100:8443/ws

# Different device
python robot_ipython.py --config i16_config.json
```

Type `rhelp` in the IPython terminal for a full command reference.

### 3. Real Robot Bridge (optional — Meca500 only)

Bridges the VR viewer to a physical Meca500 robot. Joint angles from the viewer become velocity targets for the real robot. The bridge appears as a "Real Robot" panel in the viewer UI with Enable/Disable, E-Stop, Reset, and speed controls.

```bash
pip install mecademicpy websockets
python meca500_bridge.py
```

**Options:**

| Flag | Default | Description |
|------|---------|-------------|
| `--robot-ip IP` | `192.168.0.100` | Meca500 IP address |
| `--ws-url URL` | `ws://localhost:8080/ws` | Server WebSocket URL |
| `--session ID` | — | Target a specific viewer session |
| `--vel-scale FLOAT` | `0.25` | Velocity scaling 0-1 (25% = safe default) |
| `--gain FLOAT` | `2.0` | Proportional control gain |
| `--hz INT` | `50` | Control loop frequency |
| `--auto-enable` | off | Start teleoperation immediately |
| `--sim` | off | Simulation mode (no real robot needed) |

**Examples:**

```bash
# Simulation mode (test without a real robot)
python meca500_bridge.py --sim

# Connect to robot at default IP
python meca500_bridge.py --robot-ip 192.168.0.100

# Slower, gentler control
python meca500_bridge.py --vel-scale 0.1 --gain 1.5

# Target a specific VR session with HTTPS server
python meca500_bridge.py --ws-url wss://localhost:8443/ws --session abc123

# Full auto-start
python meca500_bridge.py --robot-ip 192.168.0.100 --auto-enable --vel-scale 0.25
```

**Safety features:**
- Velocity clamped to `--vel-scale` fraction of max joint speed
- `SetVelTimeout` auto-stops robot if no command arrives within 100 ms
- Connection watchdog pauses robot if bridge disconnects
- E-Stop available from the viewer panel and VR (A/X button on Quest controllers)
- Ctrl+C performs clean shutdown (deactivate + disconnect)

**Architecture:**

```
Direct bridge (default):
  VR Headset / Browser ←WebSocket→ server.py ←WebSocket→ meca500_bridge.py ←TCP→ Meca500

Via EPICS IOC (for monitoring, interlocks, audit):
  VR Headset / Browser ←WebSocket→ server.py ←WebSocket→ epics_bridge.py ←pvAccess→ EPICS IOC ←TCP→ Meca500
```

### 4. EPICS Bridge (optional — Meca500 only)

Routes teleoperation through the EPICS IOC (`EPICS_Control_Container`) instead of connecting directly to the robot. This adds PV monitoring, archiving, and interlock integration at the cost of pvAccess round-trip latency. The robot's `SetVelTimeout(0.1s)` hardware deadman is active in both paths.

**Prerequisites:** The EPICS IOC must be running (`docker compose up` in `EPICS_Control_Container`).

```bash
pip install p4p websockets
python epics_bridge.py
```

**Options:**

| Flag | Default | Description |
|------|---------|-------------|
| `--prefix PREFIX` | `MECA500` | EPICS PV prefix |
| `--ws-url URL` | `ws://localhost:8080/ws` | Server WebSocket URL |
| `--session ID` | — | Target a specific viewer session |
| `--vel-scale FLOAT` | `0.25` | Velocity scaling 0-1 (25% = safe default) |
| `--gain FLOAT` | `2.0` | Proportional control gain |
| `--hz INT` | `50` | Control loop frequency |
| `--auto-enable` | off | Start teleoperation immediately |
| `--sim` | off | Simulation mode (no IOC needed) |

**Examples:**

```bash
# Simulation mode (no IOC or robot needed)
python epics_bridge.py --sim

# Connect to IOC with default prefix
python epics_bridge.py --prefix MECA500

# Slower control, target a specific VR session
python epics_bridge.py --vel-scale 0.1 --gain 1.5 --session abc123

# Full auto-start with HTTPS viewer
python epics_bridge.py --ws-url wss://localhost:8443/ws --auto-enable
```

**Compared to meca500_bridge.py:**

| | `meca500_bridge.py` | `epics_bridge.py` |
|---|---|---|
| Robot connection | Direct TCP via mecademicpy | pvAccess via EPICS IOC |
| Dependencies | mecademicpy, websockets | p4p, websockets |
| Latency | Lower (direct TCP) | Higher (pvAccess round-trip) |
| Monitoring | None | All PVs visible to any EPICS client |
| Interlocks | None | Can be added at the IOC level |

See the [EPICS_Control_Container README](../EPICS_Control_Container/README.md) for the full PV reference and IOC setup.

### Typical Multi-Component Setup

A full setup with VR, real robot control, and scripted automation:

```bash
# Terminal 1: Start the server with HTTPS for VR
python server.py --ssl --port 8443

# Terminal 2: Connect the robot bridge (direct TCP — pick one)
python meca500_bridge.py --robot-ip 192.168.0.100 --ws-url wss://localhost:8443/ws

# Terminal 2 (alternative): Connect via EPICS IOC
# First start the IOC: cd ../EPICS_Control_Container && docker compose up --build
python epics_bridge.py --ws-url wss://localhost:8443/ws

# Terminal 3: Open an IPython scripting terminal
python robot_ipython.py --url wss://localhost:8443/ws

# Browser / VR headset: open https://localhost:8443
```

### Standalone (no server)

For viewing only (no remote control, no bridge):

```bash
python3 -m http.server 8000
```

Open `http://localhost:8000/threejs_scene.html?config=meca500_config.json`.

## Adding New Devices

### Importing from Blender

The `import_robot.py` script extracts bone transforms, rotation axes, mesh parenting, and joint limits from a Blender armature, then exports a config JSON and skin-free GLB.

#### Blender Setup

1. **Armature**: create an armature with one bone per joint/kinematic link. Bones should form a parent-child chain matching the kinematic chain.
2. **Mesh parenting**: parent each mesh to its corresponding bone (select mesh, then bone in Pose Mode, `Ctrl+P` -> Bone).
3. **Rotation mode**: set all pose bones to **Quaternion** rotation mode.
4. **Quaternion locks**: for each bone, lock all quaternion components except the one that corresponds to the rotation axis:
   - Lock W, Y, Z -> free X (rotation about bone-local X)
   - Lock W, X, Z -> free Y (rotation about bone-local Y)
   - Lock W, X, Y -> free Z (rotation about bone-local Z)
   - Lock all (W, X, Y, Z) -> fixed joint (no rotation)
   - Leave all unlocked -> fixed joint (treated as structural)
5. **Joint limits**: set IK limits on the free axis for each movable bone (`Bone Properties -> Inverse Kinematics`). If no IK limits are set, the importer defaults to [-180, 180].
6. **Render visibility**: hide any helper meshes by disabling their render visibility (camera icon in outliner). These will be excluded from the GLB export.

#### Running the Importer

In Blender's Script Editor (or Python console):

```python
exec(open('/path/to/import_robot.py').read())
```

To override defaults, set variables before `exec()`:

```python
ARMATURE_NAME = 'MyArmature'     # default: auto-detect first armature
DEVICE_NAME = 'My Robot'         # default: armature name
CONFIG_FILE = 'my_robot_config.json'  # default: derived from device name
GLB_FILE = 'my_robot_scene.glb'  # default: derived from config name
exec(open('/path/to/import_robot.py').read())
```

The script:
1. Finds the armature and extracts bone rest transforms (converted from Blender Z-up to Three.js Y-up)
2. Determines rotation axes from quaternion locks
3. Maps meshes to bones via parent chains
4. Extracts IK joint limits
5. Exports a skin-free GLB (temporarily unparents bone-parented meshes to avoid glTF skins)
6. Generates a config JSON with joints, links, eeOffset, and eeAxes
7. Registers the config in `js/panel.js` if not already present

#### Post-Import Checklist

Open `http://localhost:8000/threejs_scene.html?config=my_robot_config.json` and verify:

1. All meshes load and appear correctly
2. FK sliders move the correct joints
3. Each joint rotates about the correct axis and in the correct direction
4. Labels toggle shows correct mesh names

If rotation directions are wrong for specific joints, negate the `axis` array in the config JSON (e.g. `[0, -1, 0]` -> `[0, 1, 0]`). If meshes are missing, check GLTFLoader name sanitization in the browser console — the importer sanitizes names (spaces -> underscores, dots removed) but the GLB mesh names must match.

#### apiSign

If the robot manufacturer's joint angle convention is opposite to the viewer's for specific joints, add `"apiSign": -1` to those joints in the config. This flips the sign on the slider display and the WebSocket API without changing the physical rotation axis. For example, the Meca500 J4 has `"apiSign": -1` because the manufacturer defines positive J4 in the opposite direction.

### Importing Hexapod/Stewart Platforms

Use `import_hexapod.py` for parallel kinematic platforms. The hexapod config defines base pivot points, platform pivot points, leg geometry, and pose limits rather than a serial joint chain.

### Adding Robot Kinematics Definitions

To enable IK via the Python `GNKinematics` library (used by `robot_ipython.py`), add a kinematics definition to `RobotDefinitions.py`:

```python
from GNKinematics import kinematics

MyRobot_kin = kinematics.kinematics.from_home_positions(
    v0=np.array([0, 0, 135.0]),       # base to J2 (mm)
    v1=np.array([0, 0, 270.0]),       # J2 to J3
    v2=np.array([60, 0, 308]),        # J3 to J4
    v3=np.array([120, 0, 308]),       # J4 to J5
    v4=np.array([190, 0, 308]),       # J5 to J6 flange
    motor_limits=np.array([[-175, 175], [-70, 90], [-135, 70],
                            [-170, 120], [-90, 115], [-360, 360]]),
    centre_offset=[0, 0, 0],
    tool_offset=[0, 0, 0],
    strategy='minimum_movement',
    weighting=[6, 5, 4, 3, 2, 1],
)
```

The `v0`-`v4` vectors are the joint positions at home (all joints at zero), in the robot's Z-up coordinate frame. `motor_limits` are per-joint angle limits in degrees.

Then add the new object to the IPython namespace in `robot_ipython.py`:

1. Import it at the top: `from RobotDefinitions import ..., MyRobot_kin`
2. Add it to the `user_ns` dict passed to `IPython.start_ipython()`:
   ```python
   user_ns={
       ...
       "MyRobot_kin": MyRobot_kin,
   }
   ```

### Config File Structure

```json
{
  "name": "Device Name",
  "model": "device_scene.glb",
  "joints": [
    {
      "name": "joint_name",
      "bone": "blender_bone_name",
      "restPos": [x, y, z],
      "restQuat": [w, x, y, z],
      "axis": [x, y, z],
      "limits": [-180, 180],
      "parent": 0,
      "fixed": false,
      "apiSign": 1
    }
  ],
  "links": [
    { "name": "mesh_name", "label": "Display Name", "joint": 0 }
  ],
  "eeOffset": [0, 0, -0.05],
  "eeAxes": [[0, 0, -1], [0, -1, 0], [1, 0, 0]],
  "demoPose": [0, 45, -90]
}
```

Key fields:
- **joints**: one entry per bone — `restPos`/`restQuat` from Blender (C matrix converted), `axis` from quaternion lock analysis, `parent` index (-1 for roots), `fixed` for non-movable kinematic links, optional `apiSign` (-1 to flip slider/API convention)
- **links**: maps GLB mesh names (sanitized: spaces->underscores, dots removed) to joint indices
- **eeOffset**: displacement from last joint to end-effector point (derived from last bone length)
- **eeAxes**: end-effector crosshair axes — use `[[0,0,-1],[0,-1,0],[1,0,0]]` for all robots
- **demoPose**: joint angles in degrees for the demo button (one per joint, fixed joints = 0)

Hexapod configs use `"type": "hexapod"` and define `basePivots`, `platformPivots`, `legs`, and `poseLimits` instead of serial joints.

## Objects

### Mesh Import

Click **Import Mesh** to load files into the scene. Supported formats:

| Format | Extension | Notes |
|--------|-----------|-------|
| STL | `.stl` | Binary or ASCII; auto-scaled if bounding box > 1 m |
| OBJ | `.obj` | Geometry only; no MTL material files |
| PLY | `.ply` | Binary and ASCII; vertex colours preserved if present |
| GLB / GLTF | `.glb`, `.gltf` | Full scene hierarchy, materials, and textures |

### Primitives

Click **Cube**, **Sphere**, or **Cylinder** to add a primitive shape. Primitives behave identically to imported objects — they can be moved, coloured, parented, and are persisted across reloads.

### Transform Gizmos

| Key | Mode |
|-----|------|
| `T` | Move (translate) |
| `R` | Rotate |
| `S` | Scale |
| `Esc` | Deselect |

Both device and object gizmos have a **World/Local** toggle button. In World mode the gizmo axes align with the scene axes; in Local mode they align with the object's own axes. The toggle resets to World when the gizmo is deactivated.

### Parent-Child Linking

Use the **Parent** dropdown to attach objects to device links. Parented objects follow the kinematic chain. Local transforms are preserved when reparenting.

## Collision Detection

Toggle **Collision: ON/OFF** in the panel. The viewer tests for triangle-level intersections using:

1. **Broad phase** — AABB check to eliminate distant pairs
2. **Narrow phase** — BVH-accelerated triangle-triangle intersection via [three-mesh-bvh](https://github.com/gkjohnson/three-mesh-bvh)

Self-collision between device links uses kinematic adjacency analysis — links sharing the same joint or connected through a parent-child relationship are skipped. Links on separate branches are always checked. Hexapod devices skip self-collision entirely (parallel kinematics).

## VR Support

The viewer supports WebXR for Meta Quest headsets. VR features:

- **Controller interaction** — grip to grab the IK target or hexapod platform; thumbstick for locomotion
- **Passthrough toggle** — switch between VR passthrough and rendered background
- **Persistent anchors** — saves VR anchor to IndexedDB for drift correction across sessions
- **E-Stop** — A/X button triggers E-Stop when the real robot bridge is active; otherwise resets to home
- **Exit VR** — button in the panel or B/Y on controller

VR over network requires HTTPS. Use `python server.py --ssl` to enable it.

## Remote Control

The WebSocket API at `ws://localhost:8080/ws` allows any client to control devices, manage multi-device scenes, manipulate objects, and control the camera in real time. All commands target the active device by default; include `"device": "<name>"` to target a specific device.

### Session Routing

Each browser tab that connects to the viewer is assigned a unique **session ID** (an 8-character hex string, e.g. `ab12cd34`). The ID is shown in the status bar (`API: connected [ab12cd34]`) and in the connection info panel.

Controllers can target a specific viewer tab or broadcast to all:

| Connection | Routes to |
|-----------|-----------|
| `ws://localhost:8080/ws` | All connected viewer tabs |
| `ws://localhost:8080/ws?session=ab12cd34` | Only the tab with that session ID |

Active sessions can be listed via the HTTP endpoint:
```
GET http://localhost:8080/sessions
```

### Interactive Client (IPython)

```bash
python3 robot_ipython.py --config meca500_config.json                        # broadcast to all tabs
python3 robot_ipython.py --config meca500_config.json --session ab12cd34     # target a specific tab
python3 robot_ipython.py --config i16_config.json --session ab12cd34       # i16 diffractometer
python3 robot_ipython.py --url ws://192.168.1.100:8080/ws --session ab12cd34  # remote server
```

The client launches an IPython terminal with a pre-connected `robot` object. It supports two syntaxes — **Python method calls** for full programmatic control, and **space-separated commands** (via IPython magics) for quick interactive use. The prompt, help text, and tab completion adapt to the loaded device. The `robot_ipython.py` file can also be downloaded from the connection info panel in the viewer.

**Both syntaxes work side by side:**

```python
meca500 [1]: home                                    # space-separated
meca500 [2]: robot.home()                            # Python method
meca500 [3]: joints 0 30 60 0 45 90                  # space-separated
meca500 [4]: robot.joints(0, 30, 60, 0, 45, 90)     # Python method
meca500 [5]: for a in range(0, 91, 10):              # full Python syntax
         ...:     robot.joint('J1', a)
         ...:     time.sleep(0.1)
```

**Position queries** (return values for programmatic use):

| Property / Method | Example | Description |
|---------|---------|-------------|
| `robot.pos` | `x, y, z = robot.pos` | End-effector position [x, y, z] mm |
| `robot.ori` | `robot.ori` | End-effector orientation [a, b, g] degrees |
| `robot.angles` | `robot.angles` | All joint angles (list) |
| `robot.get_joint('J1')` | `a = robot.get_joint('J1')` | Single joint angle by name |
| `robot.mode` | `robot.mode` | Current mode ('FK' or 'IK') |
| `robot.platform_pose` | `robot.platform_pose` | Hexapod platform pose [x,y,z,rx,ry,rz] |
| `robot.leg_lengths` | `robot.leg_lengths` | Hexapod leg lengths [l1..l6] mm |
| `robot.get_device_pos('GP225')` | `d = robot.get_device_pos()` | Any device: pos, rot, joints, EE |
| `robot.get_obj_pos('cube_1')` | `o = robot.get_obj_pos(0)` | Any object: pos, rot, scale, BB |

**Device commands:**

| Space-separated | Python | Description |
|---------|---------|-------------|
| `state` | `robot.state()` | Request current device state |
| `devices` | `robot.devices()` | List all loaded devices |
| `device i16` | `robot.device('i16')` | Switch active device by name |
| `sessions` | `robot.sessions()` | List viewer session IDs |
| `home` | `robot.home()` | All joints to 0 / platform to home |
| `fk` | `robot.fk()` | Switch to FK mode |
| `ik` | `robot.ik()` | Switch to IK mode |
| `joints 45 -90 0 0 30 0` | `robot.joints(45, -90, 0, 0, 30, 0)` | Set all movable joint angles (degrees) |
| `joint gamma 45` | `robot.joint('gamma', 45)` | Set a single joint by name |
| `pos meca500 [0,0,0,0,0,0]` | `robot.set_pos('meca500', [0,0,0,0,0,0])` | Set joints on a named device |
| `pos meca500 {2: 45}` | `robot.set_pos('meca500', {2: 45})` | Set individual axes only |
| `inc meca500 [0,0,0,0,0,10]` | `robot.inc_pos('meca500', [0,0,0,0,0,10])` | Increment joints relative to current |
| `move 150 100 300 45 0 0` | `robot.move(150, 100, 300, 45, 0, 0)` | IK move to position (mm) + orientation (deg) |
| `target 190 0 308` | `robot.target(190, 0, 308)` | Set IK target without switching mode |
| `demo` | `robot.demo()` | Run the config's demo pose |

**Device transform commands:**

| Python | Description |
|--------|-------------|
| `robot.devtranslate(dx, dy, dz, space='parent')` | Translate device origin by delta (mm) |
| `robot.devrotate(rx, ry, rz, space='parent')` | Rotate device origin by delta (deg) |
| `robot.devpose([x, y, z, rx, ry, rz])` | Set device origin position and rotation |
| `robot.worldToLocal([x,y,z], [a,b,g])` | World -> device-local coordinate transform |

**Hexapod commands:**

| Python | Description |
|--------|-------------|
| `robot.platform([x,y,z,rx,ry,rz])` | Set platform pose (mm, deg) |
| `robot.hexapod_fk([pose])` | FK: pose -> leg lengths |
| `robot.hexapod_ik([l1..l6])` | IK: leg lengths -> pose |
| `robot.get_leg_lengths()` | Get current leg lengths (detailed) |
| `robot.set_leg_lengths(l1..l6)` | Set pose via leg lengths (mm) |

**Motion planning commands:**

| Space-separated | Python | Description |
|---------|---------|-------------|
| `plan --start 0 0 0 --end 45 90 0` | `robot.plan([0,0,0], [45,90,0])` | Path plan between two poses |
| `scan theta 0 90 5` | `robot.scan(('theta', 0, 90, 5))` | 1D scan |
| `scan theta 0 90 5 phi 0 30 2` | `robot.scan(('theta',0,90,5), ('phi',0,30,2))` | 2D grid scan |
| `scan theta 0 90 5 phi 0 1` | `robot.scan(('theta',0,90,5), ('phi',0,1))` | Coupled scan |
| `scan DevA:J1 0 50 5 DevB:J1 0 30 5` | `robot.scan(('DevA:J1',0,50,5), ('DevB:J1',0,30,5))` | Multi-device scan |
| `scan v:chi 0 90 5` | `robot.scan(('v:chi', 0, 90, 5))` | Kappa virtual-axis scan |
| `scan GP180_120 scanpoints` | `robot.scan('GP180_120', waypoints)` | Vector scan with device name |
| — | `robot.scan(('@Cube:tx', 0, 100, 10))` | Object translation scan |
| — | `robot.scan(('@Cube:tx',0,100,10), space='world')` | Object scan in world coords |
| — | `robot.scan(('J1',0,90,5), ('@Cube:tz',0,50,5))` | Mixed joint + object scan |

Object scan axes use `@ObjectName:component` syntax where component is `tx`, `ty`, `tz`, `rx`, `ry`, or `rz`. The `space` parameter (`'local'` or `'world'`) controls the coordinate frame for object transforms (default: `'local'`).

Kappa virtual axes use a `v:` prefix (`v:chi`, `v:theta`, `v:phi`) to disambiguate from the physical `theta`/`phi` joints. Virtual scans target the active kappa device and cannot be mixed with physical-joint axes in the same scan.

Vector scans accept a device name in place of listing all its joints: `robot.scan('GP180_120', waypoints)` expands to all joints on that device. Multiple devices can be combined: `robot.scan('Meca500', 'GP225', combined_pts)`.

**Object commands:**

| Space-separated | Python | Description |
|---------|---------|-------------|
| `objects` | `robot.objects()` | List all imported objects |
| `obj MyPart` | `robot.obj('MyPart')` | Get object details |
| `objpos MyPart 100 50 0` | `robot.objpos('MyPart', 100, 50, 0)` | Set object position (mm) |
| `objrot #0 0 0 45` | `robot.objrot('#0', 0, 0, 45)` | Set object rotation (degrees) |
| `objscale MyPart 2` | `robot.objscale('MyPart', 2)` | Set uniform scale |
| `objvis MyPart on` | `robot.objvis('MyPart', True)` | Show/hide object |
| `collision on` | `robot.collision(True)` | Enable/disable collision detection |
| `collisions` | `robot.collisions()` | Get current collision pairs |

Object transforms accept a `space` parameter (`'parent'`, `'local'`, or `'world'`):

| Python | Description |
|--------|-------------|
| `robot.objtranslate('Cube', dx, dy, dz, space='parent')` | Translate object by delta (mm) |
| `robot.objrotate('Cube', rx, ry, rz, space='parent')` | Rotate object by delta (deg) |

**Visualization and camera:**

| Python | Description |
|--------|-------------|
| `robot.labels()` / `robot.labels(False)` | Show/hide joint labels |
| `robot.origins()` / `robot.origins(False)` | Show/hide joint origin axes |
| `robot.chain()` / `robot.chain(False)` | Show/hide kinematic chain |
| `robot.ortho()` / `robot.ortho(False)` | Orthographic/perspective camera |
| `robot.camera(position=[500,500,500])` | Set camera position/target |
| `robot.snap('iso')` | Snap to preset view |
| `robot.virtual_angles(chi=45)` | Set kappa virtual angles (diffractometers) |

### API Protocol (JSON over WebSocket)

All positions are in mm, angles in degrees, using Z-up robot convention. Most commands accept an optional `"device"` field to target a specific device by name or ID; if omitted, the active device is used.

**Device management:**
```json
{"cmd": "getState"}
{"cmd": "listDevices"}
{"cmd": "getDevice"}
{"cmd": "addDevice", "config": "i16_config.json"}
{"cmd": "removeDevice", "device": "Meca500"}
{"cmd": "renameDevice", "name": "MyRobot"}
{"cmd": "setActiveDevice", "device": "i16"}
{"cmd": "setDeviceOrigin", "position": [100, 0, 0], "rotation": [0, 0, 45]}
{"cmd": "translateDevice", "delta": [10, 0, 0], "space": "parent"}
{"cmd": "rotateDevice", "delta": [0, 0, 45], "space": "local"}
{"cmd": "setDeviceParent", "parent": "dev_0:L3"}
{"cmd": "listConfigs"}
```

**Joint control:**
```json
{"cmd": "setJoints", "angles": [0, -30, 60, 0, 45, 90]}
{"cmd": "setSingleJoint", "index": 1, "angle": -30}
{"cmd": "home"}
{"cmd": "demoPose"}
```

**Hexapod (Stewart platform):**
```json
{"cmd": "setPlatformPose", "pose": [0, 0, 10, 3, 0, 0]}
{"cmd": "hexapodFK", "pose": [0, 0, 10, 3, 0, 0]}
{"cmd": "hexapodIK", "legLengths": [150.1, 150.1, 150.1, 150.1, 150.1, 150.1]}
{"cmd": "getLegLengths"}
{"cmd": "setLegLengths", "legLengths": [150.1, 150.1, 150.1, 150.1, 150.1, 150.1]}
```

**Kappa virtual angles** (diffractometer geometry):
```json
{"cmd": "setVirtualAngles", "chi": 45, "theta": 10, "phi": 20}
{"cmd": "getVirtualAngles"}
{"cmd": "setKappaSign", "positive": true}
```

**IK control:**
```json
{"cmd": "setMode", "mode": "IK"}
{"cmd": "setIKTarget", "position": [190, 0, 308], "orientation": [0, 0, 0]}
{"cmd": "moveTo", "position": [150, 100, 300], "orientation": [45, 0, 0]}
```

**Coordinate transforms:**
```json
{"cmd": "worldToLocal", "position": [100, 0, 50], "orientation": [0, 0, 90]}
```

**Object commands:**
```json
{"cmd": "listObjects"}
{"cmd": "getObject", "object": "MyPart"}
{"cmd": "setObject", "object": "MyPart", "position": [100, 50, 0], "rotation": [0, 0, 45]}
{"cmd": "setObject", "object": "MyPart", "position": [100, 50, 0], "space": "world"}
{"cmd": "setObject", "object": "MyPart", "color": "#ff0000", "parent": "dev_0:L3"}
{"cmd": "translateObject", "name": "MyPart", "delta": [10, 0, 0], "space": "parent"}
{"cmd": "rotateObject", "name": "MyPart", "delta": [0, 0, 45], "space": "local"}
{"cmd": "addPrimitive", "type": "cube"}
{"cmd": "removeObject", "object": "MyPart"}
{"cmd": "duplicateObject", "object": "MyPart"}
{"cmd": "resetObjectRotation", "object": "MyPart"}
{"cmd": "resetObjectScale", "index": 0}
```

`setObject`, `translateObject`, and `rotateObject` accept `"space": "parent"|"local"|"world"` (default: `"parent"`). The `getObject` response includes both local (`position`, `rotation`) and world-frame (`worldPosition`, `worldRotation`) coordinates. `translateDevice` and `rotateDevice` use the same space convention.

**Collision commands:**
```json
{"cmd": "setCollision", "enabled": true}
{"cmd": "getCollisions"}
```

**Visualization toggles:**
```json
{"cmd": "setLabels", "enabled": true}
{"cmd": "setOrigins", "enabled": true}
{"cmd": "setChain", "enabled": true}
{"cmd": "setOrtho", "enabled": true}
```

**Camera control:**
```json
{"cmd": "getCamera"}
{"cmd": "setCamera", "position": [500, 500, 500], "target": [0, 0, 150]}
{"cmd": "snapCamera", "view": "front"}
```
Snap views: `+X`, `-X`, `+Y`, `-Y`, `+Z`, `-Z`, `top`, `bottom`, `front`, `back`, `left`, `right`, `iso`.

**Scene persistence:**
```json
{"cmd": "getSceneState"}
{"cmd": "saveScene"}
{"cmd": "help"}
```

**State response (serial device):**
```json
{
  "type": "state",
  "device": "Meca500",
  "deviceType": "serial",
  "joints": [0, -30, 60, 0, 45, 90],
  "eePosition": [190.0, 0.0, 308.0],
  "eeOrientation": [0.0, 0.0, 0.0],
  "mode": "FK",
  "ikError": null,
  "collisionEnabled": true,
  "collision": false,
  "collisions": []
}
```

**State response (hexapod):**
```json
{
  "type": "state",
  "device": "Hexapod",
  "deviceType": "hexapod",
  "platformPose": [0, 0, 10, 3, 0, 0],
  "legLengths": [150.12, 150.12, 150.12, 150.12, 150.12, 150.12],
  "platformPosition": [0.0, 0.0, 10.0]
}
```

### Custom Client Example (Python)

```python
import asyncio, json, websockets

async def main():
    # Target a specific viewer tab by session ID (omit ?session=... to broadcast to all)
    async with websockets.connect("ws://localhost:8080/ws?session=ab12cd34") as ws:
        # Set joint angles
        await ws.send(json.dumps({"cmd": "setJoints", "angles": [0, -30, 60, 0, 45, 90]}))
        state = json.loads(await ws.recv())
        print(state["eePosition"])

        # Set a single joint by index
        await ws.send(json.dumps({"cmd": "setSingleJoint", "index": 1, "angle": -45}))

        # Multi-device: add a second device and position it
        await ws.send(json.dumps({"cmd": "addDevice", "config": "i16_config.json"}))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "setDeviceOrigin", "device": "i16", "position": [500, 0, 0]}))

        # Hexapod control
        await ws.send(json.dumps({"cmd": "addDevice", "config": "hexapod_config.json"}))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "setPlatformPose", "device": "Hexapod", "pose": [0, 0, 10, 3, 0, 0]}))

        # Camera: snap to front view
        await ws.send(json.dumps({"cmd": "snapCamera", "view": "front"}))

        # World-to-local coordinate transform
        await ws.send(json.dumps({"cmd": "worldToLocal", "position": [100, 0, 50], "orientation": [0, 0, 90]}))
        result = json.loads(await ws.recv())
        print(result["position"], result["orientation"])

        # Enable collision detection
        await ws.send(json.dumps({"cmd": "setCollision", "enabled": True}))

asyncio.run(main())
```

## Deployment

### Docker

```bash
docker build -t robot-visualisation .
docker run -p 8080:8080 robot-visualisation
```

### Kubernetes (Helm)

```bash
helm install robot-vis ./helm
```

See `helm/values.yaml` for configuration.

## Project Structure

```
threejs_scene.html       HTML shell — loads viewer.css and js/main.js
viewer.css               All viewer styles
js/
  main.js                Entry point — animate loop, event handlers, initialisation
  state.js               Shared mutable state (scene, cameras, controls, devices)
  scene.js               Three.js scene setup, cameras, lights, ground, nav gizmo
  device.js              Device loading, GLB import, slider/IK sync
  kinematics.js          FK, IK solver, kappa geometry math
  hexapod.js             Stewart platform loader, IK solver, leg positioning
  panel.js               Control panel UI, device list, parent dropdowns
  stl.js                 Mesh import/export, primitives, duplication, IndexedDB persistence
  storage.js             IndexedDB persistence for scene auto-save and VR anchors
  collision.js           BVH-accelerated collision detection (Web Worker + main-thread fallback)
  collision-worker.js    Background thread for collision math
  vr.js                  WebXR VR support, Meta Quest controllers, passthrough, anchors
  websocket.js           WebSocket client for remote control API + bridge status
server.py                WebSocket + HTTP server (aiohttp) with optional HTTPS
robot_ipython.py         IPython remote control client (any device)
meca500_bridge.py        Real robot bridge — VR/viewer to physical Meca500 via mecademicpy
epics_bridge.py          EPICS bridge — VR/viewer to Meca500 via pvAccess IOC velocity-mode PVs
import_robot.py          Blender import script — extracts armature to config JSON + GLB
import_hexapod.py        Blender import script for hexapod/Stewart platforms
install_dependencies.py  Install Python dependencies (websockets, ipython, numpy, etc.)
GNKinematics/            Python forward/inverse kinematics library
RobotDefinitions.py      Robot DH / geometry parameters for GNKinematics
RemoteAPI.zip            Bundled client (IPython client + GNKinematics + RobotDefinitions)
meca500_config.json      Meca500 R3 device config
hexapod_config.json      Hexapod (Stewart platform) device config
i16_config.json          i16 diffractometer device config
i19_config.json          i19 kappa diffractometer device config
gp225_config.json        Yaskawa GP225 device config
gp280_config.json        Yaskawa GP280 device config
gp180_config.json        Yaskawa GP180-120 device config
robot_scene.glb          Meca500 GLB model
hexapod_scene.glb        Hexapod GLB model
i16_scene.glb            i16 diffractometer GLB model
i19_scene.glb            i19 kappa diffractometer GLB model
gp225_scene.glb          Yaskawa GP225 GLB model
gp280_scene.glb          Yaskawa GP280 GLB model
gp180_scene.glb          Yaskawa GP180-120 GLB model
Dockerfile               Multi-stage container build
helm/                    Kubernetes Helm chart
```

## IK Solver

The viewer uses a 6xN geometric Jacobian with damped least-squares (DLS):

- **Position error**: difference between target and end-effector world position
- **Orientation error**: rotation vector from quaternion error (target x current^-1)
- **Convention**: ZYX Euler angles (alpha=Rz, beta=Ry, gamma=Rx)
- Orientation is weighted at 0.3x relative to position to prioritise reach accuracy
- For N < 6 joints: underdetermined for full 6-DOF; for N > 6: redundancy handled naturally by DLS

Hexapod (Stewart platform) IK uses a Newton-Raphson iterative solver on the leg-length equations.
