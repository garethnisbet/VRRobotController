# VR Robot Controller — Meca500

Interactive 3D visualisation, VR teleoperation, and remote control for the Mecademic Meca500 R3. Config-driven — the viewer loads the robot from a JSON config and GLB model. Includes forward and inverse kinematics, mesh import, collision detection, VR support (Meta Quest), real robot teleoperation, and a remote control API.

## Supported Devices

| Device | Config | Type | Description |
|--------|--------|------|-------------|
| Meca500 R3 | `meca500_config.json` | 6-DOF serial | Compact industrial manipulator |

Other devices can be added from Blender scenes using `import_robot.py` (serial robots) or `import_hexapod.py` (Stewart platforms). See [Adding New Devices](#adding-new-devices).

## Features

- **Config-driven viewer** — a single generic `threejs_scene.html` viewer loads any device via JSON config
- **Multi-device scene** — load multiple devices simultaneously from the add-device dropdown; click a device in the list or click its mesh to switch active device
- **Serial and parallel kinematics** — supports serial manipulators (FK/IK via Jacobian); Stewart platform hexapod kinematics remain available for imported configs
- **Device renaming** — double-click a device name in the device list to rename it
- **Device origin transform** — move and rotate device origins with translate/rotate gizmo modes; World/Local space toggle for gizmo axis alignment; numeric X/Y/Z (mm) and Rx/Ry/Rz (deg) inputs for precise positioning, synced live with the gizmo
- **Device parenting** — parent a device to a link on another device so it follows the kinematic chain
- **Auto-fit camera** — camera automatically frames the loaded model on startup
- **Forward Kinematics** — joint angle sliders for all movable joints (fixed kinematic links are hidden)
- **Inverse Kinematics** — 6-DOF damped least-squares solver (position + ZYX Euler orientation)
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
- **VR support** — WebXR-based VR with Meta Quest controller interaction, passthrough toggle, and persistent anchors for drift correction
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

Self-collision between device links uses kinematic adjacency analysis — links sharing the same joint or connected through a parent-child relationship are skipped. Links on separate branches are always checked.

## VR Support

The viewer supports WebXR for Meta Quest headsets. VR features:

- **Controller interaction** — grip to grab the IK target; thumbstick for locomotion
- **Passthrough toggle** — switch between VR passthrough and rendered background
- **Persistent anchors** — saves VR anchor to IndexedDB for drift correction across sessions
- **E-Stop** — A/X button triggers E-Stop when the real robot bridge is active; otherwise resets to home
- **Exit VR** — button in the panel or B/Y on controller

VR over network requires HTTPS. Use `python server.py --ssl` to enable it.

### Headless Mode

By default the checks are driven from the animation loop: they run on every 6th drawn frame, so the rate is capped at about a sixth of the display refresh — and because the viewer only draws on demand, a pose change that triggers just one or two frames can be missed entirely.

Toggle **Headless Collision: ON/OFF** (or `{"cmd": "setCollisionHeadless", "enabled": true}` / `robot.collision_headless(True)`) to take the checks off the render loop. The next pass then starts as soon as the previous result lands, so throughput is bound by the collision computation rather than by vsync, and checks continue while the tab is in the background, where `requestAnimationFrame` is suspended.

- The achieved rate is shown next to the collision readout (`Collisions: none · 240 Hz`).
- A pass is skipped when nothing has moved — a fingerprint of every participating mesh's world matrix is compared first — so a static scene reads `idle` instead of burning a core.
- Highlights and the info panel are only rewritten when the set of colliding pairs actually changes.
- It samples the current pose; if a device is driven faster than a pass completes, intermediate poses are still skipped.

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
| `robot.get_device_pos('Meca500')` | `d = robot.get_device_pos()` | Any device: pos, rot, joints, EE |
| `robot.get_obj_pos('cube_1')` | `o = robot.get_obj_pos(0)` | Any object: pos, rot, scale, BB |

**Device commands:**

| Space-separated | Python | Description |
|---------|---------|-------------|
| `state` | `robot.state()` | Request current device state |
| `devices` | `robot.devices()` | List all loaded devices |
| `device Meca500` | `robot.device('Meca500')` | Switch active device by name |
| `sessions` | `robot.sessions()` | List viewer session IDs |
| `home` | `robot.home()` | All joints to 0 |
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
| `robot.devpose([x,y,z], [rx,ry,rz])` | Set device origin position (mm) and/or rotation (deg); also accepts a single 6-element list |
| `robot.devtranslate(dx, dy, dz, space='parent')` | Translate device origin by delta (mm) in parent/local/world frame |
| `robot.devrotate(rx, ry, rz, space='parent')` | Rotate device origin by delta (deg) in parent/local/world frame |

`devpose` sets the absolute position and/or rotation of the device origin in its parent frame (the same values shown in the numeric inputs when **Move Device Origin** is active). Both arguments are optional — pass only `position` or only `rotation` to change one without affecting the other.

```python
r.devpose([100, 0, 0])                  # position only
r.devpose([100, 0, 0], [0, 0, 90])      # position + rotation
r.devpose([100, 0, 0, 0, 0, 90])        # full pose as one list
r.devpose(rotation=[0, 0, 90])          # rotation only
r.devpose([0, 0, 0], device='Meca500')  # specific device
```

**Coordinate transform:**

| Python | Description |
|--------|-------------|
| `robot.worldToLocal([x,y,z,rx,ry,rz])` | Transform a world-frame pose into the active device's local frame |
| `robot.worldToLocal([x,y,z], [rx,ry,rz])` | Same, with separate position and orientation arguments |
| `robot.worldToLocal(pos, ori, device='Meca500')` | Transform relative to a specific device |

`worldToLocal` converts a world-frame pose (position in mm, orientation as XYZ intrinsic Euler angles in degrees) into the coordinate frame of a device's origin. This is useful when a robot is mounted at an arbitrary position/rotation and you need to express a world target in the robot's own coordinate system — for example, to feed into an IK solver that expects local coordinates.

The orientation convention is `R = Rx(α)·Ry(β)·Rz(γ)` in a right-handed Z-up frame (X right, Y into scene, Z up). Note that Y points *into* the scene, which is the negation of the viewer's Y readback from `dev_pose()`.

When called with a 6-element list, returns a 6-element list `[x, y, z, rx, ry, rz]`. When called with separate position and orientation arguments, returns a `(position, orientation)` tuple.

```python
# Robot mounted at world position [500, 200, 0] with 45° rotation
meca500 [1]: robot.worldToLocal([600, 200, 300, 0, 0, 0])
# → [70.71, 70.71, 300.0, 0.0, 0.0, -45.0]   (position and orientation in robot's local frame)

meca500 [2]: p, o = robot.worldToLocal([600, 200, 300], [0, 0, 0])
# p = [70.71, 70.71, 300.0], o = [0.0, 0.0, -45.0]

meca500 [3]: robot.worldToLocal([600, 200, 300], [0, 0, 0], device='Meca500')
# Transform relative to a named device
```

**Motion planning commands:**

| Space-separated | Python | Description |
|---------|---------|-------------|
| `plan --start 0 0 0 --end 45 90 0` | `robot.plan([0,0,0], [45,90,0])` | Path plan between two poses |
| `scan theta 0 90 5` | `robot.scan(('theta', 0, 90, 5))` | 1D scan |
| `scan theta 0 90 5 phi 0 30 2` | `robot.scan(('theta',0,90,5), ('phi',0,30,2))` | 2D grid scan |
| `scan theta 0 90 5 phi 0 1` | `robot.scan(('theta',0,90,5), ('phi',0,1))` | Coupled scan |
| `scan DevA:J1 0 50 5 DevB:J1 0 30 5` | `robot.scan(('DevA:J1',0,50,5), ('DevB:J1',0,30,5))` | Multi-device scan |
| `scan ee:x 150 250 10` | `robot.scan(('ee:x', 150, 250, 10))` | Cartesian end-effector scan (Python IK) |
| `scan ee:x 150 250 5 ee:y -50 50 5` | `robot.scan(('ee:x',150,250,5), ('ee:y',-50,50,5))` | Cartesian grid/coupled scan |
| `scan ee:z 200 400 10 --space world` | `robot.scan(('ee:z',200,400,10), space='world')` | Cartesian scan in world frame |
| `scan ee:x ee:y ee:z waypoints` | `robot.scan('ee:x','ee:y','ee:z', waypoints)` | Cartesian array scan (rows = poses) |
| `scan Meca500:ee:z 200 400 10` | `robot.scan(('Meca500:ee:z', 200, 400, 10))` | Cartesian scan on a named device |
| `scan Meca500:ee:z 200 400 10 Robot2:ee:x 150 250 10` | `robot.scan(('Meca500:ee:z',200,400,10), ('Robot2:ee:x',150,250,10))` | Multi-device Cartesian scan |
| `scan Meca500:ee:y 354 400 10 Robot2:J1 0 120 10` | `robot.scan(('Meca500:ee:y',354,400,10), ('Robot2:J1',0,120,10))` | Mixed Cartesian + joint scan |
| `scan Meca500 scanpoints` | `robot.scan('Meca500', scanpoints)` | Full-vector array scan (device name expands to all joints) |
| `scan Meca500 Robot2 combined_pts` | `robot.scan('Meca500', 'Robot2', combined_pts)` | Multi-device vector scan (cols = joints of each device) |
| `scan robot1 robot2 my_func()` | `robot.scan('robot1', 'robot2', my_func)` | Multi-device vector scan with callable |
| — | `robot.scan(('@Cube:tx', 0, 100, 10))` | Object translation scan |
| — | `robot.scan(('@Cube:tx',0,100,10), space='world')` | Object scan in world coords |
| — | `robot.scan(('J1',0,90,5), ('@Cube:tz',0,50,5))` | Mixed joint + object scan |

Object scan axes use `@ObjectName:component` syntax where component is `tx`, `ty`, `tz`, `rx`, `ry`, or `rz`. The `space` parameter (`'local'` or `'world'`) controls the coordinate frame for object transforms (default: `'local'`).

Vector scans accept a device name in place of listing all its joints: `robot.scan('Meca500', waypoints)` expands to all joints on that device. Multiple devices can be combined: `robot.scan('Meca500', 'Robot2', combined_pts)`.

Cartesian end-effector axes use an `ee:` prefix — `x`, `y`, `z` (mm) and `a`, `b`, `g` (ZYX Euler degrees). Each target pose is solved to joint angles by the analytical Python IK (`GNKinematics`) and streamed as joint waypoints, so the on-screen pose matches the analytical solution. Start/end/step are absolute coordinates and unlisted axes hold their current value. `--space`/`space=` selects the frame: `local` (default, robot base frame) or `world` (converted per waypoint via `worldToLocal`, so the end-effector tracks world axes regardless of how the device is mounted). Supported on the Meca500 (and any imported robot with a `GNKinematics` definition — see [Adding Robot Kinematics Definitions](#adding-robot-kinematics-definitions)); cannot be mixed with joint, virtual, or object axes in the same scan.

By default an `ee:` scan targets the active device. Prefix the axis with a device name — `Device:ee:<axis>` (e.g. `Meca500:ee:z`) — to target a specific device, or list several to scan multiple arms in one command. Each device is solved with its own IK and base pose, and the per-step joint solutions are streamed together (grid axes form a product across devices, coupled axes lock-step with the primary), mirroring multi-device joint scans. Device-prefixed axes are supported in the range form only; for an array scan, switch to the device first with `robot.device('Name')`.

Cartesian `ee:` axes can be combined with ordinary joint axes on *other* devices in the same scan (e.g. `scan Meca500:ee:y 354 400 10 Robot2:J1 0 120 10`) — the Cartesian device is IK-solved while the joint axis is set directly, and all devices step together. A single device cannot mix `ee:` and joint axes, and `ee:` cannot be combined with object (`@`) or virtual (`v:`) axes.

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
| `collision headless on` | `robot.collision_headless(True)` | Run checks off the render loop (uncapped rate) |
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

### API Protocol (JSON over WebSocket)

All positions are in mm, angles in degrees, using Z-up robot convention. Most commands accept an optional `"device"` field to target a specific device by name or ID; if omitted, the active device is used.

**Device management:**
```json
{"cmd": "getState"}
{"cmd": "listDevices"}
{"cmd": "getDevice"}
{"cmd": "addDevice", "config": "meca500_config.json"}
{"cmd": "removeDevice", "device": "Meca500"}
{"cmd": "renameDevice", "name": "MyRobot"}
{"cmd": "setActiveDevice", "device": "Meca500"}
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
{"cmd": "setCollisionHeadless", "enabled": true}
{"cmd": "getCollisions"}
```

`setCollisionHeadless` decouples the checks from the render loop so their rate is not capped by the display refresh — see [Headless Mode](#headless-mode).

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

**State response (serial robot):**
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

        # Multi-device: add a second robot and position it beside the first
        await ws.send(json.dumps({"cmd": "addDevice", "config": "meca500_config.json"}))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "setDeviceOrigin", "device": "Meca500 2", "position": [500, 0, 0]}))

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
  hexapod.js             Hexapod loader, Damped Track IK, FK solver, platform sync
  kinematics.js          FK, IK solver, kappa geometry math
  panel.js               Control panel UI, device list, parent dropdowns
  stl.js                 Mesh import/export, primitives, duplication, IndexedDB persistence
  storage.js             IndexedDB persistence for scene auto-save and VR anchors
  collision.js           BVH-accelerated collision detection (Web Worker + main-thread fallback)
  collision-worker.js    Background thread for collision math
  vr.js                  WebXR VR support, Meta Quest controllers, passthrough, anchors
  websocket.js           WebSocket client for remote control API + bridge status
import_robot.py          Blender import script — extracts serial robot armature to config JSON + GLB
import_hexapod.py        Blender import script — extracts hexapod (Damped Track legs) to config JSON + GLB
server.py                WebSocket + HTTP server (aiohttp) with optional HTTPS
robot_ipython.py         IPython remote control client (any device)
meca500_bridge.py        Real robot bridge — VR/viewer to physical Meca500 via mecademicpy
epics_bridge.py          EPICS bridge — VR/viewer to Meca500 via pvAccess IOC velocity-mode PVs
install_dependencies.py  Install Python dependencies (websockets, ipython, numpy, etc.)
GNKinematics/            Python forward/inverse kinematics library
RobotDefinitions.py      Robot DH / geometry parameters for GNKinematics
RemoteAPI.zip            Bundled client (IPython client + GNKinematics + RobotDefinitions)
meca500_config.json      Meca500 R3 device config
meca500_scene.glb        Meca500 GLB model
Dockerfile               Multi-stage container build
helm/                    Kubernetes Helm chart
```

## IK Solver (Serial Robots)

The viewer uses a 6xN geometric Jacobian with damped least-squares (DLS):

- **Position error**: difference between target and end-effector world position
- **Orientation error**: rotation vector from quaternion error (target x current^-1)
- **Convention**: ZYX Euler angles (alpha=Rz, beta=Ry, gamma=Rx)
- Orientation is weighted at 0.3x relative to position to prioritise reach accuracy
- For N < 6 joints: underdetermined for full 6-DOF; for N > 6: redundancy handled naturally by DLS
