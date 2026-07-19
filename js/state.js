// ============================================================
// js/state.js — all shared mutable globals
// ============================================================
// All values start as undefined/null and are initialised by
// scene.js (for Three.js objects) or by direct assignment from
// other modules via the setter helpers below.

// Three.js core
export let scene         = null;
export let camera        = null;
export let renderer      = null;
export let labelRenderer = null;

// Cameras
export let orthoCamera  = null;
export let activeCamera = null;
export let orthoOn      = false;

// Controls
export let orbitControls           = null;
export let transformControls       = null;
export let stlTransformControls    = null;
export let deviceTransformControls = null;

// Multi-device registry
export const devices = [];
export let activeDevice    = null;
export let deviceIdCounter = 0;

// UI flags
export let labelsOn         = false;
export let originsOn        = false;
export let moveDeviceActive = false;

// Floor
export let floorSize = 2;

// Collision
export let collisionEnabled = false;
export let lastCollisions   = [];

// STL import
export const importedSTLs = [];
export let stlColorIdx    = 0;

// STL selection
export let selectedSTL      = null;
export let selectedListItem = null;
export let lockAspect       = false;
export let stlSelectable    = true;

// WebSocket
export let ws               = null;
export let wsReconnectTimer = null;

// ============================================================
// Setters — used when a let-export must be reassigned from
// another module (ES modules cannot reassign a foreign binding)
// ============================================================

export function initCoreObjects(s, c, r, lr) {
  scene = s; camera = c; renderer = r; labelRenderer = lr;
}

export function initCameras(ortho, active) {
  orthoCamera = ortho; activeCamera = active;
}

export function initControls(orbit, transform, stlTransform, deviceTransform) {
  orbitControls          = orbit;
  transformControls      = transform;
  stlTransformControls   = stlTransform;
  deviceTransformControls = deviceTransform;
}

export function setActiveCamera(cam)  { activeCamera = cam; }
export function setOrthoOn(v)         { orthoOn = v; }

export function setActiveDevice(dev)  { activeDevice = dev; }
export function incrementDeviceId()   { return 'dev_' + (deviceIdCounter++); }
export function resetDeviceIdCounter() { deviceIdCounter = 0; }

export function setLabelsOn(v)          { labelsOn = v; }
export function setOriginsOn(v)         { originsOn = v; }
export function setMoveDeviceActive(v)  { moveDeviceActive = v; }

export function setFloorSize(v)         { floorSize = v; }

export function setCollisionEnabled(v)  { collisionEnabled = v; }
export function setLastCollisions(arr)  { lastCollisions = arr; }

export function setStlColorIdx(v)       { stlColorIdx = v; }
export function setSelectedSTL(e)       { selectedSTL = e; }
export function setSelectedListItem(i)  { selectedListItem = i; }
export function setLockAspect(v)        { lockAspect = v; }
export function setStlSelectable(v)     { stlSelectable = v; }

export function setWs(socket)           { ws = socket; }
export function setWsReconnectTimer(t)  { wsReconnectTimer = t; }

// VR
export let vrActive       = false;
export let vrRig          = null;
export let passthroughOn  = false;

export function setVRActive(v)       { vrActive = v; }
export function setVRRig(rig)        { vrRig = rig; }
export function setPassthroughOn(v)  { passthroughOn = v; }

// Real robot bridge
export let bridgeActive = false;
export function setBridgeActive(v) { bridgeActive = v; }
