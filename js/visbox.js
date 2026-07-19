// ============================================================
// js/visbox.js — visibility clip box (non-destructive view aid)
// ------------------------------------------------------------
// A blue oriented box that hides everything inside or outside it,
// so the internal structure of point clouds / splats can be
// inspected without deleting anything. Ported from LidarStudio,
// minus the editing operations (delete / crop / region limits).
// The box drives the shader clip in stl.js via setVisibilityClip();
// it only affects point clouds and splats, never robot/device meshes.
// ============================================================
import * as THREE from 'three';
import { TransformControls } from 'three/addons/controls/TransformControls.js';

import * as State from './state.js';
import { setVisibilityClip } from './stl.js';

let visGizmo = null, visBox = null, visModeOutside = false;
// Pose snapshotted when the box is cleared, so re-placing it brings it
// back at the same position/rotation/size instead of re-fitting.
let lastPose = null;

// The box has its own gizmo so it never disturbs the STL selection or
// the device-origin controls.
function ensureVisGizmo() {
  if (visGizmo) return visGizmo;
  const g = new TransformControls(State.activeCamera || State.camera, State.renderer.domElement);
  g.setSize(0.7);
  g.addEventListener('dragging-changed', (e) => { State.orbitControls.enabled = !e.value; });
  g.addEventListener('change', () => { pushVisClip(); State.requestRender(); });
  State.scene.add(g);
  State.setVisBoxControls(g);
  visGizmo = g;
  return g;
}

function pushVisClip() {
  if (!visBox) return;
  visBox.updateWorldMatrix(true, false);
  setVisibilityClip({ enabled: true, mode: visModeOutside ? 'outside' : 'inside',
    matrix: Array.from(visBox.matrixWorld.elements) });
}

export function visBoxActive() { return !!visBox; }

export function setVisBoxMode(mode) {
  if (!visGizmo) return;
  visGizmo.setMode(mode);
  for (const [id, m] of [['visBoxMove', 'translate'], ['visBoxRotate', 'rotate'], ['visBoxScale', 'scale']]) {
    document.getElementById(id)?.classList.toggle('active', m === mode);
  }
  State.requestRender();
}

export function removeVisBox() {
  if (visGizmo && visGizmo.object) visGizmo.detach();
  if (visBox) {
    lastPose = {
      position: visBox.position.clone(),
      quaternion: visBox.quaternion.clone(),
      scale: visBox.scale.clone(),
    };
    State.scene.remove(visBox);
    visBox.traverse(o => { o.geometry?.dispose?.(); o.material?.dispose?.(); });
    visBox = null;
  }
  setVisibilityClip({ enabled: false });
  State.requestRender();
}

// Bounds the box starts from: the selected object if there is one,
// otherwise the union of all visible point clouds and splats (the only
// objects the clip affects).
function _startBounds() {
  if (State.selectedSTL && State.selectedSTL.mesh) {
    const b = new THREE.Box3().setFromObject(State.selectedSTL.mesh);
    if (!b.isEmpty()) return b;
  }
  const b = new THREE.Box3();
  const item = new THREE.Box3();
  for (const e of State.importedSTLs) {
    if (!(e.isPointCloud || e.isSplat) || !e.mesh.visible) continue;
    item.setFromObject(e.mesh);
    if (!item.isEmpty()) b.union(item);
  }
  return b.isEmpty() ? null : b;
}

export function placeVisBox() {
  // Re-placing while a box is active re-fits it to the object bounds;
  // placing after a Clear restores the cleared box's pose instead.
  const refit = !!visBox;
  removeVisBox();                       // snapshots lastPose from the live box
  const restore = !refit && lastPose;
  let c = null, s = null;
  if (!restore) {
    const b = _startBounds();
    if (!b) return;
    c = b.getCenter(new THREE.Vector3());
    s = b.getSize(new THREE.Vector3());
  }
  const geo = new THREE.BoxGeometry(1, 1, 1);
  visBox = new THREE.Mesh(geo, new THREE.MeshBasicMaterial(
    { color: 0x33aaff, transparent: true, opacity: 0.10, depthWrite: false }));
  visBox.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo),
    new THREE.LineBasicMaterial({ color: 0x33aaff })));
  if (restore) {
    visBox.position.copy(lastPose.position);
    visBox.quaternion.copy(lastPose.quaternion);
    visBox.scale.copy(lastPose.scale);
  } else {
    visBox.position.copy(c);
    // Start at half the object's size so there's something to reveal.
    visBox.scale.set(Math.max(s.x * 0.5, 0.05), Math.max(s.y * 0.5, 0.05), Math.max(s.z * 0.5, 0.05));
  }
  State.scene.add(visBox);
  ensureVisGizmo().attach(visBox);
  setVisBoxMode('translate');
  pushVisClip();
  State.requestRender();
}

// Show the controls only while there is something the clip can act on
// (or a box still to clear). Called on each drawn frame from the animate
// loop, like the splat/point-cloud clip rows.
export function updateVisBoxUI() {
  const section = document.getElementById('visBoxSection');
  if (!section) return;
  const relevant = visBox || State.importedSTLs.some(e => e.isPointCloud || e.isSplat);
  const display = relevant ? 'block' : 'none';
  if (section.style.display !== display) section.style.display = display;
}

export function initVisBoxUI() {
  const modeBtn = document.getElementById('visBoxModeBtn');
  const updateModeLabel = () => {
    modeBtn.textContent = `Showing: ${visModeOutside ? 'outside' : 'inside'}`;
  };
  document.getElementById('visBoxPlaceBtn').addEventListener('click', placeVisBox);
  document.getElementById('visBoxMove').addEventListener('click', () => setVisBoxMode('translate'));
  document.getElementById('visBoxRotate').addEventListener('click', () => setVisBoxMode('rotate'));
  document.getElementById('visBoxScale').addEventListener('click', () => setVisBoxMode('scale'));
  modeBtn.addEventListener('click', () => {
    visModeOutside = !visModeOutside;
    updateModeLabel();
    pushVisClip();
  });
  document.getElementById('visBoxClearBtn').addEventListener('click', removeVisBox);
}
