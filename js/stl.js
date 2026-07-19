// ============================================================
// js/stl.js — STL/OBJ/PLY/GLB import, file-based scene save/load,
//             primitive creation, STL list UI,
//             selection, transform mode, parent assignment
// ============================================================
import * as THREE from 'three';
import { STLLoader }  from 'three/addons/loaders/STLLoader.js';
import { OBJLoader }  from 'three/addons/loaders/OBJLoader.js';
import { MTLLoader }  from 'three/addons/loaders/MTLLoader.js';
import { PLYLoader }  from 'three/addons/loaders/PLYLoader.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';
import { MeshBVH } from 'three-mesh-bvh';

import * as State from './state.js';

// ============================================================
// Loaders & colour palette
// ============================================================
const stlLoader         = new STLLoader();
const objLoader         = new OBJLoader();
const plyLoader         = new PLYLoader();
const gltfImportLoader  = new GLTFLoader();

export const stlColors = [0x44aaff, 0xff6644, 0x44ff88, 0xffaa22, 0xcc44ff, 0xff4488, 0x22ddcc, 0xaadd22];

// Helper: consume the next color slot and return the color
function nextColor() {
  const idx = State.stlColorIdx;
  State.setStlColorIdx(idx + 1);
  return stlColors[idx % stlColors.length];
}

// ============================================================
// Scene state save/load (file-based)
// ============================================================

function _arrayBufferToBase64(buf) {
  const bytes = new Uint8Array(buf);
  let binary = '';
  for (let i = 0; i < bytes.length; i++) binary += String.fromCharCode(bytes[i]);
  return btoa(binary);
}

function _base64ToArrayBuffer(b64) {
  const binary = atob(b64);
  const bytes = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);
  return bytes.buffer;
}

// Convert runtime parentLink (devId:linkName) to stable index-based format (devIndex:linkName)
function _parentLinkToStable(parentLink) {
  if (!parentLink || !parentLink.includes(':')) return parentLink;
  const [devId, linkName] = parentLink.split(':', 2);
  const devIdx = State.devices.findIndex(d => d.id === devId);
  if (devIdx < 0) return null;
  return devIdx + ':' + linkName;
}

function _buildDevicesPayload() {
  return State.devices.map((dev) => {
    const entry = {
      configFile: dev.configFile,
      name: dev.name,
      jointAngles: [...dev.jointAngles],
      position: [dev.rootGroup.position.x, dev.rootGroup.position.y, dev.rootGroup.position.z],
      rotation: [dev.rootGroup.rotation.x, dev.rootGroup.rotation.y, dev.rootGroup.rotation.z],
      parentLink: _parentLinkToStable(dev.parentLink),
    };
    if (dev.type === 'hexapod') entry.platformPose = [...dev.platformPose];
    return entry;
  });
}

function _buildCameraPayload() {
  const cam = State.camera;
  const ctrl = State.orbitControls;
  return {
    position: [cam.position.x, cam.position.y, cam.position.z],
    target: [ctrl.target.x, ctrl.target.y, ctrl.target.z],
  };
}

export function buildScenePayload() {
  const stls = State.importedSTLs.map(entry => {
    const m = entry.mesh;
    return {
      id: entry.stlId,
      name: entry.name,
      color: entry.color,
      opacity: entry.opacity,
      buffer: _arrayBufferToBase64(entry._buffer),
      fileType: entry.fileType || 'stl',
      isPointCloud: entry.isPointCloud || false,
      position: [m.position.x, m.position.y, m.position.z],
      rotation: [m.rotation.x, m.rotation.y, m.rotation.z],
      scale: [m.scale.x, m.scale.y, m.scale.z],
      visible: m.visible,
      parentLink: _parentLinkToStable(entry.parentLink),
    };
  });
  return { version: 1, devices: _buildDevicesPayload(), stls, camera: _buildCameraPayload(), floorSize: State.floorSize };
}

export function buildScenePayloadForDB() {
  const stls = State.importedSTLs.map(entry => {
    const m = entry.mesh;
    return {
      id: entry.stlId,
      name: entry.name,
      color: entry.color,
      opacity: entry.opacity,
      buffer: entry._buffer,
      fileType: entry.fileType || 'stl',
      isPointCloud: entry.isPointCloud || false,
      position: [m.position.x, m.position.y, m.position.z],
      rotation: [m.rotation.x, m.rotation.y, m.rotation.z],
      scale: [m.scale.x, m.scale.y, m.scale.z],
      visible: m.visible,
      parentLink: _parentLinkToStable(entry.parentLink),
    };
  });
  return { version: 1, devices: _buildDevicesPayload(), stls, camera: _buildCameraPayload(), floorSize: State.floorSize };
}

export async function exportSceneState() {
  const payload = buildScenePayload();
  console.log('[Save Scene]', payload.devices.length, 'devices,', payload.stls.length, 'objects');
  for (const d of payload.devices) console.log('  device:', d.name, 'joints:', d.jointAngles.map(a => (a * 180 / Math.PI).toFixed(1)));
  for (const s of payload.stls) console.log('  object:', s.name, 'pos:', s.position, 'rot:', s.rotation, 'parent:', s.parentLink);

  const json = JSON.stringify(payload, null, 2);
  const blob = new Blob([json], { type: 'application/json' });

  if (window.showSaveFilePicker) {
    try {
      const handle = await window.showSaveFilePicker({
        suggestedName: 'scene_state.json',
        types: [{
          description: 'Scene JSON',
          accept: { 'application/json': ['.json'] },
        }],
      });
      const writable = await handle.createWritable();
      await writable.write(blob);
      await writable.close();
      return;
    } catch (e) {
      if (e.name === 'AbortError') return; // user cancelled
    }
  }
  // Fallback for browsers without File System Access API
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = 'scene_state.json';
  a.click();
  URL.revokeObjectURL(url);
}

export async function importSceneState(file) {
  const text = await file.text();
  const data = JSON.parse(text);
  if (!data.version || !data.stls) throw new Error('Invalid scene file');
  return data;
}

// Convert stable index-based parentLink (devIndex:linkName) back to runtime (devId:linkName)
// Also handles legacy format (devId:linkName) from older save files
function _parentLinkFromStable(parentLink) {
  if (!parentLink || !parentLink.includes(':')) return parentLink;
  const [idxStr, linkName] = parentLink.split(':', 2);
  // Try index-based format first (e.g. "0:L1")
  const devIdx = parseInt(idxStr, 10);
  if (!isNaN(devIdx) && devIdx >= 0 && devIdx < State.devices.length) {
    return State.devices[devIdx].id + ':' + linkName;
  }
  // Legacy format (e.g. "dev_0:L1") — search by link name across all devices
  for (const dev of State.devices) {
    if (dev.linkToJoint[linkName] !== undefined ||
        (dev.parentGroups && dev.parentGroups[linkName])) {
      return dev.id + ':' + linkName;
    }
  }
  return null;
}

export async function restoreSTLsFromState(records) {
  console.log('[Load Scene v3] Restoring', records.length, 'objects — two-phase restore');

  // ── Phase 1: create meshes WITHOUT transforms (default positions) ──
  const created = [];
  for (const rec of records) {
    const buffer = rec.buffer instanceof ArrayBuffer ? rec.buffer : _base64ToArrayBuffer(rec.buffer);
    let entry = null;
    const fileType = rec.fileType || 'stl';
    if (fileType === 'stl') {
      entry = createSTLFromBuffer(buffer, rec.name, rec.color, rec.id, null);
    } else if (fileType === 'ply') {
      const geometry = plyLoader.parse(buffer);
      if (rec.isPointCloud || _isPLYPointCloud(buffer)) {
        entry = _addPointsToScene(geometry, buffer, rec.name, rec.color, rec.id, null);
      } else {
        geometry.computeVertexNormals();
        entry = _addMeshToScene(geometry, buffer, 'ply', rec.name, rec.color, rec.id, null);
      }
    } else if (fileType === 'obj') {
      const objText = new TextDecoder().decode(buffer);
      const group = objLoader.parse(objText);
      const geometry = _mergeObject3D(group);
      entry = _addMeshToScene(geometry, buffer, 'obj', rec.name, rec.color, rec.id, null);
    } else if (fileType === 'glb') {
      try {
        const gltf = await new Promise((resolve, reject) =>
          gltfImportLoader.parse(buffer, '', resolve, reject));
        const geometry = _mergeObject3D(gltf.scene);
        entry = _addMeshToScene(geometry, buffer, 'glb', rec.name, rec.color, rec.id, null);
      } catch (e) {
        console.warn('Failed to restore GLB mesh:', rec.name, e);
      }
    }
    created.push({ rec, entry });
  }

  // ── Phase 2: apply transforms and parent links explicitly ──
  console.log('[Load Scene v3] Phase 2: applying transforms & parents');
  for (const { rec, entry } of created) {
    if (!entry) continue;
    const m = entry.mesh;

    // Apply saved transforms
    if (rec.position) m.position.set(rec.position[0], rec.position[1], rec.position[2]);
    if (rec.rotation) m.rotation.set(rec.rotation[0], rec.rotation[1], rec.rotation[2]);
    if (rec.scale)    m.scale.set(rec.scale[0], rec.scale[1], rec.scale[2]);
    if (rec.visible !== undefined) m.visible = rec.visible;
    if (rec.opacity !== undefined) {
      m.material.opacity = rec.opacity;
      entry.opacity = rec.opacity;
    }

    // Apply parent link
    const resolvedParent = _parentLinkFromStable(rec.parentLink);
    if (resolvedParent) {
      setSTLParent(entry, resolvedParent, true);
    }

    console.log('[Restore]', rec.name,
      'pos:', [m.position.x.toFixed(4), m.position.y.toFixed(4), m.position.z.toFixed(4)],
      'rot:', [m.rotation.x.toFixed(4), m.rotation.y.toFixed(4), m.rotation.z.toFixed(4)],
      'scale:', [m.scale.x.toFixed(4), m.scale.y.toFixed(4), m.scale.z.toFixed(4)],
      'parent:', entry.parentLink,
      'meshParent:', m.parent?.name || 'Scene');
  }
}

// ============================================================
// Internal geometry helpers
// ============================================================
export function _mergeObject3D(object3D) {
  const chunks = [];
  const colorChunks = [];
  let totalVerts = 0;
  let hasNonWhite = false;
  object3D.updateWorldMatrix(true, true);
  object3D.traverse(child => {
    if (!child.isMesh || !child.geometry) return;
    let geo = child.geometry.clone();
    child.updateWorldMatrix(true, false);
    geo.applyMatrix4(child.matrixWorld);
    if (geo.index) geo = geo.toNonIndexed();
    const pos = geo.getAttribute('position');
    if (!pos || pos.count === 0) return;
    const arr = new Float32Array(pos.count * 3);
    for (let i = 0; i < pos.count; i++) {
      arr[i * 3]     = pos.getX(i);
      arr[i * 3 + 1] = pos.getY(i);
      arr[i * 3 + 2] = pos.getZ(i);
    }
    chunks.push(arr);

    const colors = new Float32Array(pos.count * 3);
    const existingColors = geo.getAttribute('color');
    if (existingColors) {
      for (let i = 0; i < pos.count; i++) {
        colors[i * 3]     = existingColors.getX(i);
        colors[i * 3 + 1] = existingColors.getY(i);
        colors[i * 3 + 2] = existingColors.getZ(i);
      }
      hasNonWhite = true;
    } else {
      const mat = child.material;
      const c = (mat && mat.color) ? mat.color : new THREE.Color(1, 1, 1);
      if (c.r < 0.99 || c.g < 0.99 || c.b < 0.99) hasNonWhite = true;
      for (let i = 0; i < pos.count; i++) {
        colors[i * 3]     = c.r;
        colors[i * 3 + 1] = c.g;
        colors[i * 3 + 2] = c.b;
      }
    }
    colorChunks.push(colors);

    totalVerts += pos.count;
  });
  const merged = new THREE.BufferGeometry();
  if (totalVerts > 0) {
    const positions = new Float32Array(totalVerts * 3);
    let offset = 0;
    for (const chunk of chunks) {
      positions.set(chunk, offset);
      offset += chunk.length;
    }
    merged.setAttribute('position', new THREE.BufferAttribute(positions, 3));

    if (hasNonWhite) {
      const colorsArr = new Float32Array(totalVerts * 3);
      let cOffset = 0;
      for (const cc of colorChunks) {
        colorsArr.set(cc, cOffset);
        cOffset += cc.length;
      }
      merged.setAttribute('color', new THREE.BufferAttribute(colorsArr, 3));
    }
  }
  merged.computeVertexNormals();
  return merged;
}

export function _isPLYPointCloud(buffer) {
  const header = new TextDecoder().decode(new Uint8Array(buffer, 0, Math.min(2048, buffer.byteLength)));
  return !header.includes('element face');
}

export function _addPointsToScene(geometry, buffer, name, color, stlId, transforms) {
  const hasVertexColors = geometry.hasAttribute('color');
  const matColor = hasVertexColors ? 0xffffff : color;
  const material = new THREE.PointsMaterial({
    color: matColor, size: 0.003, sizeAttenuation: true,
    vertexColors: hasVertexColors,
    transparent: true, opacity: 0.9,
  });
  const points = new THREE.Points(geometry, material);

  if (transforms) {
    points.position.set(...transforms.position);
    points.rotation.set(...transforms.rotation);
    points.scale.set(...transforms.scale);
    points.visible = transforms.visible;
  } else {
    const box = new THREE.Box3().setFromObject(points);
    const size = box.getSize(new THREE.Vector3());
    if (size.length() > 1) points.scale.setScalar(0.001);
  }

  State.scene.add(points);
  const box = new THREE.Box3().setFromObject(points);
  const div = document.createElement('div');
  div.className = 'mesh-label';
  div.textContent = name;
  const label = new CSS2DObject(div);
  label.visible = State.labelsOn;
  const center = box.getCenter(new THREE.Vector3());
  points.worldToLocal(center);
  label.position.copy(center);
  points.add(label);

  const entry = { mesh: points, label, name, color: matColor, opacity: material.opacity, stlId, _buffer: buffer, fileType: 'ply', isPointCloud: true, parentLink: null, importScale: points.scale.clone() };
  State.importedSTLs.push(entry);
  State.setStlColorIdx(Math.max(State.stlColorIdx, stlColors.indexOf(color) + 1));
  addSTLListItem(entry);

  if (transforms && transforms.parentLink) {
    setSTLParent(entry, transforms.parentLink, true);
  }
  return entry;
}

export function _addMeshToScene(geometry, buffer, fileType, name, color, stlId, transforms) {
  geometry.boundsTree = new MeshBVH(geometry);

  const hasVertexColors = geometry.hasAttribute('color');
  const material = new THREE.MeshStandardMaterial({
    color: hasVertexColors ? 0xffffff : color,
    metalness: 0.3, roughness: 0.6,
    transparent: true, opacity: 0.85,
    vertexColors: hasVertexColors,
  });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;

  if (transforms) {
    mesh.position.set(...transforms.position);
    mesh.rotation.set(...transforms.rotation);
    mesh.scale.set(...transforms.scale);
    mesh.visible = transforms.visible;
  } else {
    const box = new THREE.Box3().setFromObject(mesh);
    const size = box.getSize(new THREE.Vector3());
    if (size.length() > 1) mesh.scale.setScalar(0.001);
  }

  State.scene.add(mesh);
  const box = new THREE.Box3().setFromObject(mesh);
  const div = document.createElement('div');
  div.className = 'mesh-label';
  div.textContent = name;
  const label = new CSS2DObject(div);
  label.visible = State.labelsOn;
  const center = box.getCenter(new THREE.Vector3());
  mesh.worldToLocal(center);
  label.position.copy(center);
  mesh.add(label);

  const entry = { mesh, label, name, color, opacity: material.opacity, stlId, _buffer: buffer, fileType, parentLink: null, importScale: mesh.scale.clone() };
  State.importedSTLs.push(entry);
  State.setStlColorIdx(Math.max(State.stlColorIdx, stlColors.indexOf(color) + 1));
  addSTLListItem(entry);

  if (transforms && transforms.parentLink) {
    setSTLParent(entry, transforms.parentLink, true);
  }
  return entry;
}

export function createSTLFromBuffer(buffer, name, color, stlId, transforms) {
  const geometry = stlLoader.parse(buffer);
  geometry.computeVertexNormals();
  return _addMeshToScene(geometry, buffer, 'stl', name, color, stlId, transforms);
}

// ============================================================
// File loaders (wired to input events in main.js)
// ============================================================
export function loadSTLFile(file) {
  const reader = new FileReader();
  reader.onload = (e) => {
    const buffer = e.target.result;
    const baseName = file.name.replace(/\.stl$/i, '');
    const color = nextColor();
    const stlId = Date.now() + '_' + Math.random().toString(36).slice(2, 8);
    createSTLFromBuffer(buffer, baseName, color, stlId, null);
  };
  reader.readAsArrayBuffer(file);
}

export function loadOBJFile(file, mtlFile) {
  const readText = f => new Promise(resolve => {
    const r = new FileReader();
    r.onload = e => resolve(e.target.result);
    r.readAsText(f);
  });

  const doLoad = async () => {
    const text = await readText(file);
    const loader = new OBJLoader();

    if (mtlFile) {
      const mtlText = await readText(mtlFile);
      const mtlLoader = new MTLLoader();
      const materials = mtlLoader.parse(mtlText);
      materials.preload();
      loader.setMaterials(materials);
    }

    const group = loader.parse(text);
    const geometry = _mergeObject3D(group);
    const buffer = new TextEncoder().encode(text).buffer;
    const baseName = file.name.replace(/\.obj$/i, '');
    const color = nextColor();
    const stlId = Date.now() + '_' + Math.random().toString(36).slice(2, 8);
    _addMeshToScene(geometry, buffer, 'obj', baseName, color, stlId, null);
  };
  doLoad();
}

export function loadPLYFile(file) {
  const reader = new FileReader();
  reader.onload = (e) => {
    const buffer = e.target.result;
    const geometry = plyLoader.parse(buffer);
    const baseName = file.name.replace(/\.ply$/i, '');
    const color = nextColor();
    const stlId = Date.now() + '_' + Math.random().toString(36).slice(2, 8);
    if (_isPLYPointCloud(buffer)) {
      _addPointsToScene(geometry, buffer, baseName, color, stlId, null);
    } else {
      geometry.computeVertexNormals();
      _addMeshToScene(geometry, buffer, 'ply', baseName, color, stlId, null);
    }
  };
  reader.readAsArrayBuffer(file);
}

export async function loadGLBFile(file) {
  const buffer = await file.arrayBuffer();
  const baseName = file.name.replace(/\.(glb|gltf)$/i, '');
  const color = nextColor();
  const stlId = Date.now() + '_' + Math.random().toString(36).slice(2, 8);
  try {
    const gltf = await new Promise((resolve, reject) =>
      gltfImportLoader.parse(buffer, '', resolve, reject));
    const geometry = _mergeObject3D(gltf.scene);
    _addMeshToScene(geometry, buffer, 'glb', baseName, color, stlId, null);
  } catch (err) {
    console.error('GLB load error:', err);
    alert(`Failed to load GLB/GLTF file: ${err.message}`);
  }
}

// ============================================================
// Primitive creation
// ============================================================
function geometryToSTLBuffer(geometry) {
  const pos = geometry.getAttribute('position');
  const idx = geometry.getIndex();
  const triCount = idx ? idx.count / 3 : pos.count / 3;
  const bufLen = 84 + triCount * 50;
  const buf = new ArrayBuffer(bufLen);
  const view = new DataView(buf);
  view.setUint32(80, triCount, true);
  let offset = 84;
  const vA = new THREE.Vector3(), vB = new THREE.Vector3(), vC = new THREE.Vector3();
  const cb = new THREE.Vector3(), ab = new THREE.Vector3();
  for (let i = 0; i < triCount; i++) {
    const a = idx ? idx.getX(i * 3)     : i * 3;
    const b = idx ? idx.getX(i * 3 + 1) : i * 3 + 1;
    const c = idx ? idx.getX(i * 3 + 2) : i * 3 + 2;
    vA.fromBufferAttribute(pos, a);
    vB.fromBufferAttribute(pos, b);
    vC.fromBufferAttribute(pos, c);
    cb.subVectors(vC, vB); ab.subVectors(vA, vB); cb.cross(ab).normalize();
    view.setFloat32(offset, cb.x, true); offset += 4;
    view.setFloat32(offset, cb.y, true); offset += 4;
    view.setFloat32(offset, cb.z, true); offset += 4;
    for (const v of [vA, vB, vC]) {
      view.setFloat32(offset, v.x, true); offset += 4;
      view.setFloat32(offset, v.y, true); offset += 4;
      view.setFloat32(offset, v.z, true); offset += 4;
    }
    view.setUint16(offset, 0, true); offset += 2;
  }
  return buf;
}

export function addPrimitive(type) {
  const size = 0.05;
  let geometry;
  let name;
  if (type === 'cube') {
    geometry = new THREE.BoxGeometry(size, size, size);
    name = 'Cube';
  } else if (type === 'sphere') {
    geometry = new THREE.SphereGeometry(size / 2, 24, 16);
    name = 'Sphere';
  } else {
    geometry = new THREE.CylinderGeometry(size / 2, size / 2, size, 24);
    name = 'Cylinder';
  }
  const nonIndexed = geometry.index ? geometry.toNonIndexed() : geometry;
  const buffer = geometryToSTLBuffer(nonIndexed);
  geometry.dispose();
  nonIndexed.dispose();

  const color = nextColor();
  const stlId = Date.now() + '_' + Math.random().toString(36).slice(2, 8);
  createSTLFromBuffer(buffer, name, color, stlId, null);
}

// ============================================================
// Duplicate an imported object
// ============================================================
export async function duplicateSTL(srcEntry) {
  const m = srcEntry.mesh;
  const stlId = Date.now() + '_' + Math.random().toString(36).slice(2, 8);
  const name = srcEntry.name + ' copy';
  const color = srcEntry.color;
  const transforms = {
    position: [m.position.x + 0.02, m.position.y, m.position.z],
    rotation: [m.rotation.x, m.rotation.y, m.rotation.z],
    scale: [m.scale.x, m.scale.y, m.scale.z],
    visible: m.visible,
    parentLink: srcEntry.parentLink || null,
  };

  let newEntry;
  const ft = srcEntry.fileType || 'stl';
  if (ft === 'stl') {
    newEntry = createSTLFromBuffer(srcEntry._buffer, name, color, stlId, transforms);
  } else if (ft === 'ply' && srcEntry.isPointCloud) {
    const geometry = plyLoader.parse(srcEntry._buffer);
    newEntry = _addPointsToScene(geometry, srcEntry._buffer, name, color, stlId, transforms);
  } else if (ft === 'ply') {
    const geometry = plyLoader.parse(srcEntry._buffer);
    geometry.computeVertexNormals();
    newEntry = _addMeshToScene(geometry, srcEntry._buffer, 'ply', name, color, stlId, transforms);
  } else if (ft === 'obj') {
    const text = new TextDecoder().decode(srcEntry._buffer);
    const group = objLoader.parse(text);
    const geometry = _mergeObject3D(group);
    newEntry = _addMeshToScene(geometry, srcEntry._buffer, 'obj', name, color, stlId, transforms);
  } else if (ft === 'glb') {
    try {
      const gltf = await new Promise((resolve, reject) =>
        gltfImportLoader.parse(srcEntry._buffer, '', resolve, reject));
      const geometry = _mergeObject3D(gltf.scene);
      newEntry = _addMeshToScene(geometry, srcEntry._buffer, 'glb', name, color, stlId, transforms);
    } catch (e) {
      console.warn('Failed to duplicate GLB mesh:', name, e);
      return;
    }
  }
  // Copy opacity from source
  if (newEntry) {
    newEntry.opacity = srcEntry.opacity;
    newEntry.mesh.material.opacity = srcEntry.opacity;
  }
}

// ============================================================
// STL list UI
// ============================================================
export function addSTLListItem(entry) {
  const list = document.getElementById('stl-list');
  const item = document.createElement('div');
  item.className = 'stl-item';

  const colorSwatch = document.createElement('input');
  colorSwatch.type = 'color';
  colorSwatch.className = 'stl-color';
  colorSwatch.value = '#' + new THREE.Color(entry.color).getHexString();
  colorSwatch.title = 'Change color';
  colorSwatch.addEventListener('input', () => {
    entry.mesh.material.color.set(colorSwatch.value);
    entry.color = entry.mesh.material.color.getHex();
  });

  const alphaSlider = document.createElement('input');
  alphaSlider.type = 'range';
  alphaSlider.className = 'stl-alpha';
  alphaSlider.min = '0';
  alphaSlider.max = '100';
  alphaSlider.value = Math.round((entry.opacity ?? entry.mesh.material.opacity) * 100);
  alphaSlider.title = 'Opacity';
  alphaSlider.addEventListener('input', () => {
    const val = parseInt(alphaSlider.value, 10) / 100;
    entry.mesh.material.opacity = val;
    entry.opacity = val;
  });

  const nameSpan = document.createElement('span');
  nameSpan.className = 'stl-name';
  nameSpan.textContent = entry.name;
  nameSpan.title = 'Click to select, double-click to rename';
  nameSpan.addEventListener('click', (e) => {
    e.stopPropagation();
    selectSTL(entry, item);
  });
  nameSpan.addEventListener('dblclick', () => {
    const input = document.createElement('input');
    input.type = 'text';
    input.className = 'val-input';
    input.style.width = '100px';
    input.value = entry.name;
    nameSpan.style.display = 'none';
    nameSpan.parentNode.insertBefore(input, nameSpan.nextSibling);
    input.focus();
    input.select();
    const finish = (apply) => {
      if (apply && input.value.trim()) {
        entry.name = input.value.trim();
        nameSpan.textContent = entry.name;
        entry.label.element.textContent = entry.name;
      }
      input.remove();
      nameSpan.style.display = '';
    };
    input.addEventListener('blur', () => finish(true));
    input.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') { e.preventDefault(); finish(true); }
      if (e.key === 'Escape') finish(false);
    });
  });

  const visBtn = document.createElement('button');
  visBtn.className = 'stl-vis';
  visBtn.textContent = '\uD83D\uDC41';
  visBtn.title = 'Toggle visibility';
  visBtn.style.opacity = entry.mesh.visible ? 1 : 0.3;
  visBtn.addEventListener('click', () => {
    entry.mesh.visible = !entry.mesh.visible;
    visBtn.style.opacity = entry.mesh.visible ? 1 : 0.3;
  });

  const rmBtn = document.createElement('button');
  rmBtn.className = 'stl-rm';
  rmBtn.textContent = '\u2715';
  rmBtn.title = 'Remove';
  rmBtn.addEventListener('click', () => {
    if (State.selectedSTL === entry) deselectSTL();
    entry.mesh.removeFromParent();
    entry.mesh.geometry.dispose();
    entry.mesh.material.dispose();
    const si = State.importedSTLs.indexOf(entry);
    if (si >= 0) State.importedSTLs.splice(si, 1);
    item.remove();
  });

  const dupBtn = document.createElement('button');
  dupBtn.className = 'stl-vis';
  dupBtn.textContent = '\u2398';
  dupBtn.title = 'Duplicate';
  dupBtn.addEventListener('click', (e) => {
    e.stopPropagation();
    duplicateSTL(entry);
  });

  item.appendChild(colorSwatch);
  item.appendChild(alphaSlider);
  item.appendChild(nameSpan);
  item.appendChild(dupBtn);
  item.appendChild(visBtn);
  item.appendChild(rmBtn);
  list.appendChild(item);
}

// ============================================================
// STL Selection & Transform
// ============================================================
const stlModePanel = document.getElementById('stl-mode');
const stlSelName   = document.getElementById('stl-sel-name');

// STL parent-link assignment (multi-device aware)
const _reparentMat = new THREE.Matrix4();

export function resolveParentLink(parentValue) {
  // parentValue is either 'devId:linkName' (new format) or just 'linkName' (legacy)
  if (!parentValue) return { dev: null, linkName: null };
  if (parentValue.includes(':')) {
    const [devId, linkName] = parentValue.split(':', 2);
    const dev = State.devices.find(d => d.id === devId);
    return { dev: dev || null, linkName };
  }
  // Legacy: search all devices for this link name
  for (const dev of State.devices) {
    if (dev.linkToJoint[parentValue] !== undefined) {
      return { dev, linkName: parentValue };
    }
  }
  return { dev: null, linkName: parentValue };
}

export function setSTLParent(entry, parentValue, preserveLocal = false) {
  const mesh = entry.mesh;
  const { dev, linkName } = resolveParentLink(parentValue);

  if (!preserveLocal) {
    mesh.updateWorldMatrix(true, false);
    _reparentMat.copy(mesh.matrixWorld);
  }

  mesh.removeFromParent();

  let targetGroup = null;
  if (dev && linkName) {
    if (dev.linkToJoint[linkName] !== undefined) {
      targetGroup = dev.jointRotGroups[dev.linkToJoint[linkName]];
    } else if (dev.parentGroups && dev.parentGroups[linkName]) {
      targetGroup = dev.parentGroups[linkName];
    }
  }

  if (targetGroup) {
    if (!preserveLocal) {
      targetGroup.updateWorldMatrix(true, false);
      const localMat = targetGroup.matrixWorld.clone().invert().multiply(_reparentMat);
      localMat.decompose(mesh.position, mesh.quaternion, mesh.scale);
      mesh.rotation.setFromQuaternion(mesh.quaternion);
    }
    targetGroup.add(mesh);
    entry.parentLink = dev.id + ':' + linkName;
  } else {
    if (!preserveLocal) {
      _reparentMat.decompose(mesh.position, mesh.quaternion, mesh.scale);
      mesh.rotation.setFromQuaternion(mesh.quaternion);
    }
    State.scene.add(mesh);
    entry.parentLink = null;
  }
}

export function selectSTL(entry, listItem) {
  deselectSTL();
  State.setSelectedSTL(entry);
  State.setSelectedListItem(listItem || null);

  State.stlTransformControls.attach(entry.mesh);
  stlModePanel.style.display = 'block';
  stlSelName.textContent = entry.name;
  document.getElementById('stlParentSelect').value = entry.parentLink || '';
  syncSTLNumericInputs(entry);

  if (listItem) listItem.classList.add('selected');
}

export function syncSTLNumericInputs(entry) {
  if (!entry) return;
  const m = entry.mesh;
  const fmt = v => +v.toFixed(2);
  const fmtScale = v => +v.toFixed(6);
  document.getElementById('stlPosX').value = fmt(m.position.x * 1000);
  document.getElementById('stlPosY').value = fmt(m.position.z * 1000);
  document.getElementById('stlPosZ').value = fmt(m.position.y * 1000);
  document.getElementById('stlRotX').value = fmt(m.rotation.x * (180 / Math.PI));
  document.getElementById('stlRotY').value = fmt(m.rotation.z * (180 / Math.PI));
  document.getElementById('stlRotZ').value = fmt(m.rotation.y * (180 / Math.PI));
  document.getElementById('stlScX').value = fmtScale(m.scale.x);
  document.getElementById('stlScY').value = fmtScale(m.scale.z);
  document.getElementById('stlScZ').value = fmtScale(m.scale.y);
}

export function deselectSTL() {
  if (State.selectedSTL) {
    State.stlTransformControls.detach();
    State.stlTransformControls.setSpace('world');
    if (State.selectedListItem) State.selectedListItem.classList.remove('selected');
    State.setSelectedSTL(null);
    State.setSelectedListItem(null);
    stlModePanel.style.display = 'none';
    stlSelName.textContent = '';
    document.getElementById('stlSpaceBtn').textContent = 'World';
    document.getElementById('stlSpaceBtn').classList.remove('active');
  }
}

export function setSTLTransformMode(mode) {
  State.stlTransformControls.setMode(mode);
  document.getElementById('stlModeT').classList.toggle('active', mode === 'translate');
  document.getElementById('stlModeR').classList.toggle('active', mode === 'rotate');
  document.getElementById('stlModeS').classList.toggle('active', mode === 'scale');
}
