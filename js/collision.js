// ============================================================
// js/collision.js — collision detection (Web Worker + fallback)
// ============================================================
import * as THREE from 'three';
import * as State from './state.js';
import { resolveParentLink } from './stl.js';

// ============================================================
// Highlight helpers
// ============================================================
const materialOriginalEmissive  = new WeakMap();
const highlightedMaterials      = new Set();
const materialOriginalColor     = new WeakMap();
const highlightedPointMaterials = new Set();

function saveOriginalEmissive(material) {
  if (!materialOriginalEmissive.has(material)) {
    materialOriginalEmissive.set(material, material.emissive.clone());
  }
}

export function clearCollisionHighlights() {
  if (highlightedMaterials.size === 0 && highlightedPointMaterials.size === 0) return;
  for (const material of highlightedMaterials) {
    const original = materialOriginalEmissive.get(material);
    if (original) material.emissive.copy(original);
  }
  highlightedMaterials.clear();
  for (const material of highlightedPointMaterials) {
    const original = materialOriginalColor.get(material);
    if (original) material.color.copy(original);
  }
  highlightedPointMaterials.clear();
  const collisionInfoEl  = document.getElementById('collision-info');
  const collisionTextEl  = document.getElementById('collision-text');
  collisionInfoEl.classList.remove('hit');
  collisionTextEl.textContent = 'none';
  while (collisionTextEl.nextSibling) collisionTextEl.nextSibling.remove();
}

function _highlightObject(obj) {
  const mat = obj.material;
  if (mat.emissive) {
    saveOriginalEmissive(mat);
    mat.emissive.set(0xff2200);
    highlightedMaterials.add(mat);
  } else {
    if (!materialOriginalColor.has(mat)) materialOriginalColor.set(mat, mat.color.clone());
    mat.color.set(0xff2200);
    highlightedPointMaterials.add(mat);
  }
}

function highlightCollisionMeshes(meshA, meshB) {
  _highlightObject(meshA);
  _highlightObject(meshB);
}

// ============================================================
// Web Worker state
// ============================================================
let worker        = null;
let workerReady   = false;
let workerBusy    = false;
const sentMeshes  = new Set();     // mesh UUIDs sent to worker
const meshByUUID  = new Map();     // UUID -> mesh (for highlighting)

export function initCollisionWorker() {
  try {
    worker = new Worker('js/collision-worker.js', { type: 'module' });
    worker.onmessage = onWorkerMessage;
    worker.onerror = (e) => {
      console.warn('[Collision] Worker failed, using main thread:', e.message || e);
      workerReady = false;
      worker = null;
    };
  } catch (e) {
    console.warn('[Collision] Worker not supported, using main thread');
    worker = null;
  }
}

function onWorkerMessage(e) {
  const msg = e.data;
  switch (msg.type) {
    case 'ready':
      workerReady = true;
      console.log('[Collision] Worker ready — offloaded to background thread');
      break;
    case 'results':
      workerBusy = false;
      applyWorkerResults(msg.collisions, _pendingFloorCollisions);
      _pendingFloorCollisions = [];
      break;
    case 'error':
      console.warn('[Collision] Worker init failed, using main thread:', msg.message);
      workerReady = false;
      worker = null;
      break;
  }
}

function ensureMeshInWorker(mesh) {
  if (sentMeshes.has(mesh.uuid)) return;
  const geom = mesh.geometry;
  const pos  = geom.getAttribute('position');
  if (!pos) return;
  const idx  = geom.getIndex();

  // Copy buffers (originals stay on main thread for rendering)
  const positions = new Float32Array(pos.array);
  const index     = idx ? new Uint32Array(idx.array) : null;
  const isPointCloud = !!mesh.isPoints;

  const transfer = [positions.buffer];
  if (index) transfer.push(index.buffer);

  worker.postMessage({
    type: 'addMesh',
    meshId: mesh.uuid,
    geomId: geom.uuid,
    positions,
    index,
    isPointCloud,
  }, transfer);

  sentMeshes.add(mesh.uuid);
  meshByUUID.set(mesh.uuid, mesh);
}

export function removeCollisionMesh(mesh) {
  if (worker && sentMeshes.has(mesh.uuid)) {
    worker.postMessage({ type: 'removeMesh', meshId: mesh.uuid });
    sentMeshes.delete(mesh.uuid);
    meshByUUID.delete(mesh.uuid);
  }
}

// ============================================================
// Shared: build extended links + pair context
// ============================================================
function buildCollisionContext() {
  const visibleSTLs       = State.importedSTLs.filter(e => e.mesh.visible && !e.isPointCloud);
  const visiblePointClouds = State.importedSTLs.filter(e => e.mesh.visible && e.isPointCloud);

  const worldSTLs    = visibleSTLs.filter(e => !e.parentLink);
  const parentedSTLs = visibleSTLs.filter(e => e.parentLink);

  const allExtendedLinks = [];
  for (const dev of State.devices) {
    for (const link of dev.robotLinkMeshes) {
      allExtendedLinks.push({
        name: link.name,
        deviceName: dev.name,
        deviceId: dev.id,
        meshes: [...link.meshes],
        stlEntries: [],
      });
    }
  }

  for (const stl of parentedSTLs) {
    const { dev, linkName } = resolveParentLink(stl.parentLink);
    if (dev && linkName) {
      const extLink = allExtendedLinks.find(l => l.deviceId === dev.id && l.name === linkName);
      if (extLink) {
        extLink.meshes.push(stl.mesh);
        extLink.stlEntries.push(stl);
      }
    }
  }

  return { worldSTLs, parentedSTLs, visiblePointClouds, allExtendedLinks };
}

function meshDisplayName(link, mesh) {
  const stlEntry = link.stlEntries.find(e => e.mesh === mesh);
  if (stlEntry) return stlEntry.name;
  return State.devices.length > 1 ? `${link.deviceName}:${link.name}` : link.name;
}

// ============================================================
// Worker path: build pairs & send to worker
// ============================================================
function checkCollisionsOffThread() {
  if (workerBusy) return;

  const { worldSTLs, parentedSTLs, visiblePointClouds, allExtendedLinks } = buildCollisionContext();

  const matrices  = {};
  const meshPairs = [];
  const pcPairs   = [];
  const hitPairs  = new Set();

  function collectMatrix(mesh) {
    if (!matrices[mesh.uuid]) {
      matrices[mesh.uuid] = Array.from(mesh.matrixWorld.elements);
    }
    ensureMeshInWorker(mesh);
  }

  function addPair(target, meshA, meshB, nameA, nameB) {
    const key = meshA.uuid < meshB.uuid
      ? meshA.uuid + '|' + meshB.uuid
      : meshB.uuid + '|' + meshA.uuid;
    if (hitPairs.has(key)) return;
    hitPairs.add(key);
    collectMatrix(meshA);
    collectMatrix(meshB);
    target.push([meshA.uuid, meshB.uuid, nameA, nameB]);
  }

  // 1) World STLs vs all device links
  for (const stlEntry of worldSTLs) {
    for (const link of allExtendedLinks) {
      for (const robotMesh of link.meshes) {
        const displayName = meshDisplayName(link, robotMesh);
        addPair(meshPairs, stlEntry.mesh, robotMesh, displayName, stlEntry.name);
      }
    }
  }

  // 2) Parented STLs vs other links
  for (const stl of parentedSTLs) {
    const { dev: stlDev, linkName: stlLinkName } = resolveParentLink(stl.parentLink);
    for (const link of allExtendedLinks) {
      if (link.deviceId === (stlDev ? stlDev.id : null) && link.name === stlLinkName) continue;
      for (const robotMesh of link.meshes) {
        const displayName = meshDisplayName(link, robotMesh);
        addPair(meshPairs, stl.mesh, robotMesh, displayName, stl.name);
      }
    }
  }

  // 3) Point clouds vs all device links
  for (const pc of visiblePointClouds) {
    for (const link of allExtendedLinks) {
      for (const robotMesh of link.meshes) {
        const displayName = meshDisplayName(link, robotMesh);
        addPair(pcPairs, pc.mesh, robotMesh, displayName, pc.name);
      }
    }
  }

  // 4) Link vs link (self-collision + cross-device)
  for (let i = 0; i < allExtendedLinks.length; i++) {
    for (let j = i + 1; j < allExtendedLinks.length; j++) {
      const linkA = allExtendedLinks[i];
      const linkB = allExtendedLinks[j];
      if (linkA.deviceId === linkB.deviceId) {
        const dev = State.devices.find(d => d.id === linkA.deviceId);
        if (dev && dev.type === 'hexapod') continue;
        if (dev && dev.adjPairs.has([linkA.name, linkB.name].sort().join('|'))) continue;
      }
      for (const meshA of linkA.meshes) {
        const nameA = meshDisplayName(linkA, meshA);
        for (const meshB of linkB.meshes) {
          const nameB = meshDisplayName(linkB, meshB);
          addPair(meshPairs, meshA, meshB, nameA, nameB);
        }
      }
    }
  }

  // Collect floor collisions synchronously (cheap AABB check, no worker needed)
  _pendingFloorCollisions = collectFloorCollisions(worldSTLs, parentedSTLs, visiblePointClouds, allExtendedLinks);

  if (meshPairs.length === 0 && pcPairs.length === 0) {
    applyWorkerResults([], _pendingFloorCollisions);
    _pendingFloorCollisions = [];
    return;
  }

  workerBusy = true;
  worker.postMessage({
    type: 'checkPairs',
    matrices,
    meshPairs,
    pointCloudPairs: pcPairs,
  });
}

function applyWorkerResults(collisions, floorCollisions = []) {
  clearCollisionHighlights();

  const collisionInfoEl = document.getElementById('collision-info');
  const collisionTextEl = document.getElementById('collision-text');

  const collisionList = [];
  for (const [idA, idB, nameA, nameB] of collisions) {
    collisionList.push({ linkName: nameA, stlName: nameB });
    const meshA = meshByUUID.get(idA);
    const meshB = meshByUUID.get(idB);
    if (meshA && meshB) highlightCollisionMeshes(meshA, meshB);
  }
  for (const fc of floorCollisions) {
    collisionList.push({ linkName: 'floor', stlName: fc.name });
    const mesh = meshByUUID.get(fc.mesh.uuid);
    if (mesh) _highlightObject(mesh);
  }

  // Remove stale pair spans
  const parent = collisionInfoEl;
  while (parent.children.length > 0 && parent.lastChild !== collisionTextEl &&
         parent.lastChild.classList && parent.lastChild.classList.contains('collision-pair')) {
    parent.lastChild.remove();
  }

  State.setLastCollisions(collisionList);

  if (collisionList.length > 0) {
    collisionInfoEl.classList.add('hit');
    collisionTextEl.textContent = `${collisionList.length} collision${collisionList.length > 1 ? 's' : ''}`;
    for (const c of collisionList) {
      const span = document.createElement('span');
      span.className = 'collision-pair';
      span.textContent = `${c.linkName} \u2194 ${c.stlName}`;
      parent.appendChild(span);
    }
  } else {
    collisionInfoEl.classList.remove('hit');
    collisionTextEl.textContent = 'none';
  }
}

// ============================================================
// Main-thread fallback (original BVH logic)
// ============================================================
const _collBox1   = new THREE.Box3();
const _collBox2   = new THREE.Box3();
const _collMatrix = new THREE.Matrix4();
const _collPoint  = new THREE.Vector3();
const _collToLocal = new THREE.Matrix4();
const POINT_CLOUD_COLLISION_THRESHOLD = 0.04;

const _corners = new Array(8).fill(null).map(() => new THREE.Vector3());
function fastWorldAABB(mesh, target) {
  if (!mesh.geometry.boundingBox) mesh.geometry.computeBoundingBox();
  const bb = mesh.geometry.boundingBox;
  const m = mesh.matrixWorld;
  let i = 0;
  for (let x = 0; x <= 1; x++)
    for (let y = 0; y <= 1; y++)
      for (let z = 0; z <= 1; z++)
        _corners[i++].set(
          x ? bb.max.x : bb.min.x,
          y ? bb.max.y : bb.min.y,
          z ? bb.max.z : bb.min.z
        ).applyMatrix4(m);
  target.makeEmpty();
  for (let j = 0; j < 8; j++) target.expandByPoint(_corners[j]);
  return target;
}

function testPointCloudCollision(pointsObj, meshObj) {
  fastWorldAABB(pointsObj, _collBox1);
  fastWorldAABB(meshObj, _collBox2);
  if (!_collBox1.intersectsBox(_collBox2)) return false;

  const bvh = meshObj.geometry.boundsTree;
  if (!bvh) return false;

  if (!pointsObj._collSamples) {
    const pos = pointsObj.geometry.getAttribute('position');
    if (!pos || pos.count === 0) return false;
    const stride = Math.max(1, Math.floor(pos.count / 20000));
    const arr = new Float32Array(Math.ceil(pos.count / stride) * 3);
    let j = 0;
    for (let i = 0; i < pos.count; i += stride) {
      arr[j++] = pos.getX(i);
      arr[j++] = pos.getY(i);
      arr[j++] = pos.getZ(i);
    }
    pointsObj._collSamples = arr;
    pointsObj._collSampleCount = j / 3 | 0;
  }

  const matKey = pointsObj.matrixWorld.elements.join(',');
  if (!pointsObj._collWorld || pointsObj._collWorldKey !== matKey) {
    const src = pointsObj._collSamples;
    const n   = pointsObj._collSampleCount;
    const dst = pointsObj._collWorld = new Float32Array(n * 3);
    const m   = pointsObj.matrixWorld.elements;
    for (let i = 0; i < n; i++) {
      const x = src[i*3], y = src[i*3+1], z = src[i*3+2];
      dst[i*3]   = m[0]*x + m[4]*y + m[8]*z  + m[12];
      dst[i*3+1] = m[1]*x + m[5]*y + m[9]*z  + m[13];
      dst[i*3+2] = m[2]*x + m[6]*y + m[10]*z + m[14];
    }
    pointsObj._collWorldKey = matKey;
  }

  _collToLocal.copy(meshObj.matrixWorld).invert();
  const m   = _collToLocal.elements;
  const src = pointsObj._collWorld;
  const n   = pointsObj._collSampleCount;
  for (let i = 0; i < n; i++) {
    const x = src[i*3], y = src[i*3+1], z = src[i*3+2];
    _collPoint.set(
      m[0]*x + m[4]*y + m[8]*z  + m[12],
      m[1]*x + m[5]*y + m[9]*z  + m[13],
      m[2]*x + m[6]*y + m[10]*z + m[14],
    );
    if (bvh.closestPointToPoint(_collPoint, {}, 0, POINT_CLOUD_COLLISION_THRESHOLD)) return true;
  }
  return false;
}

function testFloorCollision(mesh) {
  fastWorldAABB(mesh, _collBox1);
  return _collBox1.min.y < 0;
}

function testMeshPairCollision(meshA, meshB) {
  fastWorldAABB(meshA, _collBox1);
  fastWorldAABB(meshB, _collBox2);
  if (!_collBox1.intersectsBox(_collBox2)) return false;
  const bvhA = meshA.geometry.boundsTree;
  const bvhB = meshB.geometry.boundsTree;
  if (bvhA && bvhB) {
    _collMatrix.copy(meshA.matrixWorld).invert().multiply(meshB.matrixWorld);
    if (!bvhA.intersectsGeometry(meshB.geometry, _collMatrix)) return false;
  }
  return true;
}

// Pending floor collisions computed synchronously before worker check
let _pendingFloorCollisions = [];

function collectFloorCollisions(worldSTLs, parentedSTLs, visiblePointClouds, allExtendedLinks) {
  const results = [];
  const seen = new Set();
  function add(name, mesh) {
    if (seen.has(name)) return;
    seen.add(name);
    results.push({ name, mesh });
    meshByUUID.set(mesh.uuid, mesh);
  }
  for (const e of [...worldSTLs, ...parentedSTLs]) {
    if (testFloorCollision(e.mesh)) add(e.name, e.mesh);
  }
  for (const pc of visiblePointClouds) {
    if (testFloorCollision(pc.mesh)) add(pc.name, pc.mesh);
  }
  for (const link of allExtendedLinks) {
    for (const robotMesh of link.meshes) {
      const displayName = meshDisplayName(link, robotMesh);
      if (testFloorCollision(robotMesh)) add(displayName, robotMesh);
    }
  }
  return results;
}

function checkCollisionsMainThread() {
  clearCollisionHighlights();

  const collisionInfoEl = document.getElementById('collision-info');
  const collisionTextEl = document.getElementById('collision-text');

  const { worldSTLs, parentedSTLs, visiblePointClouds, allExtendedLinks } = buildCollisionContext();

  const collisions = [];
  const hitPairs = new Set();

  function addCollision(nameA, nameB, meshA, meshB) {
    const key = [nameA, nameB].sort().join('|');
    if (hitPairs.has(key)) return;
    hitPairs.add(key);
    collisions.push({ linkName: nameA, stlName: nameB });
    highlightCollisionMeshes(meshA, meshB);
  }

  // 1) World STLs vs all device links
  for (const stlEntry of worldSTLs) {
    for (const link of allExtendedLinks) {
      for (const robotMesh of link.meshes) {
        const displayName = meshDisplayName(link, robotMesh);
        if (testMeshPairCollision(stlEntry.mesh, robotMesh)) {
          addCollision(displayName, stlEntry.name, stlEntry.mesh, robotMesh);
        }
      }
    }
  }

  // 2) Parented STLs vs other links
  for (const stl of parentedSTLs) {
    const { dev: stlDev, linkName: stlLinkName } = resolveParentLink(stl.parentLink);
    for (const link of allExtendedLinks) {
      if (link.deviceId === (stlDev ? stlDev.id : null) && link.name === stlLinkName) continue;
      for (const robotMesh of link.meshes) {
        const displayName = meshDisplayName(link, robotMesh);
        if (testMeshPairCollision(stl.mesh, robotMesh)) {
          addCollision(displayName, stl.name, stl.mesh, robotMesh);
        }
      }
    }
  }

  // 3) Point clouds vs all device links
  for (const pc of visiblePointClouds) {
    for (const link of allExtendedLinks) {
      for (const robotMesh of link.meshes) {
        const displayName = meshDisplayName(link, robotMesh);
        if (testPointCloudCollision(pc.mesh, robotMesh)) {
          addCollision(displayName, pc.name, pc.mesh, robotMesh);
        }
      }
    }
  }

  // 4) Link vs link (self-collision within same device + cross-device)
  for (let i = 0; i < allExtendedLinks.length; i++) {
    for (let j = i + 1; j < allExtendedLinks.length; j++) {
      const linkA = allExtendedLinks[i];
      const linkB = allExtendedLinks[j];
      if (linkA.deviceId === linkB.deviceId) {
        const dev = State.devices.find(d => d.id === linkA.deviceId);
        if (dev && dev.type === 'hexapod') continue;
        if (dev && dev.adjPairs.has([linkA.name, linkB.name].sort().join('|'))) continue;
      }
      for (const meshA of linkA.meshes) {
        const nameA = meshDisplayName(linkA, meshA);
        for (const meshB of linkB.meshes) {
          const nameB = meshDisplayName(linkB, meshB);
          if (testMeshPairCollision(meshA, meshB)) {
            addCollision(nameA, nameB, meshA, meshB);
          }
        }
      }
    }
  }

  // 5) Floor collisions (imported objects + robot links below y=0)
  for (const fc of collectFloorCollisions(worldSTLs, parentedSTLs, visiblePointClouds, allExtendedLinks)) {
    addCollision('floor', fc.name, fc.mesh, fc.mesh);
  }

  // Update UI
  const parent = collisionInfoEl;
  while (parent.children.length > 0 && parent.lastChild !== collisionTextEl &&
         parent.lastChild.classList && parent.lastChild.classList.contains('collision-pair')) {
    parent.lastChild.remove();
  }

  State.setLastCollisions(collisions);

  if (collisions.length > 0) {
    collisionInfoEl.classList.add('hit');
    collisionTextEl.textContent = `${collisions.length} collision${collisions.length > 1 ? 's' : ''}`;
    for (const c of collisions) {
      const span = document.createElement('span');
      span.className = 'collision-pair';
      span.textContent = `${c.linkName} \u2194 ${c.stlName}`;
      parent.appendChild(span);
    }
  } else {
    collisionInfoEl.classList.remove('hit');
    collisionTextEl.textContent = 'none';
  }
}

// ============================================================
// checkCollisions — called every frame
// ============================================================
let _collisionFrame = 0;
const COLLISION_THROTTLE = 6;

export function checkCollisions() {
  if (!State.collisionEnabled) return;
  if (++_collisionFrame % COLLISION_THROTTLE !== 0) return;

  if (workerReady && worker && !workerBusy) {
    checkCollisionsOffThread();
  } else if (!workerReady || !worker) {
    checkCollisionsMainThread();
  }
  // If workerBusy, skip — last results still displayed
}
