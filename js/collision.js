// ============================================================
// js/collision.js — collision detection (Web Worker + fallback)
// ============================================================
import * as THREE from 'three';
import * as State from './state.js';
import { resolveParentLink } from './stl.js';

// ============================================================
// Highlight helpers
// ============================================================
const highlightedMeshes = new Set();
const meshOriginalMaterial = new WeakMap();

// Info-panel elements, resolved on first use (this module is imported
// before the rest of the document is guaranteed to be parsed).
let _infoEl = null, _textEl = null, _pairsEl = null, _rateEl = null;
function infoPanel() {
  if (!_infoEl) {
    _infoEl  = document.getElementById('collision-info');
    _textEl  = document.getElementById('collision-text');
    _pairsEl = document.getElementById('collision-pairs');
    _rateEl  = document.getElementById('collision-rate');
  }
  return _infoEl;
}

function restoreMaterials() {
  if (highlightedMeshes.size === 0) return;
  for (const mesh of highlightedMeshes) {
    const orig = meshOriginalMaterial.get(mesh);
    if (orig) mesh.material = orig;
  }
  highlightedMeshes.clear();
}

export function clearCollisionHighlights() {
  restoreMaterials();
  // Force a full refresh of highlights/panel next time a result arrives.
  _lastCollisionSig = null;
  _lastSceneHash    = NaN;
  infoPanel();
  _infoEl.classList.remove('hit');
  _textEl.textContent  = 'none';
  _pairsEl.textContent = '';
  resetRateMeter();
}

function _highlightObject(obj) {
  if (highlightedMeshes.has(obj)) return;
  const orig = obj.material;
  meshOriginalMaterial.set(obj, orig);
  const clone = orig.clone();
  if (clone.emissive) clone.emissive.set(0xff2200);
  else clone.color.set(0xff2200);
  obj.material = clone;
  highlightedMeshes.add(obj);
}

// ============================================================
// Result publication
// ------------------------------------------------------------
// Materials and DOM are only touched when the set of colliding pairs
// actually changes. In headless mode a pass can run hundreds of times a
// second, so cloning materials / rebuilding spans every pass would
// dominate the cost — and requesting a render unconditionally would spin
// the render loop forever whenever a standing collision exists.
// ============================================================
let _lastCollisionSig = null;

function publishCollisions(list) {
  State.setLastCollisions(list);

  let sig = '';
  for (const c of list) sig += c.linkName + '↔' + c.stlName + ';';
  if (sig === _lastCollisionSig) return;
  _lastCollisionSig = sig;

  restoreMaterials();
  for (const c of list) {
    if (c.meshA) _highlightObject(c.meshA);
    if (c.meshB) _highlightObject(c.meshB);
  }

  infoPanel();
  _pairsEl.textContent = '';
  if (list.length > 0) {
    _infoEl.classList.add('hit');
    _textEl.textContent = `${list.length} collision${list.length > 1 ? 's' : ''}`;
    for (const c of list) {
      const span = document.createElement('span');
      span.className = 'collision-pair';
      span.textContent = `${c.linkName} ↔ ${c.stlName}`;
      _pairsEl.appendChild(span);
    }
  } else {
    _infoEl.classList.remove('hit');
    _textEl.textContent = 'none';
  }

  State.requestRender();
}

// ---- check-rate meter --------------------------------------
// Headless only: the loop ticks it at a known cadence, so the reading
// stays honest (a static scene ticks with no work and reads "idle").
// The frame-driven path can stop being called entirely when the render
// loop goes idle, which would leave a stale number on screen.
let _rateWork = 0, _rateT0 = 0, _rateShown = -1;

function resetRateMeter() {
  _rateWork = 0; _rateT0 = 0; _rateShown = -1;
  infoPanel();
  _rateEl.textContent = '';
}

function tickRate(didWork) {
  if (!headlessRunning) return;
  const now = performance.now();
  if (didWork) _rateWork++;
  if (_rateT0 === 0) { _rateT0 = now; return; }
  const dt = now - _rateT0;
  if (dt < 500) return;
  const hz = Math.round(_rateWork * 1000 / dt);
  _rateWork = 0; _rateT0 = now;
  if (hz === _rateShown) return;
  _rateShown = hz;
  infoPanel();
  _rateEl.textContent = hz > 0 ? ` · ${hz} Hz` : ' · idle';
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
      // Headless mode is paced by the worker round-trip, not by rAF:
      // start the next pass as soon as this one lands.
      if (headlessRunning) headlessStep();
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
  const visibleSTLs       = State.importedSTLs.filter(e => e.mesh.visible && !e.isPointCloud && !e.isSplat);
  const visiblePointClouds = State.importedSTLs.filter(e => e.mesh.visible && e.isPointCloud);
  const visibleSplatClouds = State.importedSTLs.filter(e => e.mesh.visible && e.isSplat && e._collisionPoints);
  for (const s of visibleSplatClouds) {
    visiblePointClouds.push({ mesh: s._collisionPoints, name: s.name, parentLink: s.parentLink, isPointCloud: true });
  }

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
// Returns true when a request was posted to the worker (so its 'results'
// message will drive the next headless step), false when it completed
// synchronously because there was nothing to test.
function checkCollisionsOffThread(ctx) {
  const { worldSTLs, parentedSTLs, visiblePointClouds, allExtendedLinks } = ctx;

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
  _pendingFloorCollisions = State.floorCollisionEnabled
    ? collectFloorCollisions(worldSTLs, parentedSTLs, visiblePointClouds, allExtendedLinks)
    : [];

  if (meshPairs.length === 0 && pcPairs.length === 0) {
    applyWorkerResults([], _pendingFloorCollisions);
    _pendingFloorCollisions = [];
    return false;
  }

  workerBusy = true;
  worker.postMessage({
    type: 'checkPairs',
    matrices,
    meshPairs,
    pointCloudPairs: pcPairs,
  });
  return true;
}

function applyWorkerResults(collisions, floorCollisions = []) {
  const collisionList = [];
  for (const [idA, idB, nameA, nameB] of collisions) {
    collisionList.push({
      linkName: nameA, stlName: nameB,
      meshA: meshByUUID.get(idA), meshB: meshByUUID.get(idB),
    });
  }
  for (const fc of floorCollisions) {
    collisionList.push({
      linkName: 'floor', stlName: fc.name,
      meshA: meshByUUID.get(fc.mesh.uuid), meshB: null,
    });
  }
  publishCollisions(collisionList);
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

function checkCollisionsMainThread(ctx) {
  const { worldSTLs, parentedSTLs, visiblePointClouds, allExtendedLinks } = ctx;

  const collisions = [];
  const hitPairs = new Set();

  function addCollision(nameA, nameB, meshA, meshB) {
    const key = [nameA, nameB].sort().join('|');
    if (hitPairs.has(key)) return;
    hitPairs.add(key);
    collisions.push({ linkName: nameA, stlName: nameB, meshA, meshB });
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
  if (State.floorCollisionEnabled) {
    for (const fc of collectFloorCollisions(worldSTLs, parentedSTLs, visiblePointClouds, allExtendedLinks)) {
      addCollision('floor', fc.name, fc.mesh, null);
    }
  }

  publishCollisions(collisions);
}

// ============================================================
// Scene fingerprint
// ------------------------------------------------------------
// Cheap hash of everything a collision result depends on: which meshes
// take part and where they are. An unchanged fingerprint means the
// previous result is still valid and the whole pass can be skipped —
// without this, headless mode would burn a core re-testing a static
// scene as fast as the worker can turn the pairs around.
// ============================================================
const _hashF64 = new Float64Array(1);
const _hashI32 = new Int32Array(_hashF64.buffer);

function _hashNum(h, v) {
  _hashF64[0] = v;
  h = Math.imul(h ^ _hashI32[0], 16777619);
  return Math.imul(h ^ _hashI32[1], 16777619);
}

function _hashMesh(h, mesh) {
  h = Math.imul(h ^ mesh.id, 16777619);
  const e = mesh.matrixWorld.elements;
  for (let i = 0; i < 16; i++) h = _hashNum(h, e[i]);
  return h;
}

function contextFingerprint(ctx) {
  let h = Math.imul(2166136261 ^ (State.floorCollisionEnabled ? 1 : 2), 16777619);
  for (const e of ctx.worldSTLs)           h = _hashMesh(h, e.mesh);
  for (const e of ctx.parentedSTLs)        h = _hashMesh(h, e.mesh);
  for (const pc of ctx.visiblePointClouds) h = _hashMesh(h, pc.mesh);
  for (const link of ctx.allExtendedLinks) {
    for (const m of link.meshes) h = _hashMesh(h, m);
  }
  return h;
}

let _lastSceneHash = NaN;   // NaN never compares equal -> first pass always runs

// ============================================================
// One collision pass
// ------------------------------------------------------------
// Returns PASS_SKIPPED (nothing moved, or the worker is still busy),
// PASS_DISPATCHED (posted to the worker — its reply carries the result)
// or PASS_DONE (completed synchronously).
// ============================================================
const PASS_SKIPPED    = 0;
const PASS_DISPATCHED = 1;
const PASS_DONE       = 2;

function runCollisionPass() {
  const useWorker = workerReady && worker;
  if (!useWorker && worker) return PASS_SKIPPED;   // worker still initialising
  if (useWorker && workerBusy) return PASS_SKIPPED;

  const ctx  = buildCollisionContext();
  const hash = contextFingerprint(ctx);
  if (hash === _lastSceneHash) return PASS_SKIPPED;
  _lastSceneHash = hash;

  if (useWorker) {
    return checkCollisionsOffThread(ctx) ? PASS_DISPATCHED : PASS_DONE;
  }
  checkCollisionsMainThread(ctx);
  return PASS_DONE;
}

// ============================================================
// Headless mode — collision checks decoupled from the render loop
// ------------------------------------------------------------
// Normally checkCollisions() runs from the animation loop, so the check
// rate is capped by the display refresh — and by the on-demand render
// gate, which draws no frames at all while the camera is idle. In
// headless mode the pass is paced by the worker round-trip instead: the
// next pass starts the moment the previous result lands, so throughput
// is bound by the collision computation, not by vsync. It also keeps
// running while the tab is in the background, where rAF is suspended.
// ============================================================
let headlessPref    = false;   // user's toggle
let headlessRunning = false;   // loop actually active
let _headlessTimer  = null;

// Idle poll: how often to re-check a scene that has not moved.
const HEADLESS_IDLE_MS = 8;
// Safety net in case a worker reply never arrives (e.g. the worker died).
const HEADLESS_WATCHDOG_MS = 250;

export function isCollisionHeadless() { return headlessPref; }

export function setCollisionHeadless(on) {
  headlessPref = !!on;
  updateCollisionLoop();
  return headlessPref;
}

// Called by the collision on/off toggle so the loop follows it.
export function updateCollisionLoop() {
  const shouldRun = headlessPref && State.collisionEnabled;
  if (shouldRun === headlessRunning) return;
  headlessRunning = shouldRun;
  if (shouldRun) {
    headlessStep();
  } else {
    clearTimeout(_headlessTimer);
    _headlessTimer = null;
    resetRateMeter();
  }
}

function headlessStep() {
  clearTimeout(_headlessTimer);
  _headlessTimer = null;
  if (!headlessRunning) return;

  // The render loop is not driving us here, so world matrices have to be
  // brought up to date before anything is measured.
  State.scene.updateMatrixWorld();

  const result = runCollisionPass();
  tickRate(result !== PASS_SKIPPED);

  // A dispatched pass is normally continued by the worker's 'results'
  // message; the timer is only a watchdog in that case.
  _headlessTimer = setTimeout(headlessStep,
    result === PASS_DISPATCHED ? HEADLESS_WATCHDOG_MS : HEADLESS_IDLE_MS);
}

// ============================================================
// checkCollisions — called from the animation loop
// ============================================================
let _collisionFrame = 0;
const COLLISION_THROTTLE = 6;

export function checkCollisions() {
  if (!State.collisionEnabled) return;
  if (headlessRunning) return;   // the headless loop owns the checks

  // A frame throttle must never be the last word here. Rendering is
  // on-demand, so a change that draws only a few frames — importing a
  // point cloud, deleting one — can have every one of those frames land
  // on a skipped count. The loop then goes idle and the panel keeps a
  // result that no longer describes the scene, naming objects that have
  // since been removed.
  //
  // The worker path needs no throttle at all: runCollisionPass already
  // refuses to dispatch while a pass is in flight (workerBusy), and
  // early-outs on an unchanged scene fingerprint before doing any real
  // work, so the per-frame cost on a settled scene is just the hash.
  if (workerReady && worker) {
    runCollisionPass();
    return;
  }

  // Main-thread fallback: the pass is synchronous, so the throttle earns
  // its keep. Hold the render loop awake across the skipped frames so a
  // pass still lands once the scene settles.
  if (++_collisionFrame % COLLISION_THROTTLE !== 0) {
    State.requestRender();
    return;
  }

  runCollisionPass();
}
