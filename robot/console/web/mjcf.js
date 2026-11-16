// Minimal MJCF (MuJoCo XML) loader for three.js, enough for XLeRobot's simulation/mujoco/xlerobot.xml (2026-11-10:
// that model is the three-omniwheel build the user has; the URDF in the repo is the older two-wheel cart).
// Supports: nested bodies with pos / quat (w x y z) / euler (radian, intrinsic xyz), mesh geoms with scale and
// rgba, hinge joints (axis, pos), default classes for joints. Collision geoms and translucent helper boxes are
// skipped. Returns a Group (MuJoCo z-up) with .joints[name].setJointValue(rad) like urdf-loader.
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';

const nums = (s, n) => (s ? s.trim().split(/\s+/).map(Number) : n);
const stl = new STLLoader();
const geomCache = {};  // file -> Promise<geometry>, shared so the goal layer reuses the measured layer's meshes

function placement(el, obj) {
  const p = nums(el.getAttribute('pos'));
  if (p) obj.position.set(p[0], p[1], p[2]);
  const q = nums(el.getAttribute('quat'));
  if (q) obj.quaternion.set(q[1], q[2], q[3], q[0]).normalize();
  const e = nums(el.getAttribute('euler'));
  if (e) obj.quaternion.setFromEuler(new THREE.Euler(e[0], e[1], e[2], 'XYZ'));
}

// maxTopZ: keep only the parts of the first (chassis) body lower than this (the base), dropping arms, head and the
// cart stand-in box above it.
export async function loadMJCF(url, { lightJoints = [], maxTopZ = null } = {}) {
  const base = url.slice(0, url.lastIndexOf('/') + 1);
  const doc = new DOMParser().parseFromString(await (await fetch(url)).text(), 'application/xml');
  const meshdir = doc.querySelector('compiler')?.getAttribute('meshdir') || '';
  const meshes = {};
  for (const m of doc.querySelectorAll('asset > mesh')) {
    meshes[m.getAttribute('name')] = { file: base + meshdir.replace(/^\.\//, '') + m.getAttribute('file'), scale: nums(m.getAttribute('scale'), [1, 1, 1]) };
  }
  // joint defaults by class (axis/range), including nested default classes
  const jointDefaults = {};
  for (const d of doc.querySelectorAll('default[class]')) {
    const j = d.querySelector(':scope > joint');
    if (j) jointDefaults[d.getAttribute('class')] = j;
  }

  const geometry = (name) => (geomCache[meshes[name].file] ||= new Promise((res, rej) => stl.load(meshes[name].file, res, undefined, rej)));
  const pending = [];
  const root = new THREE.Group();
  root.joints = {};

  const build = (bodyEl, parent, light, depth = 0) => {
    const body = new THREE.Group();
    body.name = bodyEl.getAttribute('name') || '';
    placement(bodyEl, body);
    parent.add(body);
    // hinge joint: content rotates about axis through the joint position (body frame)
    let content = body;
    const j = bodyEl.querySelector(':scope > joint');
    if (j && (j.getAttribute('type') || 'hinge') === 'hinge') {
      const cls = jointDefaults[j.getAttribute('class')];
      const axis = new THREE.Vector3(...nums(j.getAttribute('axis') || cls?.getAttribute('axis'), [0, 0, 1])).normalize();
      const jp = nums(j.getAttribute('pos'), [0, 0, 0]);
      const pivot = new THREE.Group(); pivot.position.set(...jp); body.add(pivot);
      content = new THREE.Group(); content.position.set(-jp[0], -jp[1], -jp[2]); pivot.add(content);
      const name = j.getAttribute('name');
      if (lightJoints.includes(name)) light = true;
      root.joints[name] = { name, body: content, axis, setJointValue: (v) => pivot.quaternion.setFromAxisAngle(axis, v) };
      content.userData.joint = name;
    }
    for (const g of bodyEl.querySelectorAll(':scope > geom')) {
      const mesh = g.getAttribute('mesh');
      const cls = g.getAttribute('class') || '';
      const rgba = nums(g.getAttribute('rgba'), [0.5, 0.5, 0.5, 1]);
      if (maxTopZ != null && depth === 0 && nums(g.getAttribute("pos"), [0, 0, 0])[2] > maxTopZ) continue;
      if (g.getAttribute('type') === 'box' && rgba[3] < 0.5) {  // the model's stand-in for the cart body
        const s = nums(g.getAttribute('size'));
        const box = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(2 * s[0], 2 * s[1], 2 * s[2])),
          new THREE.LineBasicMaterial({ color: '#3a424b' }));
        placement(g, box); box.userData.placeholder = true; content.add(box);
        continue;
      }
      if (!mesh || cls.includes('collision') || rgba[3] < 0.5) continue;
      const m = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshStandardMaterial());
      placement(g, m);
      m.scale.set(...meshes[mesh].scale);
      m.userData = { file: meshes[mesh].file, light, rgba };
      content.add(m);
      pending.push(geometry(mesh).then((geo) => { m.geometry = geo; }));
    }
    for (const child of bodyEl.querySelectorAll(":scope > body")) {
      if (maxTopZ != null && depth === 0 && nums(child.getAttribute("pos"), [0, 0, 0])[2] > maxTopZ) continue;
      build(child, content, light, depth + 1);
    }
  };
  for (const b of doc.querySelectorAll('worldbody > body')) build(b, root, false);
  await Promise.all(pending);
  return root;
}
