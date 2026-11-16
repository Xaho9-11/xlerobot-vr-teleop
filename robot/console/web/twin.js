// Digital twin: the XLeRobot model in three.js, two layers (solid = measured, translucent = goal), joint
// highlighting both ways (hover a joint row <-> hover the 3D link). console-architecture.md, 2026-11-10.
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/examples/jsm/environments/RoomEnvironment.js';
import URDFLoader from './vendor/urdf-loader/URDFLoader.js';
import { loadMJCF } from './mjcf.js';

const BODY = new THREE.Color('#a9aeb5');
const FRAME = new THREE.Color('#2a2f35');
const css = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();

export async function createTwin(canvas, { model, map, onHover }) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 0.8;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  const bg = new THREE.Color(css('--bg'));
  const scene = new THREE.Scene();
  scene.background = bg;
  scene.fog = new THREE.Fog(bg, 2.2, 5.5);
  const pmrem = new THREE.PMREMGenerator(renderer);
  scene.environment = pmrem.fromScene(new RoomEnvironment(renderer), 0.04).texture;
  scene.environmentIntensity = 0.35;

  const camera = new THREE.PerspectiveCamera(32, 1, 0.02, 30);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true; controls.dampingFactor = 0.08;
  controls.minDistance = 0.5; controls.maxDistance = 4; controls.maxPolarAngle = Math.PI * 0.49;

  scene.add(new THREE.HemisphereLight('#dfe6ee', '#0b0d10', 0.5));
  const key = new THREE.DirectionalLight('#ffffff', 1.6);
  key.position.set(1.4, 2.6, 1.6); key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048); key.shadow.radius = 6; key.shadow.bias = -0.0004;
  Object.assign(key.shadow.camera, { left: -1, right: 1, top: 1, bottom: -1, near: 0.5, far: 6 });
  scene.add(key);
  const rim = new THREE.DirectionalLight('#9fd8d0', 0.5); rim.position.set(-2, 1.2, -2); scene.add(rim);

  // floor: shadow catcher + a faint radial grid
  const floor = new THREE.Mesh(new THREE.CircleGeometry(4, 64), new THREE.ShadowMaterial({ opacity: 0.42 }));
  floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true; scene.add(floor);
  const grid = new THREE.PolarGridHelper(2.4, 12, 10, 96, '#1d2228', '#161a1f');
  grid.position.y = 0.0005; scene.add(grid);

  // The user's build = the URDF's upper body (IKEA RASKOG cart, arms, head; user confirmed 2026-11-10) on the
  // MJCF's three-omniwheel base (the URDF has the older two-wheel cart, the MJCF only a box for the cart and a
  // different arm/head layout). model = { urdf, mjcf, base: { offset, yaw_deg } } (twin_model.json).
  const loadURDF = (url) => new Promise((res, rej) => {  // resolve once every mesh is in (the skeleton comes first)
    const manager = new THREE.LoadingManager();
    let r = null;
    manager.onLoad = () => r && res(r);
    manager.onError = (u) => rej(new Error('failed to load ' + u));
    new URDFLoader(manager).load(url, (x) => { r = x; }, undefined, rej);
  });
  const [robot, ghost, base] = await Promise.all([loadURDF(model.urdf), loadURDF(model.urdf),
    loadMJCF(model.mjcf, { maxTopZ: 0.2 })]);
  const meshFile = (o) => {
    for (let p = o; p; p = p.parent) if (p.isURDFVisual) return p.urdfNode?.querySelector('mesh')?.getAttribute('filename') || '';
    return '';
  };
  const lightJoints = new Set(Object.values(map).map((m) => m.joint).filter(Boolean));
  const underJoint = (o) => { for (let p = o; p; p = p.parent) if (p.isURDFJoint && lightJoints.has(p.name)) return true; return false; };
  // MJCF base: drop its own chassis placement, hang it under the URDF chassis so the cart bottom sits on the plate
  const chassis = base.children[0];
  chassis.position.set(0, 0, 0); chassis.quaternion.identity();
  base.position.set(...model.base.offset); base.rotation.z = (model.base.yaw_deg * Math.PI) / 180;
  robot.links.chassis.add(base);
  // mounting differences from the URDF (user's build 2026-11-10: arm bases and head camera face forward, the URDF
  // has the arm bases turned outward and the head backward): turn those joints about their parent's vertical axis
  // before any joint value is set (urdf-loader keeps the first quaternion as the joint's zero)
  // An entry is degrees, or {deg, pivot}: turn about the vertical through another joint's axis (2026-11-10: turning
  // the arm bases about their own origin moved them off the round mounting plate; the pan axis is its centre).
  const Z = new THREE.Vector3(0, 0, 1);
  for (const r of [robot, ghost]) {
    r.updateMatrixWorld(true);
    const turns = Object.entries(model.joint_yaw_deg || {}).map(([jname, v]) => {
      const j = r.joints[jname], spec = typeof v === 'number' ? { deg: v } : v;
      if (!j) { console.warn('twin: no joint', jname); return null; }
      // pivot: another joint's axis; pivot_local: a point in the turned part's own frame (e.g. its mesh centre)
      const pv = spec.pivot && r.joints[spec.pivot];
      const p = pv ? j.parent.worldToLocal(pv.getWorldPosition(new THREE.Vector3()))
        : spec.pivot_local ? new THREE.Vector3(...spec.pivot_local).applyQuaternion(j.quaternion).add(j.position)
        : j.position.clone();
      return { j, p, q: new THREE.Quaternion().setFromAxisAngle(Z, (spec.deg * Math.PI) / 180) };
    }).filter(Boolean);
    for (const { j, p, q } of turns) {  // pivots measured first, so one turn does not shift another's pivot
      j.position.sub(p).applyQuaternion(q).add(p);
      j.quaternion.premultiply(q);
      if (j.origQuaternion) j.origQuaternion.premultiply(q);
      if (j.origPosition) j.origPosition.copy(j.position);
    }
    // small translations in the parent frame (metres), e.g. centring the arm bases on their round plates
    for (const [jname, d] of Object.entries(model.joint_shift || {})) {
      const j = r.joints[jname];
      if (!j) continue;
      j.position.add(new THREE.Vector3(...d));
      if (j.origPosition) j.origPosition.copy(j.position);
    }
  }

  const accent = new THREE.Color(css('--accent'));
  // the cart and base recede (dark graphite), the arms and head read as the subject
  const solidMat = new THREE.MeshStandardMaterial({ color: BODY, roughness: 0.55, metalness: 0.1 });
  const cartMat = new THREE.MeshStandardMaterial({ color: FRAME, roughness: 0.85, metalness: 0.05 });
  const ghostMat = new THREE.MeshBasicMaterial({ color: accent, transparent: true, opacity: 0.16, depthWrite: false });
  // base parts matching model.base.light_parts (wheels, hubs, drive servos) in model.base.color; the plates stay
  // graphite (user 2026-11-11: an all-white base was too bright)
  const baseCol = new THREE.Color(model.base.color || FRAME);
  const baseMat = new THREE.MeshStandardMaterial({ color: baseCol, roughness: 0.6, metalness: 0.05 });
  const lightPart = new RegExp(model.base.light_parts || '$^', 'i');
  const isBaseLight = (o) => !!o.userData.file && lightPart.test(o.userData.file);
  for (const [r, isGhost] of [[robot, false], [ghost, true]]) {
    r.rotation.x = -Math.PI / 2;  // z-up -> three y-up
    r.traverse((o) => {
      if (!o.isMesh) return;
      if (/raskogwheel/i.test(meshFile(o)) || /wheel/i.test(o.parent?.parent?.name || '') && !o.userData.file) { o.visible = false; return; }
      const light = !o.userData.file && underJoint(o);  // MJCF base meshes carry userData.file
      if (isGhost && !light) { o.visible = false; return; }
      o.material = isGhost ? ghostMat : (light ? solidMat.clone() : isBaseLight(o) ? baseMat : cartMat);
      o.userData.base = light ? BODY : isBaseLight(o) ? baseCol : FRAME;
      o.castShadow = !isGhost; o.receiveShadow = !isGhost;
      if (isGhost) o.renderOrder = 2;
    });
    scene.add(r);
  }

  // frame the upper body (arms + head); the lower cart falls off into the dark
  const box = new THREE.Box3().setFromObject(robot);
  const size = box.getSize(new THREE.Vector3()), c = box.getCenter(new THREE.Vector3());
  // stand on the omniwheels (user 2026-11-11: the real robot rests on its wheels; the box of the whole robot also
  // counts the hidden cart casters)
  const groundY = new THREE.Box3().setFromObject(base).min.y;
  robot.position.y -= groundY; ghost.position.y -= groundY;
  const top = size.y, focus = top * 0.8;
  controls.target.set(c.x, focus, c.z);
  const dist = top * 1.65;
  camera.position.set(c.x + dist * 0.8, focus + top * 0.22, c.z + dist * 0.6);  // front-right 3/4 (front = +x)
  scene.fog.near = dist * 0.9; scene.fog.far = dist * 2.4;

  // the meshes a joint moves directly (its child link, not the links further out), for highlighting one joint
  const ownMeshes = (link) => {
    const out = [];
    const walk = (o) => { for (const ch of o.children) { if (ch.isURDFJoint && lightJoints.has(ch.name)) continue; if (ch.isMesh) out.push(ch); walk(ch); } };
    walk(link); return out;
  };
  const jointMeshes = {};
  for (const [name, m] of Object.entries(map)) {
    const j = robot.joints[m.joint];
    const link = j && j.children.find((ch) => ch.isURDFLink);
    if (link) jointMeshes[name] = ownMeshes(link);
  }
  let lit = null;
  const highlight = (name) => {
    if (lit === name) return;
    for (const mesh of jointMeshes[lit] || []) { mesh.material.color.copy(mesh.userData.base); mesh.material.emissive.setRGB(0, 0, 0); }
    lit = name;
    for (const mesh of jointMeshes[lit] || []) { mesh.material.color.copy(accent); mesh.material.emissive.copy(accent).multiplyScalar(0.18); }
  };

  // hover in 3D -> joint name
  const ray = new THREE.Raycaster(), ptr = new THREE.Vector2();
  const meshToJoint = new Map();
  for (const [name, meshes] of Object.entries(jointMeshes)) for (const m of meshes) meshToJoint.set(m, name);
  canvas.addEventListener('pointermove', (e) => {
    const r = canvas.getBoundingClientRect();
    ptr.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    ray.setFromCamera(ptr, camera);
    const hit = ray.intersectObject(robot, true).find((h) => meshToJoint.has(h.object));
    const name = hit ? meshToJoint.get(hit.object) : null;
    highlight(name); onHover && onHover(name);
  });

  const toRad = (name, deg) => {
    const m = map[name];
    if (!m) return null;
    if (m.gripper) return (m.closed_rad ?? 0) + ((m.open_rad - (m.closed_rad ?? 0)) * deg) / 100;  // percent, 0 = closed
    return ((m.sign * deg + m.offset_deg) * Math.PI) / 180;
  };
  const set = (r, values) => {
    for (const [name, deg] of Object.entries(values)) {
      const v = toRad(name, deg);
      if (v != null && r.joints[map[name].joint]) r.joints[map[name].joint].setJointValue(v);
    }
  };

  const resize = () => {
    const w = canvas.clientWidth, h = canvas.clientHeight;
    renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix();
  };
  addEventListener('resize', resize); resize();
  renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });
  window.__twin = { THREE, scene, camera, controls, robot, ghost, map };  // debugging from the browser console

  // target zone above the tray (core zone.py, 2026-11-11): a translucent cylinder in the arm's base frame (the
  // frame the core tests in), a ring at the tray surface and a dot at the tool point; brighter while inside
  let zoneKey = '', zoneGrp = null;
  const zoneMat = new THREE.MeshBasicMaterial({ color: accent, transparent: true, opacity: 0.08, depthWrite: false, side: THREE.DoubleSide });
  const ringMat = new THREE.LineBasicMaterial({ color: accent, transparent: true, opacity: 0.6 });
  const tcpDot = new THREE.Mesh(new THREE.SphereGeometry(0.006, 16, 12), new THREE.MeshBasicMaterial({ color: accent }));
  tcpDot.visible = false;
  function setZone(z) {
    const link = z && robot.links[z.root];
    const key = z?.set && link ? [z.root, ...z.center, z.radius, z.below, z.above].join('|') : '';
    if (key !== zoneKey) {
      zoneKey = key;
      if (zoneGrp) { zoneGrp.parent?.remove(zoneGrp); zoneGrp.traverse((o) => o.geometry?.dispose()); zoneGrp = null; }
      if (key) {
        const h = z.below + z.above;
        zoneGrp = new THREE.Group();
        zoneGrp.position.set(...z.center);
        const cyl = new THREE.Mesh(new THREE.CylinderGeometry(z.radius, z.radius, h, 64, 1, true), zoneMat);
        cyl.rotation.x = Math.PI / 2; cyl.position.z = (z.above - z.below) / 2; cyl.renderOrder = 3;
        const pts = Array.from({ length: 65 }, (_, i) => new THREE.Vector3(Math.cos(i / 32 * Math.PI) * z.radius, Math.sin(i / 32 * Math.PI) * z.radius, 0));
        zoneGrp.add(cyl, new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), ringMat));
        link.add(zoneGrp);
      }
    }
    if (z?.tcp && link) {
      if (tcpDot.parent !== link) link.add(tcpDot);
      tcpDot.position.set(...z.tcp); tcpDot.visible = !!z.set;
    } else tcpDot.visible = false;
    zoneMat.opacity = z?.inside ? 0.22 : 0.07;
    ringMat.opacity = z?.inside ? 1 : 0.5;
  }

  return {
    setZone,
    setMeasured: (v) => set(robot, v),
    setGoal: (v) => set(ghost, v),
    showGoal: (on) => { ghost.visible = on; },
    highlight,
  };
}
