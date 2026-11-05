import * as THREE from 'three';
import { OrbitControls } from '/vendor/OrbitControls.js';
import { STLLoader } from '/vendor/STLLoader.js';

const $ = id => document.getElementById(id);
const viewport = $('viewport');
const scene = new THREE.Scene();
scene.background = new THREE.Color('#e9eef1');
scene.fog = new THREE.Fog('#e9eef1', 5, 16);
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
const camera = new THREE.PerspectiveCamera(38, 1, .01, 50);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.domElement.setAttribute('aria-label', '机器人三维仿真视图');
viewport.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.minDistance = .35;
controls.maxDistance = 10;
controls.maxPolarAngle = Math.PI * .48;
scene.add(new THREE.HemisphereLight(0xffffff, 0x7c919e, 2.6));
const light = new THREE.DirectionalLight(0xffffff, 3.2);
light.position.set(2, -3, 5); light.castShadow = true;
light.shadow.mapSize.set(2048, 2048);
Object.assign(light.shadow.camera, { left: -3, right: 3, top: 3, bottom: -3, near: .1, far: 10 });
light.shadow.bias = -.0001; scene.add(light);
const floor = new THREE.Mesh(new THREE.PlaneGeometry(40, 40), new THREE.MeshStandardMaterial({ color: '#e2e9ed', roughness: 1 }));
floor.receiveShadow = true; floor.position.z = -.002; scene.add(floor);
const grid = new THREE.GridHelper(20, 80, '#a7bac4', '#cbd6dc');
grid.rotation.x = Math.PI / 2; grid.position.z = -.001; scene.add(grid);
const obstacle = new THREE.Mesh(new THREE.BoxGeometry(.2, .6, .3), new THREE.MeshStandardMaterial({ color: '#ca8c61', roughness: .8 }));
obstacle.position.set(1.2, 0, .15); obstacle.castShadow = true; obstacle.receiveShadow = true; scene.add(obstacle);
const robot = new THREE.Group(); scene.add(robot);
const joints = new Map();
const robotPosition = new THREE.Vector3();
const trailPoints = [];
const trailGeometry = new THREE.BufferGeometry();
const trail = new THREE.Line(trailGeometry, new THREE.LineBasicMaterial({ color: '#2eaa88' })); scene.add(trail);
const xyz = text => (text || '0 0 0').trim().split(/\s+/).map(Number);
function origin(object, element) {
  if (!element) return;
  object.position.fromArray(xyz(element.getAttribute('xyz')));
  const [r, p, y] = xyz(element.getAttribute('rpy'));
  object.rotation.set(r, p, y, 'ZYX');
}
function home() {
  controls.target.copy(robotPosition).add(new THREE.Vector3(0, 0, .30));
  camera.position.copy(robotPosition).add(new THREE.Vector3(1.15, -1.3, .95));
  controls.update();
}
$('home').onclick = home; home();
new ResizeObserver(() => {
  camera.aspect = viewport.clientWidth / viewport.clientHeight; camera.updateProjectionMatrix();
  renderer.setSize(viewport.clientWidth, viewport.clientHeight);
}).observe(viewport);

async function loadRobot() {
  const response = await fetch('/robot/urdf/robot.urdf');
  if (!response.ok) throw new Error('机器人描述加载失败');
  const xml = new DOMParser().parseFromString(await response.text(), 'application/xml');
  if (xml.querySelector('parsererror')) throw new Error('机器人描述格式错误');
  const links = new Map();
  for (const link of xml.querySelectorAll('robot > link')) links.set(link.getAttribute('name'), new THREE.Group());
  for (const joint of xml.querySelectorAll('robot > joint')) {
    const object = new THREE.Group(); origin(object, joint.querySelector('origin'));
    object.userData.rest = object.quaternion.clone();
    object.userData.axis = new THREE.Vector3(...xyz(joint.querySelector('axis')?.getAttribute('xyz') || '1 0 0')).normalize();
    links.get(joint.querySelector('parent').getAttribute('link')).add(object);
    object.add(links.get(joint.querySelector('child').getAttribute('link')));
    joints.set(joint.getAttribute('name'), object);
  }
  robot.add(links.get('base_footprint'));
  const loader = new STLLoader();
  const visuals = [...xml.querySelectorAll('robot > link > visual')]; let done = 0;
  await Promise.all(visuals.map(async visual => {
    const filename = visual.querySelector('mesh')?.getAttribute('filename');
    if (!filename) return;
    const geometry = await loader.loadAsync(filename.replace('package://robowheels_description/', '/robot/'));
    const name = visual.parentElement.getAttribute('name');
    const material = new THREE.MeshStandardMaterial({ color: name.includes('wheel') ? '#303b45' : '#93a7b5', metalness: .25, roughness: .6 });
    const mesh = new THREE.Mesh(geometry, material); origin(mesh, visual.querySelector('origin'));
    mesh.castShadow = true; mesh.receiveShadow = true; links.get(name).add(mesh);
    $('progress').textContent = `${++done} / ${visuals.length} 组机械网格`;
  }));
  $('loading').style.display = 'none';
}
loadRobot().catch(error => {
  $('loading').style.display = 'none'; $('error').hidden = false;
  $('error').textContent = `模型加载失败：${error.message}。请刷新重试。`;
});

let stateBusy = false;
async function updateState() {
  if (stateBusy) return; stateBusy = true;
  try {
    const response = await fetch('/api/state', { cache: 'no-store' });
    if (!response.ok) throw new Error('连接中断');
    const state = await response.json();
    $('status').textContent = state.connected ? '仿真运行中' : '等待仿真数据';
    $('status-dot').classList.toggle('live', state.connected);
    $('clock').textContent = `${state.sim_time.toFixed(1)} s`;
    const odom = state.odom;
    if (odom) {
      $('linear').innerHTML = `${odom.linear.toFixed(2)} <em>m/s</em>`;
      $('angular').innerHTML = `${odom.angular.toFixed(2)} <em>rad/s</em>`;
    }
    if (state.pose) {
      const next = new THREE.Vector3(...state.pose.position);
      if ($('follow').checked) { const delta = next.clone().sub(robotPosition); camera.position.add(delta); controls.target.add(delta); }
      robotPosition.copy(next); robot.position.copy(next); robot.quaternion.fromArray(state.pose.quaternion);
      $('position').textContent = `X ${next.x.toFixed(3)}　Y ${next.y.toFixed(3)}`;
      if (!trailPoints.length || next.distanceTo(trailPoints.at(-1)) > .006) {
        trailPoints.push(new THREE.Vector3(next.x, next.y, .002));
        if (trailPoints.length > 3000) trailPoints.shift();
        trailGeometry.setFromPoints(trailPoints);
      }
    }
    for (const [name, angle] of Object.entries(state.joints)) {
      const joint = joints.get(name);
      if (joint) joint.quaternion.copy(joint.userData.rest).multiply(new THREE.Quaternion().setFromAxisAngle(joint.userData.axis, angle));
    }
  } catch {
    $('status').textContent = '连接已断开 · 正在重连'; $('status-dot').classList.remove('live'); stop();
  } finally { stateBusy = false; }
}
setInterval(updateState, 100); updateState();
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });

let direction = null;
let commandBusy = false;
const keys = { w: 'forward', s: 'backward', a: 'left', d: 'right', ArrowUp: 'forward', ArrowDown: 'backward', ArrowLeft: 'left', ArrowRight: 'right' };
async function sendCommand(stopping = false) {
  if (commandBusy && !stopping) return;
  const speed = Number($('speed').value);
  const command = { linear: direction === 'forward' ? speed : direction === 'backward' ? -speed : 0, angular: direction === 'left' ? .6 : direction === 'right' ? -.6 : 0 };
  commandBusy = true;
  try { await fetch('/api/cmd_vel', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(command), keepalive: stopping }); }
  catch {} finally { commandBusy = false; }
}
function drive(next) {
  direction = next;
  document.querySelectorAll('[data-drive]').forEach(button => button.classList.toggle('active', button.dataset.drive === direction));
  sendCommand();
}
function stop() { direction = null; document.querySelectorAll('[data-drive]').forEach(button => button.classList.remove('active')); sendCommand(true); }
for (const button of document.querySelectorAll('[data-drive]')) {
  button.addEventListener('pointerdown', event => { event.preventDefault(); button.setPointerCapture(event.pointerId); drive(button.dataset.drive); });
  button.addEventListener('pointerup', stop); button.addEventListener('pointercancel', stop); button.addEventListener('lostpointercapture', stop);
}
$('stop').onclick = stop;
$('speed').oninput = () => { $('speed-value').textContent = `${Number($('speed').value).toFixed(2)} m/s`; };
window.addEventListener('keydown', event => {
  if (event.code === 'Space') { event.preventDefault(); stop(); return; }
  if (event.target.matches('input') || event.repeat) return;
  const key = event.key.length === 1 ? event.key.toLowerCase() : event.key;
  if (keys[key]) { event.preventDefault(); drive(keys[key]); }
});
window.addEventListener('keyup', event => { const key = event.key.length === 1 ? event.key.toLowerCase() : event.key; if (keys[key] && keys[key] === direction) stop(); });
window.addEventListener('blur', stop); window.addEventListener('pagehide', stop);
document.addEventListener('visibilitychange', () => { if (document.hidden) stop(); });
setInterval(() => { if (direction) sendCommand(); }, 100);
