import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Line2 } from 'three/addons/lines/Line2.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import { LineGeometry } from 'three/addons/lines/LineGeometry.js';

function release(group) {
  while (group.children.length) {
    const child = group.children[0]; group.remove(child);
    child.geometry?.dispose();
    if (Array.isArray(child.material)) child.material.forEach(m => m.dispose());
    else child.material?.dispose();
  }
}
function route(group, points, color, width) {
  release(group);
  if (points.length < 2) return;
  const geometry = new LineGeometry(); geometry.setPositions(points.flat());
  const material = new LineMaterial({ color, linewidth: width, depthTest: false, transparent: true, opacity: .95 });
  const line = new Line2(geometry, material); line.computeLineDistances(); line.renderOrder = 10;
  group.add(line);
}
function seconds(t) { return t ? t.sec + t.nanosec * 1e-9 : -Infinity; }
function buildWorld(group, world, cutaway = false) {
  const categories = new Map();
  for (const box of world.boxes) {
    const floor = Math.max(0, Math.floor((box.center[2] + .12) / 3.2));
    const slab = box.size[2] < .25 && box.size[0] > 1 && box.size[1] > 1;
    const key = `${floor}-${slab}`;
    if (!categories.has(key)) categories.set(key, {floor, slab, boxes: []});
    categories.get(key).boxes.push(box);
  }
  const matrix = new THREE.Matrix4(), position = new THREE.Vector3(), quaternion = new THREE.Quaternion(), scale = new THREE.Vector3();
  for (const category of categories.values()) {
    const clippingPlanes = cutaway && !category.slab ? [new THREE.Plane(new THREE.Vector3(0, 1, 0), 0)] : [];
    const material = new THREE.MeshStandardMaterial({
      color: category.slab ? 0x638399 : 0x90adbd, transparent: true,
      opacity: category.slab ? .15 : cutaway ? .75 : .35, depthWrite: false,
      roughness: .85, side: THREE.DoubleSide, clippingPlanes,
    });
    const mesh = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), material, category.boxes.length);
    category.boxes.forEach((box, i) => {
      position.fromArray(box.center); quaternion.fromArray(box.quaternion); scale.fromArray(box.size);
      matrix.compose(position, quaternion, scale); mesh.setMatrixAt(i, matrix);
    });
    mesh.userData.floor = category.floor; mesh.instanceMatrix.needsUpdate = true; group.add(mesh);
  }
}

// Views of the actual World and GlobalPath, not representative images.
export function Preview({world, path, mode}) {
  const host = useRef(null), [error, setError] = useState('');
  useEffect(() => {
    if (!host.current || (mode === 'map' ? !world?.boxes : !path?.points?.length)) return;
    let renderer;
    try {renderer = new THREE.WebGLRenderer({antialias: true});}
    catch {setError('WebGL 不可用'); return;}
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setClearColor(0x101c29); renderer.localClippingEnabled = true;
    host.current.prepend(renderer.domElement);
    const scene = new THREE.Scene();
    scene.add(new THREE.AmbientLight(0xe0efff, 2.1));
    const light = new THREE.DirectionalLight(0xffffff, 2); light.position.set(5, -12, 25); scene.add(light);
    const geometry = new THREE.Group(), line = new THREE.Group(); scene.add(geometry, line);
    const bounds = new THREE.Box3();
    if (mode === 'map') {
      buildWorld(geometry, world, true);
      bounds.set(new THREE.Vector3().fromArray(world.bounds.min), new THREE.Vector3().fromArray(world.bounds.max));
    } else {
      const points = path.points.map(p => [p.position.x, p.position.y, p.position.z]);
      points.forEach(p => bounds.expandByPoint(new THREE.Vector3(...p)));
      route(line, points, 0x57a7ff, 3);
      [points[0], points.at(-1)].forEach((p, i) => {
        const dot = new THREE.Mesh(new THREE.SphereGeometry(.16, 12, 8), new THREE.MeshBasicMaterial({color: i ? 0xffbc72 : 0x6ce2bf}));
        dot.position.fromArray(p); geometry.add(dot);
      });
    }
    const center = bounds.getCenter(new THREE.Vector3()), size = bounds.getSize(new THREE.Vector3());
    const camera = new THREE.PerspectiveCamera(38, 1, .03, 250); camera.up.set(0, 0, 1);
    const direction = new THREE.Vector3(1.1, -1.5, 1.15).normalize();
    const draw = () => {
      const width = host.current?.clientWidth || 1, height = host.current?.clientHeight || 1;
      renderer.setSize(width, height, false); camera.aspect = width / height;
      const distance = Math.max(size.length() * .7, Math.max(size.x, size.y) / camera.aspect) / Math.tan(38 * Math.PI / 360) * .58;
      camera.position.copy(center).addScaledVector(direction, Math.max(7, distance)); camera.lookAt(center); camera.updateProjectionMatrix();
      line.children.forEach(item => item.material.resolution?.set(width, height));
      renderer.render(scene, camera);
    };
    const observer = new ResizeObserver(draw); observer.observe(host.current); draw();
    return () => {observer.disconnect(); release(geometry); release(line); renderer.dispose(); renderer.domElement.remove();};
  }, [world, path, mode]);
  return <div className={`preview preview-${mode}`} ref={host}>{error && <div className="empty">{error}</div>}{(mode === 'map' ? !world : !path?.points?.length) && <div className="empty">{mode === 'map' ? '等待场景' : '等待全局规划'}</div>}</div>;
}

export default function Scene({ world, globalPath, localTrajectory, initialTrajectories, odom, sessionKey }) {
  const host = useRef(null), orbitSurface = useRef(null), state = useRef(null), live = useRef({ odom });
  const trajectories = useRef([]);
  const [filter, setFilter] = useState('all'), [error, setError] = useState('');
  live.current = { odom, filter };
  useEffect(() => {
    if (!host.current) return;
    let renderer;
    try { renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false }); }
    catch { setError('浏览器无法创建 WebGL 场景，请启用硬件加速。'); return; }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setClearColor(0x0d1724); renderer.setScissorTest(true);
    host.current.prepend(renderer.domElement);
    const scene = new THREE.Scene(); scene.background = new THREE.Color(0x0d1724);
    scene.add(new THREE.AmbientLight(0xdcecff, 2));
    const light = new THREE.DirectionalLight(0xffffff, 2); light.position.set(12, -8, 25); scene.add(light);
    const overview = new THREE.PerspectiveCamera(43, 1, .05, 200);
    overview.up.set(0, 0, 1); overview.position.set(25, -32, 27);
    const follow = new THREE.PerspectiveCamera(55, 1, .03, 150); follow.up.set(0, 0, 1);
    const controls = new OrbitControls(overview, orbitSurface.current);
    controls.target.set(0, 0, 3); controls.minDistance = 4; controls.maxDistance = 85;
    controls.enableDamping = true;
    const grid = new THREE.GridHelper(32, 32, 0x344559, 0x1c2c3d); grid.rotation.x = Math.PI / 2; grid.position.z = -.12; scene.add(grid);
    const axes = new THREE.AxesHelper(2); axes.position.set(-11, -8, .02); scene.add(axes);
    const geometry = new THREE.Group(), global = new THREE.Group(), local = new THREE.Group(), trail = new THREE.Group();
    scene.add(geometry, global, local, trail);
    const robot = new THREE.Mesh(new THREE.BoxGeometry(.2, .2, .2), new THREE.MeshStandardMaterial({ color: 0x50e3c2, emissive: 0x143e37 }));
    const nose = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), .65, 0x62f5d3, .16, .09); robot.add(nose);
    robot.visible = false; scene.add(robot);
    const history = [];
    state.current = { scene, renderer, geometry, global, local, trail, robot, history, overview, controls };
    let frame;
    const rendererSize = new THREE.Vector2();
    const render = () => {
      const width = host.current?.clientWidth || 1, height = host.current?.clientHeight || 1;
      renderer.getSize(rendererSize);
      if (rendererSize.x !== width || rendererSize.y !== height) renderer.setSize(width, height, false);
      const { odom: current, filter: currentFilter } = live.current;
      let floor = null;
      if (currentFilter === 'current' && current) floor = Math.max(0, Math.floor((current.pose.pose.position.z - .2) / 3.2));
      else if (currentFilter !== 'all' && currentFilter !== 'current') floor = Number(currentFilter);
      geometry.children.forEach(mesh => { mesh.visible = floor === null || mesh.userData.floor === floor; });
      const pose = current?.pose?.pose;
      if (pose) {
        const p = pose.position, q = pose.orientation;
        robot.position.set(p.x, p.y, p.z); robot.quaternion.set(q.x, q.y, q.z, q.w); robot.visible = true;
        const heading = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        follow.position.set(p.x - 3.7 * Math.cos(heading), p.y - 3.7 * Math.sin(heading), p.z + 2.1);
        follow.lookAt(p.x + 1.2 * Math.cos(heading), p.y + 1.2 * Math.sin(heading), p.z + .1);
      } else { robot.visible = false; follow.position.set(0, -14, 10); follow.lookAt(0, 0, 2); }
      controls.update();
      const left = Math.floor(width * .60);
      for (const [x, viewWidth, camera] of [[0, left, overview], [left, width - left, follow]]) {
        camera.aspect = viewWidth / height; camera.updateProjectionMatrix();
        [global, local, trail].forEach(group => group.children.forEach(line => line.material.resolution.set(viewWidth, height)));
        renderer.setViewport(x, 0, viewWidth, height); renderer.setScissor(x, 0, viewWidth, height); renderer.render(scene, camera);
      }
      frame = requestAnimationFrame(render);
    };
    render();
    return () => {
      cancelAnimationFrame(frame); controls.dispose();
      [geometry, global, local, trail].forEach(release);
      robot.geometry.dispose(); robot.material.dispose(); renderer.dispose(); renderer.domElement.remove(); state.current = null;
    };
  }, []);
  useEffect(() => {
    if (!state.current || !world?.boxes) return;
    const group = state.current.geometry; release(group);
    buildWorld(group, world);

  }, [world]);
  useEffect(() => { if (state.current) route(state.current.global, (globalPath?.points || []).map(p => [p.position.x, p.position.y, p.position.z]), 0x57a7ff, 3); }, [globalPath]);
  useEffect(() => {
    if (!state.current) return;
    state.current.history.length = 0; release(state.current.trail);
    trajectories.current = [];
  }, [sessionKey]);
  useEffect(() => {
    if (initialTrajectories) trajectories.current = initialTrajectories.slice(-4);
  }, [initialTrajectories]);
  useEffect(() => {
    if (!localTrajectory) {trajectories.current = []; return;}
    const stamp = seconds(localTrajectory.header?.stamp);
    trajectories.current = [...trajectories.current.filter(t => seconds(t.header?.stamp) !== stamp), localTrajectory]
      .sort((a, b) => seconds(a.header?.stamp) - seconds(b.header?.stamp)).slice(-4);
  }, [localTrajectory]);
  useEffect(() => {
    if (!state.current) return;
    const now = seconds(odom?.header?.stamp);
    const message = trajectories.current.filter(t => seconds(t.header?.stamp) <= now).at(-1);
    if (!message || !odom) {release(state.current.local); return;}
    const elapsed = now - seconds(message.header?.stamp);
    const remaining = message.points.filter(p => seconds(p.time_from_start) > elapsed)
      .map(p => p.transforms?.[0]?.translation).filter(Boolean).map(p => [p.x, p.y, p.z]);
    const p = odom.pose.pose.position;
    route(state.current.local, remaining.length ? [[p.x, p.y, p.z], ...remaining] : [], 0xffa64b, 5);
  }, [odom, localTrajectory]);
  useEffect(() => {
    if (!state.current || !odom) return;
    const p = odom.pose.pose.position, history = state.current.history;
    const last = history.at(-1);
    if (!last || Math.hypot(last[0] - p.x, last[1] - p.y, last[2] - p.z) > .015) {
      history.push([p.x, p.y, p.z]); route(state.current.trail, history, 0x9faeba, 2);
    }
  }, [odom]);
  return <section className="scene-panel panel">
    <div className="panel-heading"><h2>任务现场</h2><div className="scene-tools"><select aria-label="三维显示楼层" value={filter} onChange={e => setFilter(e.target.value)}><option value="all">全部楼层 · 透明</option><option value="current">机器人当前楼层</option><option value="0">一层</option><option value="1">二层</option><option value="2">三层</option></select><button className="text-button" onClick={() => { const s = state.current; if (s) {s.overview.position.set(25, -32, 27); s.controls.target.set(0, 0, 3);} }}>重置视角</button></div></div>
    <div className="scene-viewport" ref={host}><div className="orbit-surface" ref={orbitSurface}/><div className="view-label overview-label">地图总览</div><div className="view-divider"/><div className="view-label follow-label">机器人跟随</div>{error && <div className="scene-error">{error}</div>}</div>
    <div className="scene-legend"><span><i className="line blue"/>全局路径</span><span><i className="line orange"/>剩余局部轨迹</span><span><i className="line gray"/>已走轨迹</span></div>
  </section>;
}
