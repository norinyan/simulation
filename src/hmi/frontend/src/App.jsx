import React, { useEffect, useRef, useState } from 'react';
import Scene, { Preview } from './Scene.jsx';

const empty = {
  simulation: { state: 'stopped', paused: false }, selection: {},
  global_status: { state: 'idle' }, local_status: { state: 'idle' },
  global_path: null, local_trajectory: null, local_trajectories: [], odom: null, received_at: {},
  resources: null, global_timing: null, local_timing: null, dynamic_obstacles: {},
};
const names = { idle: '空闲', starting: '启动中', running: '运行中', stopped: '未启动', stopping: '停止中', planning: '规划中', ready: '已就绪', executing: '执行中', completed: '已完成', failed: '失败', waiting: '等待障碍', avoiding: '避让中', stale: '观测过期' };
const keys = ['start', 'goal', 'global_algorithm', 'local_algorithm', 'dynamic_obstacles'];
const sameSelection = (a, b) => keys.every(key => a?.[key] === b?.[key]);
const number = (v, digits = 2) => Number.isFinite(v) ? v.toFixed(digits) : '—';
const stamp = value => value ? value.sec + value.nanosec * 1e-9 : null;
const xyz = p => p ? `${number(p.x)}, ${number(p.y)}, ${number(p.z)}` : '—';
const yaw = q => q ? Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) : null;
const timeLabel = seconds => seconds ? new Date(seconds * 1000).toLocaleTimeString('zh-CN', { hour12: false }) : '—';
async function get(url) {
  const response = await fetch(url);
  if (!response.ok) { const data = await response.json(); throw new Error(typeof data.detail === 'string' ? data.detail : JSON.stringify(data.detail)); }
  return response.json();
}
function StateBadge({ value }) { return <span className={`state state-${value || 'idle'}`}>{names[value] || value || '未知'}</span>; }
function Raw({ message }) { return <details className="raw-detail"><summary>完整话题数据</summary><pre className="raw-message">{JSON.stringify(message, null, 2)}</pre></details>; }
function pathLength(message) {
  const p = message?.points || [];
  return p.length > 1 ? p.slice(1).reduce((sum, item, i) => sum + Math.hypot(item.position.x - p[i].position.x, item.position.y - p[i].position.y, item.position.z - p[i].position.z), 0) : null;
}
function summarize(message) {
  const points = message.points || [];
  const speeds = points.map(p => p.velocities?.[0]?.linear).filter(Boolean).map(v => Math.hypot(v.x, v.y, v.z));
  return {stamp: stamp(message.header?.stamp), count: points.length,
    duration: stamp(points.at(-1)?.time_from_start), first: points[0]?.transforms?.[0], last: points.at(-1)?.transforms?.[0],
    minSpeed: speeds.length ? Math.min(...speeds) : null, maxSpeed: speeds.length ? Math.max(...speeds) : null};
}
function LocalRecord({ record, index, message }) {
  return <details className="local-record" open={index === 0 ? true : undefined}>
    <summary><span className="record-dot"/><strong>t {number(record.stamp, 3)} s</strong><span>{record.count} 点 · {number(record.duration)} s</span></summary>
    <div className="record-body"><div><span>起点</span><code>{xyz(record.first?.translation)} · yaw {number(yaw(record.first?.rotation))}</code></div><div><span>终点</span><code>{xyz(record.last?.translation)} · yaw {number(yaw(record.last?.rotation))}</code></div><div><span>速度</span><code>{number(record.minSpeed)}–{number(record.maxSpeed)} m/s</code></div>{message && <Raw message={message}/>}</div>
  </details>;
}
function Sparkline({ history, node }) {
  const values = history.filter(r => Number.isFinite(r[node]?.cpu_percent));
  if (values.length < 2) return <span className="muted">等待采样</span>;
  const first = values[0].stamp, duration = Math.max(1, values.at(-1).stamp - first);
  const max = Math.max(100, ...values.map(v => v[node].cpu_percent));
  const points = values.map(v => `${(v.stamp - first) / duration * 150},${30 - v[node].cpu_percent / max * 27}`).join(' ');
  return <svg className={`sparkline ${node}`} viewBox="0 0 150 34" aria-label={`${node} CPU实际采样曲线`}><path d="M0 31 H150" stroke="currentColor" opacity=".15"/><polyline points={points} fill="none" stroke="currentColor" strokeWidth="2" vectorEffect="non-scaling-stroke"/></svg>;
}
function Resources({ live, history }) {
  return <section className="panel resources-panel"><div className="panel-heading"><h2>算法计算资源</h2><span className="heading-meta">CPU 单核 100% · 内存 MiB</span></div>
    <div className="resource-scroll"><table className="resource-table"><thead><tr><th>规划节点</th><th>CPU / 峰值</th><th>内存 / 峰值</th><th>本次规划</th><th>CPU 记录</th></tr></thead><tbody>{['global', 'local'].map(node => { const r = live.resources?.[node], timing = live[`${node}_timing`]; return <tr key={node}><th><i className={`node-dot ${node}`}/>{node === 'global' ? '全局' : '局部'}</th><td>{r ? `${number(r.cpu_percent, 1)} / ${number(r.cpu_peak_percent, 1)} %` : '—'}</td><td>{r ? `${number(r.rss_mib, 1)} / ${number(r.rss_peak_mib, 1)}` : '—'}</td><td>{Number.isFinite(timing?.elapsed_ms) ? `${number(timing.elapsed_ms)} ms` : '—'}{timing?.success === false && <span className="failure-text"> 失败</span>}</td><td><Sparkline history={history} node={node}/></td></tr>; })}</tbody></table></div>
    <details className="resource-history"><summary>采样记录 · {timeLabel(live.resources?.stamp)}</summary><div className="history-scroll"><table><thead><tr><th>采样时间</th><th>全局 CPU %</th><th>全局 MiB</th><th>局部 CPU %</th><th>局部 MiB</th></tr></thead><tbody>{history.slice().reverse().map((r, i) => <tr key={`${r.stamp}-${i}`}><td>{timeLabel(r.stamp)}</td><td>{number(r.global?.cpu_percent, 1)}</td><td>{number(r.global?.rss_mib, 1)}</td><td>{number(r.local?.cpu_percent, 1)}</td><td>{number(r.local?.rss_mib, 1)}</td></tr>)}</tbody></table></div></details>
  </section>;
}

export default function App() {
  const [config, setConfig] = useState(null), [world, setWorld] = useState(null);
  const [live, setLive] = useState(empty), [connected, setConnected] = useState(false);
  const [selection, setSelection] = useState({ start: '', goal: '', global_algorithm: 'pct', local_algorithm: 'scan', dynamic_obstacles: true });
  const [busy, setBusy] = useState(''), [error, setError] = useState(''), [clock, setClock] = useState(Date.now());
  const [records, setRecords] = useState([]), [resourceHistory, setResourceHistory] = useState([]);
  const [pausedDisplay, setPausedDisplay] = useState(false), [frozen, setFrozen] = useState(null);
  const initialized = useRef(false), selectionEdited = useRef(false);
  useEffect(() => {
    let cancelled = false;
    get('/api/config').then(data => { if (!cancelled) { setConfig(data); if (!initialized.current && data.selection) {setSelection(data.selection); initialized.current = true;} } }).catch(e => !cancelled && setError(`配置读取失败：${e.message}`));
    get('/api/map/world').then(data => !cancelled && setWorld(data)).catch(e => !cancelled && setError(`场景读取失败：${e.message}`));
    return () => {cancelled = true;};
  }, []);
  useEffect(() => {
    let stopped = false, socket, timer;
    function addRecord(message) {
      if (!message) {setRecords([]); return;}
      const record = summarize(message);
      setRecords(previous => [record, ...previous.filter(r => r.stamp !== record.stamp)].sort((a, b) => b.stamp - a.stamp).slice(0, 50));
    }
    function connect() {
      socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
      socket.onopen = () => setConnected(true);
      socket.onmessage = event => {
        try {
          const { type, data, received_at: received } = JSON.parse(event.data);
          if (type === 'snapshot') {
            setLive({ ...empty, ...data, received_at: data.received_at || {} });
            if (data.local_trajectories?.length) data.local_trajectories.forEach(addRecord);
            else addRecord(data.local_trajectory);
            if (data.resources) setResourceHistory(previous => [...previous.filter(r => r.stamp !== data.resources.stamp), data.resources].slice(-60));
            if (!selectionEdited.current && data.selection?.start) {setSelection(data.selection); initialized.current = true;}
          } else if (Object.hasOwn(empty, type)) {
            setLive(previous => ({ ...previous, [type]: data, received_at: { ...previous.received_at, [type]: received || Date.now() / 1000 } }));
            if (type === 'local_trajectory') addRecord(data);
            if (type === 'resources' && data) setResourceHistory(previous => [...previous, data].slice(-60));
            if (type === 'selection') {setRecords([]); setResourceHistory([]);}
            if (type === 'simulation' && data.state === 'stopped') {setRecords([]); setResourceHistory([]);}
          }
        } catch {setError('收到无法解析的桥接数据');}
      };
      socket.onerror = () => socket.close();
      socket.onclose = () => {setConnected(false); if (!stopped) timer = setTimeout(connect, 1500);};
    }
    connect();
    return () => {stopped = true; clearTimeout(timer); socket?.close();};
  }, []);
  useEffect(() => {const timer = setInterval(() => setClock(Date.now()), 1000); return () => clearInterval(timer);}, []);
  const running = live.simulation?.state === 'running', paused = !!live.simulation?.paused;
  const executing = ['planning', 'executing', 'waiting', 'avoiding', 'stale'].includes(live.local_status?.state), planning = live.global_status?.state === 'planning';
  const locked = !!busy || executing || planning || ['starting', 'stopping'].includes(live.simulation?.state);
  const dirty = !sameSelection(selection, live.selection);
  const age = live.odom && live.received_at?.odom ? clock / 1000 - live.received_at.odom : Infinity;
  const odomValid = !!live.odom && (paused || age < 2);
  const canTask = connected && running && !paused && !locked && !dirty && odomValid && live.global_status?.state === 'ready' && live.local_status?.state === 'ready' && live.global_path?.points?.length > 1;
  const display = pausedDisplay && frozen ? frozen.live : live;
  const shownRecords = pausedDisplay && frozen ? frozen.records : records;
  const shownResources = pausedDisplay && frozen ? frozen.resourceHistory : resourceHistory;
  const p = display.odom?.pose?.pose, velocity = display.odom?.twist?.twist;
  async function action(path, body = {}) {
    setBusy(path); setError('');
    try {
      const response = await fetch(`/api/${path}`, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
      const data = await response.json();
      if (!response.ok) throw new Error(typeof data.detail === 'string' ? data.detail : JSON.stringify(data.detail));
      if (data.simulation) setLive(previous => ({ ...previous, simulation: data.simulation, ...(data.selection ? { selection: data.selection } : {}) }));
      if (path === 'planning/global' || path === 'simulation/start') selectionEdited.current = false;
    } catch (e) {setError(e.message);} finally {setBusy('');}
  }
  const change = (key, value) => {selectionEdited.current = true; setSelection(previous => ({ ...previous, [key]: value }));};
  const sessionKey = `${live.simulation?.state === 'stopped'}:${JSON.stringify(live.selection)}:${stamp(live.global_path?.header?.stamp)}`;
  const progress = live.local_status?.total ? Math.min(100, live.local_status.current / live.local_status.total * 100) : 0;
  return <div className="app-shell">
    <header className="app-header"><h1><span className="brand-mark"/>机器人仿真平台</h1><div className="header-status"><span className={`connection ${connected ? 'online' : ''}`}><i/>{connected ? '已连接' : '连接断开'}</span><span className="sim-time">仿真 <strong>{number(stamp(live.odom?.header?.stamp), 1)}</strong> s</span><button className="secondary display-toggle" onClick={() => {if (!pausedDisplay) setFrozen({live, records, resourceHistory}); setPausedDisplay(!pausedDisplay);}}>{pausedDisplay ? '恢复数据显示' : '暂停数据显示'}</button></div></header>
    {!connected && <div className="disconnect-banner" role="status">数据连接断开，画面保留最后状态。</div>}
    {error && <div className="error-banner" role="alert"><span>{error}</span><button aria-label="关闭提示" onClick={() => setError('')}>×</button></div>}
    <main className="workspace"><div className="visual-column"><div className="data-row">
      <section className="panel data-card map-card"><div className="panel-heading"><h2>场景概览</h2><span className="heading-meta">World 剖切</span></div><Preview world={world} mode="map"/><div className="card-bottom">{world?.file?.split('/').at(-1) || '等待场景数据'}</div></section>
      <section className="panel data-card global-card"><div className="panel-heading"><h2>全局路线</h2><StateBadge value={live.global_status?.state}/></div><Preview world={world} path={display.global_path} mode="path"/><div className="global-metrics"><span><strong>{number(pathLength(display.global_path))}</strong> m</span><span><strong>{number(display.global_timing?.elapsed_ms)}</strong> ms</span>{display.global_path && <Raw message={display.global_path}/>}</div></section>
      <section className="panel data-card local-card"><div className="panel-heading"><h2>局部实时输出</h2><StateBadge value={live.local_status?.state}/></div><div className="local-feed" aria-label="局部轨迹真实消息记录">{shownRecords.length ? shownRecords.map((record, index) => <LocalRecord key={record.stamp} record={record} index={index} message={record.stamp === stamp(display.local_trajectory?.header?.stamp) ? display.local_trajectory : null}/>) : <div className="empty">等待 /local_trajectory</div>}</div></section>
    </div>
      <Scene initialTrajectories={live.local_trajectories} world={world} dynamicObstacles={live.dynamic_obstacles} globalPath={live.global_path} localTrajectory={live.local_trajectory} odom={live.odom} sessionKey={sessionKey}/>
      <section className="panel odom-strip"><div className="odom-title"><h2>机器人状态</h2><StateBadge value={live.local_status?.state}/></div><div><span>位置 XYZ · m</span><strong>{xyz(p?.position)}</strong></div><div><span>yaw · rad</span><strong>{number(yaw(p?.orientation))}</strong></div><div><span>速度 · m/s</span><strong>{velocity ? number(Math.hypot(velocity.linear.x, velocity.linear.y, velocity.linear.z)) : '—'}</strong></div><div><span>角速度 · rad/s</span><strong>{number(velocity?.angular?.z)}</strong></div><span className={`odom-health ${!odomValid && running ? 'stale' : ''}`}>{paused ? '仿真暂停' : live.odom ? age > 2 ? '里程计已过期' : '实时' : '等待数据'}</span><details className="odom-raw"><summary>数据</summary><pre className="raw-message">{display.odom ? JSON.stringify(display.odom, null, 2) : '尚未收到消息'}</pre></details></section>
      <Resources live={display} history={shownResources}/>
    </div><aside className="panel controls"><div className="panel-heading"><h2>任务控制</h2><StateBadge value={live.simulation?.state}/></div><section className="control-section"><h3>仿真</h3><button type="button" className="wide pedestrian-toggle" aria-label="行人" aria-pressed={!!selection.dynamic_obstacles} disabled={!connected || !!busy || !config || live.simulation?.state !== 'stopped'} onClick={() => change('dynamic_obstacles', !selection.dynamic_obstacles)}><svg viewBox="0 0 24 24" width="24" height="24" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><circle cx="13" cy="4" r="2"/><path d="m7 11 4-4 4 3 4 1M11 7l-1 7-4 6m4-6 5 2 1 5"/></svg><span>行人</span><span className="pedestrian-toggle-status">{selection.dynamic_obstacles ? '已开启' : '已关闭'}</span><i className="pedestrian-switch" aria-hidden="true"/></button><div className="button-row"><button className="primary" disabled={!connected || !!busy || running || executing || live.simulation?.state === 'starting'} onClick={() => action('simulation/start', selection)}>启动仿真</button><button className="danger-subtle" disabled={!connected || !!busy || ['stopped', 'stopping'].includes(live.simulation?.state)} onClick={() => action('simulation/stop')}>停止仿真</button></div><button className="wide secondary" disabled={!connected || !running || !!busy} onClick={() => action(`simulation/${paused ? 'resume' : 'pause'}`)}>{paused ? '恢复仿真' : '暂停仿真'}</button></section>
      <section className="control-section"><h3>任务配置</h3><label>起点<select aria-label="起点" value={selection.start} disabled={locked || !config} onChange={e => change('start', e.target.value)}>{config?.points?.map(point => <option key={point.id} value={point.id}>{point.label || point.id}</option>)}</select></label><label>终点<select aria-label="终点" value={selection.goal} disabled={locked || !config} onChange={e => change('goal', e.target.value)}>{config?.points?.map(point => <option key={point.id} value={point.id}>{point.label || point.id}</option>)}</select></label><div className="algorithm-row"><label>全局算法<select aria-label="全局算法" value={selection.global_algorithm} disabled={locked} onChange={e => change('global_algorithm', e.target.value)}>{config?.algorithms?.global?.map(item => <option key={item.id} value={item.id} disabled={!item.available}>{item.id.toUpperCase()}{!item.available ? ' · 未接入' : ''}</option>)}</select></label><label>局部算法<select aria-label="局部算法" value={selection.local_algorithm} disabled={locked} onChange={e => change('local_algorithm', e.target.value)}>{config?.algorithms?.local?.map(item => <option key={item.id} value={item.id} disabled={!item.available}>{item.id.toUpperCase()}{!item.available ? ' · 未接入' : ''}</option>)}</select></label></div>{dirty && running && <p className="warning-note">配置已改变，请重新全局规划。</p>}</section>
      <section className="control-section"><h3>规划与执行</h3><button className="wide plan-button" disabled={!connected || !running || paused || locked || !odomValid} onClick={() => action('planning/global', selection)}>开始全局规划</button><button className="wide primary task-button" disabled={!canTask} onClick={() => action('task/start')}>启动任务</button></section>
      <section className="control-section task-status"><div><span>全局</span><StateBadge value={live.global_status?.state}/></div><div><span>局部 / 执行</span><StateBadge value={live.local_status?.state}/></div><div className="progress-track" aria-label="任务进度"><i style={{width:`${progress}%`}}/></div><p className="progress-value">{number(progress, 0)}%</p>{['waiting', 'avoiding', 'stale'].includes(live.local_status?.state) && <p className="warning-note">{live.local_status.reason}</p>}{[live.global_status, live.local_status, live.simulation].filter(s => s?.state === 'failed').map((s, i) => <p key={i} className="failure-text">{s.reason}</p>)}{busy && <p className="pending">正在处理…</p>}</section>
    </aside></main>
  </div>;
}
