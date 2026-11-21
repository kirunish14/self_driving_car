(() => {
'use strict';
const $ = id => document.getElementById(id);
const canvas = $('view'), ctx = canvas.getContext('2d');

// ------------------------------------------------------------------ state
const S = {
  info: null, map: null, mapId: null,
  pose: null, localized: false, speed: 0, nav: null,
  scan: [], plan: [],
  mode: 'pan', follow: false, confirm: false,
  view: { s: 1, tx: 0, ty: 0 },
  drag: null, pending: null,
  videoOpen: false, videoActive: false,
};
let pin = '';
try { pin = sessionStorage.getItem('pin') || ''; S.follow = localStorage.getItem('follow') === '1'; S.confirm = localStorage.getItem('confirm') === '1'; } catch (e) {}
let es = null, dirty = false;

// ------------------------------------------------------------------ helpers
function toast(msg, err) {
  const t = $('toast'); t.textContent = msg; t.className = 'show' + (err ? ' err' : '');
  clearTimeout(toast.t); toast.t = setTimeout(() => { t.className = ''; }, err ? 4500 : 2500);
}
async function api(path, body) {
  const headers = {}; if (pin) headers['X-PIN'] = pin;
  const opt = { headers };
  if (body !== undefined) { opt.method = 'POST'; opt.headers['Content-Type'] = 'application/json'; opt.body = JSON.stringify(body); }
  const r = await fetch(path, opt);
  let data = null; try { data = await r.json(); } catch (e) {}
  if (r.status === 401) { const e = new Error('PIN required'); e.auth = true; throw e; }
  if (!r.ok) throw new Error((data && data.error) || ('HTTP ' + r.status));
  return data;
}
const q = () => (pin ? '?pin=' + encodeURIComponent(pin) : '');
const deg = r => Math.round(r * 180 / Math.PI);
const redraw = () => { if (!dirty) { dirty = true; requestAnimationFrame(draw); } };

// ------------------------------------------------------------------ map <-> screen
function w2s(x, y) { const m = S.map, v = S.view; return [(x - m.ox) / m.res * v.s + v.tx, (m.h - (y - m.oy) / m.res) * v.s + v.ty]; }
function s2w(sx, sy) { const m = S.map, v = S.view; return [m.ox + (sx - v.tx) / v.s * m.res, m.oy + (m.h - (sy - v.ty) / v.s) * m.res]; }
function fit() {
  if (!S.map) return;
  const w = canvas.clientWidth, h = canvas.clientHeight, s = Math.min(w / S.map.w, h / S.map.h) * 0.96;
  S.view = { s, tx: (w - S.map.w * s) / 2, ty: (h - S.map.h * s) / 2 };
  redraw();
}
function zoomAt(cx, cy, f) {
  const v = S.view, s = Math.max(0.05, Math.min(60, v.s * f)), k = s / v.s;
  v.tx = cx - (cx - v.tx) * k; v.ty = cy - (cy - v.ty) * k; v.s = s; redraw();
}
function centerOnCar() {
  if (!S.map || !S.pose) return;
  const [sx, sy] = w2s(S.pose.x, S.pose.y);
  S.view.tx += canvas.clientWidth / 2 - sx; S.view.ty += canvas.clientHeight / 2 - sy;
}

// ------------------------------------------------------------------ drawing
function arrow(x0, y0, x1, y1, col, w) {
  ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = w; ctx.lineCap = 'round';
  ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
  const a = Math.atan2(y1 - y0, x1 - x0), L = 5 + w * 2.2;
  ctx.beginPath(); ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - L * Math.cos(a - 0.45), y1 - L * Math.sin(a - 0.45));
  ctx.lineTo(x1 - L * Math.cos(a + 0.45), y1 - L * Math.sin(a + 0.45)); ctx.closePath(); ctx.fill();
}
function poseArrow(x, y, yaw, col, lenM) {
  const [a, b] = w2s(x, y), [c, d] = w2s(x + Math.cos(yaw) * lenM, y + Math.sin(yaw) * lenM);
  ctx.beginPath(); ctx.arc(a, b, 6, 0, 6.2832); ctx.fillStyle = col; ctx.fill();
  arrow(a, b, c, d, col, 3);
}
function draw() {
  dirty = false;
  const dpr = window.devicePixelRatio || 1, w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) { canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr); }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = '#0d1014'; ctx.fillRect(0, 0, w, h);
  if (!S.map) {
    ctx.fillStyle = '#93a1ae'; ctx.font = '16px system-ui'; ctx.textAlign = 'center';
    ctx.fillText('No map loaded - choose a map above and press "Load map"', w / 2, h / 2); return;
  }
  const v = S.view;
  ctx.save(); ctx.translate(v.tx, v.ty); ctx.scale(v.s, v.s); ctx.imageSmoothingEnabled = v.s < 1; ctx.drawImage(S.map.img, 0, 0); ctx.restore();

  if (S.plan.length > 3) {                                   // planned path
    ctx.strokeStyle = '#2ee66b'; ctx.lineWidth = 3; ctx.lineJoin = 'round'; ctx.beginPath();
    for (let i = 0; i < S.plan.length; i += 2) { const [x, y] = w2s(S.plan[i], S.plan[i + 1]); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }
    ctx.stroke();
  }
  ctx.fillStyle = '#ff3b3b';                                  // lidar
  for (let i = 0; i < S.scan.length; i += 2) { const [x, y] = w2s(S.scan[i], S.scan[i + 1]); ctx.fillRect(x - 1.2, y - 1.2, 2.4, 2.4); }

  const g = S.nav && S.nav.goal; if (g) poseArrow(g.x, g.y, g.yaw, '#ffa500', 0.7);
  if (S.pending) poseArrow(S.pending.x, S.pending.y, S.pending.yaw, '#ffd24a', 0.7);

  if (S.pose && S.info) {                                     // the car
    const c = S.info.car, hw = c.width / 2, cs = Math.cos(S.pose.yaw), sn = Math.sin(S.pose.yaw);
    const body = [[c.front, hw], [c.front, -hw], [-c.rear, -hw], [-c.rear, hw]].map(([px, py]) => w2s(S.pose.x + cs * px - sn * py, S.pose.y + sn * px + cs * py));
    ctx.beginPath(); body.forEach(([x, y], i) => i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)); ctx.closePath();
    ctx.fillStyle = 'rgba(61,155,255,.35)'; ctx.fill(); ctx.strokeStyle = '#3d9bff'; ctx.lineWidth = 2; ctx.stroke();
    const [ax, ay] = w2s(S.pose.x, S.pose.y), [bx, by] = w2s(S.pose.x + cs * (c.front + 0.35), S.pose.y + sn * (c.front + 0.35));
    arrow(ax, ay, bx, by, '#ffffff', 2);
    ctx.beginPath(); ctx.arc(ax, ay, 3.5, 0, 6.2832); ctx.fillStyle = '#fff'; ctx.fill();
  }
  if (S.drag) {                                               // arrow being dragged
    const col = S.mode === 'goal' ? '#ffa500' : '#ffd24a';
    ctx.beginPath(); ctx.arc(S.drag.sx, S.drag.sy, 7, 0, 6.2832); ctx.fillStyle = col; ctx.fill();
    arrow(S.drag.sx, S.drag.sy, S.drag.cx, S.drag.cy, col, 4);
  }
}

// ------------------------------------------------------------------ modes
const HINTS = { pan: '', pose: 'SET POSE: touch where the car is, drag toward the direction it faces, release.', goal: 'SET GOAL: touch the destination, drag toward the direction the car should face, release.' };
function setMode(m) {
  S.mode = m; S.drag = null;
  ['pan', 'pose', 'goal'].forEach(k => $('m' + k[0].toUpperCase() + k.slice(1)).classList.toggle('active', k === m));
  $('hint').textContent = HINTS[m]; canvas.style.cursor = m === 'pan' ? 'grab' : 'crosshair'; redraw();
}
async function sendPose(x, y, yaw) {
  try { await api('/api/initialpose', { x, y, yaw }); toast('Initial pose sent - wait for the car to appear'); } catch (e) { toast(e.message, true); }
}
async function sendGoal(x, y, yaw) {
  try { await api('/api/goal', { x, y, yaw }); toast('Goal sent'); } catch (e) { toast(e.message, true); }
}
function finishDrag(d) {
  const dist = Math.hypot(d.cx - d.sx, d.cy - d.sy);
  if (dist < 14) { toast('Touch and DRAG to show the direction', true); return; }
  const [x, y] = s2w(d.sx, d.sy), yaw = Math.atan2(-(d.cy - d.sy), d.cx - d.sx);
  if (S.mode === 'pose') { sendPose(x, y, yaw); setMode('pan'); }
  else if (S.confirm) { S.pending = { x, y, yaw }; $('confirmText').textContent = `Go to x ${x.toFixed(2)}, y ${y.toFixed(2)}, ${deg(yaw)}\u00b0 ?`; $('confirm').hidden = false; setMode('pan'); }
  else { sendGoal(x, y, yaw); setMode('pan'); }
}

// ------------------------------------------------------------------ pointer input (mouse + touch)
const ptrs = new Map(); let pinch = null;
const pos = e => { const r = canvas.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; };
canvas.addEventListener('pointerdown', e => {
  canvas.setPointerCapture(e.pointerId); const [x, y] = pos(e); ptrs.set(e.pointerId, { x, y });
  if (ptrs.size === 2) {                                      // second finger: pinch zoom, cancels any arrow
    S.drag = null; const [a, b] = [...ptrs.values()]; pinch = { d: Math.hypot(a.x - b.x, a.y - b.y), cx: (a.x + b.x) / 2, cy: (a.y + b.y) / 2 }; redraw();
  } else if (ptrs.size === 1 && S.map && (S.mode === 'pose' || S.mode === 'goal')) { S.drag = { sx: x, sy: y, cx: x, cy: y }; redraw(); }
});
canvas.addEventListener('pointermove', e => {
  const p = ptrs.get(e.pointerId); if (!p) return; const [x, y] = pos(e), dx = x - p.x, dy = y - p.y; p.x = x; p.y = y;
  if (ptrs.size >= 2 && pinch) {
    const [a, b] = [...ptrs.values()], d = Math.hypot(a.x - b.x, a.y - b.y), cx = (a.x + b.x) / 2, cy = (a.y + b.y) / 2;
    S.view.tx += cx - pinch.cx; S.view.ty += cy - pinch.cy; if (pinch.d > 0) zoomAt(cx, cy, d / pinch.d); pinch = { d, cx, cy };
  } else if (S.drag) { S.drag.cx = x; S.drag.cy = y; redraw(); }
  else if (S.mode === 'pan' && ptrs.size === 1) { S.view.tx += dx; S.view.ty += dy; redraw(); }
});
function endPtr(e, cancel) {
  if (!ptrs.has(e.pointerId)) return; ptrs.delete(e.pointerId);
  if (ptrs.size < 2) pinch = null;
  if (S.drag && ptrs.size === 0) { const d = S.drag; S.drag = null; if (!cancel) finishDrag(d); redraw(); }
}
canvas.addEventListener('pointerup', e => endPtr(e, false));
canvas.addEventListener('pointercancel', e => endPtr(e, true));
canvas.addEventListener('wheel', e => { e.preventDefault(); const [x, y] = pos(e); zoomAt(x, y, e.deltaY < 0 ? 1.15 : 1 / 1.15); }, { passive: false });
window.addEventListener('resize', redraw);
new ResizeObserver(() => redraw()).observe(canvas);

// ------------------------------------------------------------------ buttons
$('mPan').onclick = () => setMode('pan'); $('mPose').onclick = () => setMode('pose'); $('mGoal').onclick = () => setMode('goal');
$('bFit').onclick = fit;
$('cFollow').checked = S.follow; $('cConfirm').checked = S.confirm;
$('cFollow').onchange = e => { S.follow = e.target.checked; try { localStorage.setItem('follow', S.follow ? '1' : '0'); } catch (x) {} };
$('cConfirm').onchange = e => { S.confirm = e.target.checked; try { localStorage.setItem('confirm', S.confirm ? '1' : '0'); } catch (x) {} };
$('confirmGo').onclick = () => { const p = S.pending; S.pending = null; $('confirm').hidden = true; if (p) sendGoal(p.x, p.y, p.yaw); redraw(); };
$('confirmNo').onclick = () => { S.pending = null; $('confirm').hidden = true; redraw(); };
$('stop').onclick = async () => { try { await api('/api/cancel', {}); toast('STOP sent'); } catch (e) { toast('STOP failed: ' + e.message, true); } };
window.addEventListener('keydown', e => { if (e.key === 'Escape') $('stop').click(); });
$('mapLoad').onclick = async () => {
  const id = $('mapSelect').value; if (!id) return;
  $('mapLoad').disabled = true;
  try { await api('/api/select_map', { id }); toast('Map loaded - now set the initial pose'); setMode('pose'); } catch (e) { toast(e.message, true); }
  $('mapLoad').disabled = false;
};

// ------------------------------------------------------------------ camera panel
let vTimer = null;
function startVideo() {
  clearTimeout(vTimer);
  const img = $('video');
  img.onload = () => $('videoMsg').classList.add('hide');
  img.onerror = () => { $('videoMsg').classList.remove('hide'); clearTimeout(vTimer); vTimer = setTimeout(() => { if (S.videoOpen) startVideo(); }, 2000); };
  img.src = '/api/video' + q() + (q() ? '&' : '?') + 't=' + Date.now();
}
function stopVideo() { clearTimeout(vTimer); const img = $('video'); img.onload = img.onerror = null; img.removeAttribute('src'); $('videoMsg').classList.remove('hide'); }
$('bCam').onclick = () => { S.videoOpen = !S.videoOpen; $('videoPanel').hidden = !S.videoOpen; if (S.videoOpen) startVideo(); else stopVideo(); };
$('videoClose').onclick = () => { S.videoOpen = false; $('videoPanel').hidden = true; stopVideo(); };
$('videoBig').onclick = () => $('videoPanel').classList.toggle('big');
function videoStateChanged(active) {
  const was = S.videoActive; S.videoActive = active;
  if (!S.videoOpen) return;
  if (!active) $('videoMsg').classList.remove('hide');
  else { $('videoMsg').classList.add('hide'); if (!was) startVideo(); }   // camera came (back) -> reconnect the stream
}

// ------------------------------------------------------------------ live data from the car
function setConn(ok) { $('conn').className = 'dot ' + (ok ? 'ok' : 'bad'); if (!ok) { $('chipLoc').className = 'chip bad'; $('chipLoc').textContent = 'no connection'; } }
function connect() {
  if (es) es.close();
  es = new EventSource('/api/events' + q());
  es.onopen = () => setConn(true);
  es.onerror = () => { setConn(false); if (es.readyState === 2) setTimeout(connect, 2000); };
  es.addEventListener('scan', e => { S.scan = JSON.parse(e.data).pts; redraw(); });
  es.addEventListener('plan', e => { S.plan = JSON.parse(e.data).pts; redraw(); });
  es.addEventListener('state', e => onState(JSON.parse(e.data)));
}
const NAVTXT = { idle: 'idle', sending: 'sending goal...', navigating: 'navigating', succeeded: 'goal reached', cancelled: 'cancelled', failed: 'FAILED', rejected: 'goal rejected' };
function onState(d) {
  S.pose = d.pose; S.localized = d.localized; S.speed = d.speed; S.nav = d.nav;
  const chip = $('chipLoc'); chip.className = 'chip ' + (d.localized ? 'ok' : 'bad');
  chip.textContent = d.localized ? 'localized' : 'not localized - set pose';
  $('iPose').textContent = d.pose ? `x ${d.pose.x.toFixed(2)}  y ${d.pose.y.toFixed(2)}  ${deg(d.pose.yaw)}\u00b0` : '-';
  $('iSpeed').textContent = d.speed.toFixed(2) + ' m/s';
  $('iNav').textContent = (NAVTXT[d.nav.state] || d.nav.state) + (d.nav.active && d.nav.distance != null ? `  (${d.nav.distance.toFixed(1)} m to go)` : '') + (d.nav.active && d.nav.recoveries > 0 ? `  - obstacle: wait/reverse step ${d.nav.recoveries}` : '');
  $('mapLoad').disabled = d.nav.active;
  if (d.map_id !== S.mapId) { S.mapId = d.map_id; $('mapSelect').value = d.map_id || ''; if (d.map_id) loadMap(d.map_id); else { S.map = null; $('iMap').textContent = '-'; } }
  videoStateChanged(d.video);
  if (S.follow && S.pose && S.map) centerOnCar();
  redraw();
}

// ------------------------------------------------------------------ maps
async function loadMap(id) {
  try {
    const m = await api('/api/map/' + id + '/meta'), img = new Image();
    img.onload = () => { S.map = { id, img, w: m.width, h: m.height, res: m.resolution, ox: m.origin[0], oy: m.origin[1] }; $('iMap').textContent = m.name; S.plan = []; fit(); };
    img.onerror = () => toast('Could not load the map image', true);
    img.src = m.image_url + q() + (q() ? '&' : '?') + 't=' + Date.now();
  } catch (e) { toast('Map: ' + e.message, true); }
}
async function loadMapList() {
  const r = await api('/api/maps'), sel = $('mapSelect'); sel.innerHTML = '';
  if (!r.maps.length) { sel.innerHTML = '<option value="">no maps found (~/car_maps)</option>'; return; }
  r.maps.forEach(m => { const o = document.createElement('option'); o.value = m.id; o.textContent = m.name; o.title = m.path; sel.appendChild(o); });
  if (r.current) sel.value = r.current;
}

// ------------------------------------------------------------------ start
async function boot() {
  for (;;) {
    try {
      S.info = await api('/api/info');
      if (S.info.pin_required && !S.info.authorized) {
        const p = window.prompt('Enter the dashboard PIN'); if (p === null) { toast('PIN required', true); return; }
        pin = p; try { sessionStorage.setItem('pin', pin); } catch (e) {} continue;
      }
      break;
    } catch (e) { toast('Cannot reach the car: ' + e.message, true); await new Promise(r => setTimeout(r, 2000)); }
  }
  try { await loadMapList(); } catch (e) { toast(e.message, true); }
  setMode('pan'); connect(); redraw();
}
boot();
})();
