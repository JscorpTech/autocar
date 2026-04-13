/* ═══════════════════════════════════════════════════════════════
   AutoCar Dashboard — app.js
   ═══════════════════════════════════════════════════════════════ */

'use strict';

// ─────────────────────────────────────────── 1. DOM REFS ───
const $ = id => document.getElementById(id);

const statusChip    = $('statusChip');
const statusLabel   = $('statusLabel');
const simBadge      = $('simBadge');
const connBadge     = $('connBadge');
const connText      = $('connText');

const mapSelect     = $('mapSelect');
const btnLoadMap    = $('btnLoadMap');
const btnFindPath   = $('btnFindPath');
const fileInput     = $('fileInput');

const btnStart      = $('btnStart');
const btnStop       = $('btnStop');
const btnEmergency  = $('btnEmergency');
const btnResetEnc   = $('btnResetEnc');
const progressSection = $('progressSection');
const progressBar   = $('progressBar');
const progressText  = $('progressText');

const joystickZone  = $('joystickZone');
const speedSlider   = $('speedSlider');
const speedVal      = $('speedVal');

const mapCanvas     = $('mapCanvas');
const mapEmpty      = $('mapEmpty');

const compassNeedle  = $('compassNeedle');
const compassHeading = $('compassHeading');
const compassTicks   = $('compassTicks');

const teleEncL  = $('teleEncL');
const teleEncR  = $('teleEncR');
const teleRpmL  = $('teleRpmL');
const teleRpmR  = $('teleRpmR');

const logPanel   = $('logPanel');
const logHeader  = $('logHeader');
const logBody    = $('logBody');
const logPreview = $('logPreview');
const logChevron = $('logChevron');
const btnClearLog = $('btnClearLog');

// ─────────────────────────────────────────── 2. STATE ───
let mapData     = null;   // current loaded map JSON
let pathData    = null;   // waypoints array from server
let activeWP    = -1;     // active waypoint index
let currentHeading = 0;  // degrees for smooth wraparound
let joystick    = null;
let joystickActive = false;
let keysDown    = {};
let keyInterval = null;
let mapCtx      = null;

// ─────────────────────────────────────────── 3. SOCKET ───
const socket = io();

socket.on('connect', () => {
  connBadge.dataset.connected = 'true';
  connText.textContent = 'Online';
  appendLog('Connected to server', 'ok');
  loadMapList();
});

socket.on('disconnect', () => {
  connBadge.dataset.connected = 'false';
  connText.textContent = 'Offline';
  appendLog('Disconnected', 'error');
  setStatus('idle');
});

socket.on('state', data => {
  const sim = !!data.simulate;
  simBadge.classList.toggle('hidden', !sim);

  const st = (data.status || 'idle').toLowerCase();
  setStatus(st);

  connBadge.dataset.connected = data.connected ? 'true' : 'false';
  connText.textContent = data.connected ? 'Online' : 'Offline';

  btnStart.disabled     = st === 'navigating' || !pathData || pathData.length < 2;
  btnStop.disabled      = st !== 'navigating';
  btnEmergency.disabled = false;
});

socket.on('telemetry', data => {
  updateCompass(data.heading || 0);
  updateSensors(data);
  updateTelemetry(data);
});

socket.on('map_data', data => {
  mapData = data;
  pathData = Array.isArray(data.path) ? data.path : [];
  activeWP = -1;
  progressSection.classList.add('hidden');
  mapEmpty.classList.add('hidden');
  renderMap();
  btnFindPath.disabled = false;
  btnStart.disabled = pathData.length < 2;
  appendLog(`Map loaded`, 'ok');
});

socket.on('path_found', data => {
  pathData = data.waypoints || [];
  renderMap();
  btnStart.disabled = pathData.length < 2;
  appendLog(`Path: ${pathData.length} waypoints`, 'ok');
});

socket.on('nav_progress', data => {
  activeWP = data.current || 0;
  const total = data.total || 1;
  const pct = Math.round(((activeWP + 1) / total) * 100);
  progressBar.style.width = pct + '%';
  progressText.textContent = `${activeWP + 1} / ${total}`;
  progressSection.classList.remove('hidden');
  renderMap();
});

socket.on('log', data => {
  const lvl = (data.level || '').toLowerCase();
  const cls = lvl === 'warn' || lvl === 'warning' ? 'log-warn'
            : lvl === 'error' ? 'log-error'
            : lvl === 'ok' || lvl === 'success' ? 'log-ok'
            : '';
  appendLog(data.msg || '', cls);
});

// ─────────────────────────────────────────── 4. STATUS ───
function setStatus(st) {
  const labels = {
    idle:       'Idle',
    navigating: 'Navigating',
    manual:     'Manual',
    stopped:    'Stopped',
    emergency:  'Emergency Stop',
    obstacle:   'Obstacle!',
  };
  statusChip.dataset.status = st;
  statusLabel.textContent = labels[st] || st;
}

// ─────────────────────────────────────────── 5. COMPASS ───
(function initCompassTicks() {
  const cx = 100, cy = 100, r0 = 88, rMin = 83, rMaj = 80;
  for (let deg = 0; deg < 360; deg += 10) {
    const isMajor = deg % 30 === 0;
    const r1 = isMajor ? rMaj : rMin;
    const a  = (deg - 90) * Math.PI / 180;
    const x1 = cx + r0 * Math.cos(a);
    const y1 = cy + r0 * Math.sin(a);
    const x2 = cx + r1 * Math.cos(a);
    const y2 = cy + r1 * Math.sin(a);
    const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
    line.setAttribute('x1', x1.toFixed(2));
    line.setAttribute('y1', y1.toFixed(2));
    line.setAttribute('x2', x2.toFixed(2));
    line.setAttribute('y2', y2.toFixed(2));
    line.setAttribute('stroke', isMajor ? '#484f58' : '#30363d');
    line.setAttribute('stroke-width', isMajor ? '1.5' : '0.8');
    compassTicks.appendChild(line);
  }
})();

function updateCompass(heading) {
  // Shortest rotation path (no wrap jump)
  let delta = heading - currentHeading;
  if (delta > 180)  delta -= 360;
  if (delta < -180) delta += 360;
  currentHeading += delta;

  compassNeedle.style.transform = `rotate(${currentHeading}deg)`;
  compassHeading.textContent = (((heading % 360) + 360) % 360).toFixed(1) + '°';
}

// ─────────────────────────────────────────── 6. SENSORS ───

// Sensor beam geometry: [originX, originY, dirAngle(deg), halfSpread(deg)]
// SVG viewBox 0 0 240 342 — car spans ~x:82-158, y:67-275
const BEAMS = {
  'beam-front': { ox: 120, oy: 67,  dir: -90, spread: 18 },
  'beam-fl':    { ox: 88,  oy: 85,  dir: -135, spread: 18 },
  'beam-fr':    { ox: 152, oy: 85,  dir: -45,  spread: 18 },
  'beam-left':  { ox: 82,  oy: 171, dir: 180,  spread: 15 },
  'beam-right': { ox: 158, oy: 171, dir: 0,    spread: 15 },
  'beam-rear':  { ox: 120, oy: 275, dir: 90,   spread: 18 },
};

// Max sensor display distance in meters; maps to MAX_PX pixels beam length
const MAX_DIST_M = 3.0;
const MAX_PX     = 90;

function beamClass(dist) {
  if (dist === null || dist === undefined || dist <= 0) return 'beam-none';
  if (dist < 0.5)  return 'beam-danger';
  if (dist < 1.5)  return 'beam-warn';
  return 'beam-safe';
}

function beamPoints(cfg, dist) {
  if (!dist || dist <= 0) return `${cfg.ox},${cfg.oy} ${cfg.ox},${cfg.oy} ${cfg.ox},${cfg.oy}`;
  const len  = Math.min(dist / MAX_DIST_M, 1) * MAX_PX;
  const dRad = cfg.dir    * Math.PI / 180;
  const sRad = cfg.spread * Math.PI / 180;

  const lRad = dRad - sRad;
  const rRad = dRad + sRad;

  const tipX  = cfg.ox + len * Math.cos(dRad);
  const tipY  = cfg.oy + len * Math.sin(dRad);
  const edgL  = len * 0.92;
  const lx    = cfg.ox + edgL * Math.cos(lRad);
  const ly    = cfg.oy + edgL * Math.sin(lRad);
  const rx    = cfg.ox + edgL * Math.cos(rRad);
  const ry    = cfg.oy + edgL * Math.sin(rRad);

  return `${cfg.ox},${cfg.oy} ${lx.toFixed(1)},${ly.toFixed(1)} ${tipX.toFixed(1)},${tipY.toFixed(1)} ${rx.toFixed(1)},${ry.toFixed(1)}`;
}

function updateSensors(data) {
  const sensors = {
    'beam-front': data.distance,
    'beam-fl':    data.dist_front_left,
    'beam-fr':    data.dist_front_right,
    'beam-left':  data.dist_left,
    'beam-right': data.dist_right,
    'beam-rear':  data.dist_rear,
  };
  const labels = {
    'beam-front': 'lbl-front',
    'beam-fl':    'lbl-fl',
    'beam-fr':    'lbl-fr',
    'beam-left':  'lbl-left',
    'beam-right': 'lbl-right',
    'beam-rear':  'lbl-rear',
  };

  let anyDanger = false;
  for (const [id, dist] of Object.entries(sensors)) {
    const el = $(id);
    if (!el) continue;
    const cls = beamClass(dist);
    el.setAttribute('class', `beam ${cls}`);
    el.setAttribute('points', beamPoints(BEAMS[id], dist));
    if (cls === 'beam-danger') anyDanger = true;

    const lbl = $(labels[id]);
    if (lbl) lbl.textContent = (dist > 0) ? dist.toFixed(2) + 'm' : '—';
  }

  const bodyRect = $('carBodyRect');
  if (bodyRect) {
    bodyRect.classList.toggle('obstacle-alert', anyDanger || !!data.obstacle);
  }
}

// ─────────────────────────────────────────── 7. TELEMETRY ───
function updateTelemetry(data) {
  teleEncL.textContent = data.encoder_left  ?? '—';
  teleEncR.textContent = data.encoder_right ?? '—';
  teleRpmL.textContent = (data.rpm_left  != null) ? data.rpm_left.toFixed(1)  : '—';
  teleRpmR.textContent = (data.rpm_right != null) ? data.rpm_right.toFixed(1) : '—';
}

// ─────────────────────────────────────────── 8. MAP CANVAS ───
function renderMap() {
  if (!mapData) return;

  const canvas = mapCanvas;
  const grid   = mapData.grid;
  const rows   = grid.length;
  const cols   = grid[0].length;

  // Resize canvas to parent
  canvas.width  = canvas.offsetWidth  * devicePixelRatio;
  canvas.height = canvas.offsetHeight * devicePixelRatio;
  const ctx = canvas.getContext('2d');
  ctx.scale(devicePixelRatio, devicePixelRatio);
  const W = canvas.offsetWidth;
  const H = canvas.offsetHeight;

  const padding = 24;
  const cellW = (W - padding * 2) / cols;
  const cellH = (H - padding * 2) / rows;
  const cell  = Math.min(cellW, cellH, 64);
  const offX  = (W - cell * cols) / 2;
  const offY  = (H - cell * rows) / 2;

  // Background
  ctx.fillStyle = '#0d1117';
  ctx.fillRect(0, 0, W, H);

  // Dot grid
  ctx.fillStyle = '#21262d';
  for (let r = 0; r <= rows; r++) {
    for (let c = 0; c <= cols; c++) {
      ctx.beginPath();
      ctx.arc(offX + c * cell, offY + r * cell, 1, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // Cells
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      const x = offX + c * cell;
      const y = offY + r * cell;
      if (grid[r][c] === 0) {
        // Wall
        const gap = 2;
        ctx.fillStyle = '#1c2128';
        ctx.strokeStyle = '#30363d';
        ctx.lineWidth = 0.5;
        roundRect(ctx, x + gap, y + gap, cell - gap * 2, cell - gap * 2, 4);
        ctx.fill();
        ctx.stroke();
      }
    }
  }

  // Path
  if (pathData && pathData.length > 1) {
    ctx.save();
    ctx.shadowColor  = '#2f81f7';
    ctx.shadowBlur   = 8;
    ctx.strokeStyle  = '#2f81f7';
    ctx.lineWidth    = 2.5;
    ctx.lineJoin     = 'round';
    ctx.lineCap      = 'round';
    ctx.beginPath();
    pathData.forEach(([r, c], i) => {
      const x = offX + c * cell + cell / 2;
      const y = offY + r * cell + cell / 2;
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
    ctx.restore();

    // Waypoint circles
    pathData.forEach(([r, c], i) => {
      const x = offX + c * cell + cell / 2;
      const y = offY + r * cell + cell / 2;
      const isActive = i === activeWP;
      ctx.beginPath();
      ctx.arc(x, y, isActive ? 9 : 5, 0, Math.PI * 2);
      ctx.fillStyle = isActive ? '#2f81f7' : '#1f6feb88';
      ctx.fill();
      if (isActive) {
        ctx.save();
        ctx.shadowColor = '#2f81f7';
        ctx.shadowBlur  = 12;
        ctx.strokeStyle = '#2f81f7';
        ctx.lineWidth   = 1.5;
        ctx.stroke();
        ctx.restore();
      }
      if (cell >= 28 && pathData.length <= 20) {
        ctx.fillStyle = '#e6edf3';
        ctx.font = `bold ${Math.max(8, cell * 0.22)}px Inter, sans-serif`;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(i + 1, x, y);
      }
    });
  }

  // Start marker
  const [sr, sc] = mapData.start;
  drawMarker(ctx, offX + sc * cell + cell / 2, offY + sr * cell + cell / 2, '#3fb950', 'S', cell);

  // End marker
  const [er, ec] = mapData.end;
  drawMarker(ctx, offX + ec * cell + cell / 2, offY + er * cell + cell / 2, '#f85149', 'E', cell);
}

function drawMarker(ctx, x, y, color, label, cell) {
  ctx.save();
  ctx.beginPath();
  ctx.arc(x, y, cell * 0.28, 0, Math.PI * 2);
  ctx.fillStyle = color + '33';
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = color;
  ctx.font = `700 ${Math.max(9, cell * 0.3)}px Inter, sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(label, x, y);
  ctx.restore();
}

function roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.lineTo(x + w - r, y);
  ctx.quadraticCurveTo(x + w, y, x + w, y + r);
  ctx.lineTo(x + w, y + h - r);
  ctx.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
  ctx.lineTo(x + r, y + h);
  ctx.quadraticCurveTo(x, y + h, x, y + h - r);
  ctx.lineTo(x, y + r);
  ctx.quadraticCurveTo(x, y, x + r, y);
  ctx.closePath();
}

window.addEventListener('resize', () => {
  if (mapData) renderMap();
});

// ─────────────────────────────────────────── 9. JOYSTICK ───
function initJoystick() {
  joystick = nipplejs.create({
    zone: joystickZone,
    mode: 'static',
    position: { left: '50%', top: '50%' },
    color: '#2f81f7',
    size: 120,
    restOpacity: 0.7,
  });

  joystick.on('move', (evt, data) => {
    joystickActive = true;
    const speed    = parseInt(speedSlider.value, 10);
    const fwd      = -(data.vector.y);            // nipplejs y: up=positive
    const right    = data.vector.x;
    const motorSpd = Math.round(fwd   * speed);
    const angle    = Math.round(right * 20);      // -20..+20 deg
    socket.emit('manual_control', { speed: motorSpd, angle });
  });

  joystick.on('end', () => {
    joystickActive = false;
    socket.emit('manual_stop');
  });
}

// ─────────────────────────────────────────── 10. KEYBOARD ───
const KEY_MAP = {
  'ArrowUp':    { speed:  1, angle:  0 },
  'ArrowDown':  { speed: -1, angle:  0 },
  'ArrowLeft':  { speed:  1, angle: -1 },
  'ArrowRight': { speed:  1, angle:  1 },
  'w': { speed:  1, angle:  0 },
  's': { speed: -1, angle:  0 },
  'a': { speed:  1, angle: -1 },
  'd': { speed:  1, angle:  1 },
  'W': { speed:  1, angle:  0 },
  'S': { speed: -1, angle:  0 },
  'A': { speed:  1, angle: -1 },
  'D': { speed:  1, angle:  1 },
};

function sendKeyDrive() {
  if (joystickActive) return;
  const speed = parseInt(speedSlider.value, 10);
  let sv = 0, av = 0;
  for (const key of Object.keys(keysDown)) {
    const m = KEY_MAP[key];
    if (m) { sv += m.speed; av += m.angle; }
  }
  if (sv !== 0 || av !== 0) {
    socket.emit('manual_control', {
      speed: Math.sign(sv) * speed,
      angle: Math.max(-20, Math.min(20, av * 20)),
    });
  } else {
    socket.emit('manual_stop');
  }
}

document.addEventListener('keydown', e => {
  // Space = emergency
  if (e.code === 'Space') {
    e.preventDefault();
    socket.emit('emergency_stop');
    return;
  }
  if (KEY_MAP[e.key] && !e.ctrlKey && !e.metaKey && !e.altKey) {
    e.preventDefault();
    if (!keysDown[e.key]) {
      keysDown[e.key] = true;
      if (!keyInterval) {
        keyInterval = setInterval(sendKeyDrive, 50);
      }
    }
  }
});

document.addEventListener('keyup', e => {
  if (KEY_MAP[e.key]) {
    delete keysDown[e.key];
    if (Object.keys(keysDown).length === 0) {
      clearInterval(keyInterval);
      keyInterval = null;
      socket.emit('manual_stop');
    }
  }
});

// ─────────────────────────────────────────── 11. LOG ───
const MAX_LOG = 200;
let logCount = 0;

function appendLog(msg, cls) {
  const now  = new Date();
  const time = now.toTimeString().slice(0, 8);

  const entry = document.createElement('div');
  entry.className = `log-entry${cls ? ' ' + cls : ''}`;
  entry.innerHTML = `<span class="log-time">${time}</span><span class="log-msg">${escHtml(msg)}</span>`;
  logBody.appendChild(entry);
  logCount++;

  // Trim old entries
  while (logBody.children.length > MAX_LOG) {
    logBody.removeChild(logBody.firstChild);
  }

  logBody.scrollTop = logBody.scrollHeight;
  logPreview.textContent = msg;
}

function escHtml(s) {
  return s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}

logHeader.addEventListener('click', e => {
  if (e.target === btnClearLog || btnClearLog.contains(e.target)) return;
  logPanel.classList.toggle('expanded');
});

btnClearLog.addEventListener('click', e => {
  e.stopPropagation();
  logBody.innerHTML = '';
  logPreview.textContent = '—';
});

// ─────────────────────────────────────────── 12. MAP MANAGEMENT ───
function loadMapList() {
  fetch('/api/maps')
    .then(r => r.json())
    .then(maps => {
      mapSelect.innerHTML = '<option value="">Select map...</option>';
      maps.forEach(m => {
        const opt = document.createElement('option');
        opt.value = m.filename;
        opt.textContent = `${m.name} (${m.size})`;
        mapSelect.appendChild(opt);
      });
      btnLoadMap.disabled = true;
    })
    .catch(() => appendLog('Failed to fetch map list', 'error'));
}

mapSelect.addEventListener('change', () => {
  btnLoadMap.disabled = !mapSelect.value;
  btnFindPath.disabled = true;
  btnStart.disabled    = true;
  pathData  = null;
  activeWP  = -1;
});

btnLoadMap.addEventListener('click', () => {
  const name = mapSelect.value;
  if (!name) return;
  socket.emit('load_map', { filename: name });
});

btnFindPath.addEventListener('click', () => {
  socket.emit('find_path');
});

fileInput.addEventListener('change', () => {
  const file = fileInput.files[0];
  if (!file) return;
  const fd = new FormData();
  fd.append('file', file);
  fetch('/api/map/upload', { method: 'POST', body: fd })
    .then(r => r.json())
    .then(d => {
      if (d.success) {
        appendLog(`Uploaded: ${d.filename}`, 'ok');
        loadMapList();
      } else {
        appendLog(`Upload failed: ${d.error || '?'}`, 'error');
      }
    })
    .catch(() => appendLog('Upload error', 'error'));
  fileInput.value = '';
});

// ─────────────────────────────────────────── 13. NAV BUTTONS ───
btnStart.addEventListener('click', () => {
  socket.emit('start_navigation');
  setStatus('navigating');
  btnStart.disabled = true;
  btnStop.disabled  = false;
});

btnStop.addEventListener('click', () => {
  socket.emit('stop_navigation');
  setStatus('stopped');
  btnStop.disabled  = true;
  btnStart.disabled = false;
});

btnEmergency.addEventListener('click', () => {
  socket.emit('emergency_stop');
  setStatus('emergency');
});

btnResetEnc.addEventListener('click', () => {
  socket.emit('reset_encoders');
  appendLog('Encoders reset requested', '');
});

// ─────────────────────────────────────────── 14. SPEED SLIDER ───
speedSlider.addEventListener('input', () => {
  speedVal.textContent = speedSlider.value;
});

// ─────────────────────────────────────────── 15. BOOTSTRAP ───
initJoystick();
appendLog('Dashboard ready', 'ok');
