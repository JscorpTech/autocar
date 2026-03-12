// dashboard frontend

const socket = io();

// ─── DOM Elements ───
const els = {
  statusChip: document.getElementById('statusChip'),
  statusText: document.querySelector('.status-text'),
  connBadge: document.getElementById('connBadge'),
  connText: document.getElementById('connText'),
  mapSelect: document.getElementById('mapSelect'),
  btnLoadMap: document.getElementById('btnLoadMap'),
  btnFindPath: document.getElementById('btnFindPath'),
  btnStart: document.getElementById('btnStart'),
  btnStop: document.getElementById('btnStop'),
  btnEmergency: document.getElementById('btnEmergency'),
  btnClearLog: document.getElementById('btnClearLog'),
  fileInput: document.getElementById('fileInput'),
  mapCanvas: document.getElementById('mapCanvas'),
  mapPlaceholder: document.getElementById('mapPlaceholder'),
  compassNeedle: document.getElementById('compassNeedle'),
  compassValue: document.getElementById('compassValue'),
  teleEncL: document.getElementById('teleEncL'),
  teleEncR: document.getElementById('teleEncR'),
  teleRpmL: document.getElementById('teleRpmL'),
  teleRpmR: document.getElementById('teleRpmR'),
  teleDist: document.getElementById('teleDist'),
  distBar: document.getElementById('distBar'),
  obstacleIndicator: document.getElementById('obstacleIndicator'),
  progressSection: document.getElementById('progressSection'),
  progressText: document.getElementById('progressText'),
  progressFill: document.getElementById('progressFill'),
  logBody: document.getElementById('logBody'),
  speedSlider: document.getElementById('speedSlider'),
  speedVal: document.getElementById('speedVal'),
};

// ─── State ───
let currentState = {};
let mapData = null;
let manualSpeed = 150;

// ─── Init ───
document.addEventListener('DOMContentLoaded', () => {
  loadMapList();
  setupEventListeners();
  setupKeyboard();
});

// ─── Load Maps List ───
function loadMapList() {
  fetch('/api/maps')
    .then(r => r.json())
    .then(maps => {
      els.mapSelect.innerHTML = '<option value="">Xarita tanlang...</option>';
      maps.forEach(m => {
        const opt = document.createElement('option');
        opt.value = m.filename;
        opt.textContent = `${m.name} (${m.size})`;
        els.mapSelect.appendChild(opt);
      });
    });
}

// ─── Event Listeners ───
function setupEventListeners() {
  els.mapSelect.addEventListener('change', () => {
    els.btnLoadMap.disabled = !els.mapSelect.value;
  });

  els.btnLoadMap.addEventListener('click', () => {
    if (els.mapSelect.value) {
      socket.emit('load_map', { filename: els.mapSelect.value });
    }
  });

  els.btnFindPath.addEventListener('click', () => {
    socket.emit('find_path');
  });

  els.btnStart.addEventListener('click', () => {
    socket.emit('start_navigation');
  });

  els.btnStop.addEventListener('click', () => {
    socket.emit('stop_navigation');
  });

  els.btnEmergency.addEventListener('click', () => {
    socket.emit('emergency_stop');
    document.body.style.animation = 'none';
    document.body.offsetHeight; // reflow
    els.btnEmergency.style.animation = 'emergencyFlash 0.15s 3';
    setTimeout(() => { els.btnEmergency.style.animation = ''; }, 500);
  });

  els.btnClearLog.addEventListener('click', () => {
    els.logBody.innerHTML = '';
  });

  els.speedSlider.addEventListener('input', () => {
    manualSpeed = parseInt(els.speedSlider.value);
    els.speedVal.textContent = manualSpeed;
  });

  // D-Pad
  document.querySelectorAll('.dpad-btn').forEach(btn => {
    btn.addEventListener('mousedown', () => sendManual(btn.dataset.dir));
    btn.addEventListener('mouseup', () => sendManual('stop'));
    btn.addEventListener('mouseleave', () => sendManual('stop'));
    btn.addEventListener('touchstart', (e) => { e.preventDefault(); sendManual(btn.dataset.dir); });
    btn.addEventListener('touchend', (e) => { e.preventDefault(); sendManual('stop'); });
  });

  // File upload
  els.fileInput.addEventListener('change', (e) => {
    const file = e.target.files[0];
    if (!file) return;
    const formData = new FormData();
    formData.append('file', file);
    fetch('/api/map/upload', { method: 'POST', body: formData })
      .then(r => r.json())
      .then(data => {
        if (data.success) {
          loadMapList();
          setTimeout(() => { els.mapSelect.value = data.filename; els.btnLoadMap.disabled = false; }, 300);
        }
      });
    els.fileInput.value = '';
  });
}

// ─── Keyboard Control ───
function setupKeyboard() {
  const pressed = new Set();

  document.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    const key = e.key.toLowerCase();
    if (['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(key)) {
      e.preventDefault();
      if (!pressed.has(key)) {
        pressed.add(key);
        updateManualFromKeys(pressed);
      }
    }
    if (key === ' ') {
      e.preventDefault();
      socket.emit('emergency_stop');
    }
  });

  document.addEventListener('keyup', (e) => {
    const key = e.key.toLowerCase();
    if (pressed.has(key)) {
      pressed.delete(key);
      updateManualFromKeys(pressed);
    }
  });
}

function updateManualFromKeys(pressed) {
  let speed = 0, angle = 0;
  const s = manualSpeed;

  const fwd = pressed.has('w') || pressed.has('arrowup');
  const bwd = pressed.has('s') || pressed.has('arrowdown');
  const lft = pressed.has('a') || pressed.has('arrowleft');
  const rgt = pressed.has('d') || pressed.has('arrowright');

  if (fwd && !bwd) speed = s;
  if (bwd && !fwd) speed = -s;
  if (lft && !rgt) angle = -30;
  if (rgt && !lft) angle = 30;

  if (speed === 0 && angle === 0) {
    socket.emit('manual_stop');
  } else {
    socket.emit('manual_control', { speed, angle });
  }
}

function sendManual(dir) {
  const s = manualSpeed;
  const dirs = {
    forward:  { speed: s, angle: 0 },
    backward: { speed: -s, angle: 0 },
    left:     { speed: Math.round(s / 2), angle: -30 },
    right:    { speed: Math.round(s / 2), angle: 30 },
    stop:     null,
  };
  if (dir === 'stop') {
    socket.emit('manual_stop');
  } else if (dirs[dir]) {
    socket.emit('manual_control', dirs[dir]);
  }
}

// ─── Socket Events ───

socket.on('state', (data) => {
  currentState = data;
  updateUI(data);
});

socket.on('telemetry', (data) => {
  updateTelemetry(data);
});

socket.on('map_data', (data) => {
  mapData = data;
  drawMap(data);
});

socket.on('log', (entry) => {
  appendLog(entry);
});

socket.on('log_history', (entries) => {
  entries.forEach(appendLog);
});

socket.on('error', (data) => {
  appendLog({ time: new Date().toLocaleTimeString('uz'), msg: data.msg, level: 'error' });
});

socket.on('nav_progress', (data) => {
  els.progressText.textContent = `${data.current + 1} / ${data.total}`;
  const pct = ((data.current + 1) / data.total) * 100;
  els.progressFill.style.width = pct + '%';

  if (mapData) {
    drawMap(mapData, data.current);
  }
});

// ─── UI Update ───
function updateUI(s) {
  // Status chip
  const statusLabels = {
    idle: 'Kutish',
    navigating: 'Navigatsiya',
    manual: 'Qo\'lda',
    paused: 'To\'xtatilgan',
    error: 'Xatolik',
  };
  els.statusChip.setAttribute('data-status', s.status);
  els.statusText.textContent = statusLabels[s.status] || s.status;

  // Connection
  if (s.connected) {
    els.connBadge.classList.add('connected');
    els.connText.textContent = s.simulate ? 'Simulyatsiya' : 'Ulangan';
  } else {
    els.connBadge.classList.remove('connected');
    els.connText.textContent = 'Ulanmagan';
  }

  // Buttons
  els.btnFindPath.disabled = !s.map_loaded;
  els.btnStart.disabled = !s.path_found || s.status === 'navigating';
  els.btnStop.disabled = s.status !== 'navigating';

  // Progress
  if (s.status === 'navigating' && s.waypoints_total > 0) {
    els.progressSection.style.display = 'block';
    els.progressText.textContent = `${s.waypoints_done} / ${s.waypoints_total}`;
    els.progressFill.style.width = ((s.waypoints_done / s.waypoints_total) * 100) + '%';
  } else {
    els.progressSection.style.display = 'none';
  }
}

// ─── Telemetry Update ───
function updateTelemetry(t) {
  els.teleEncL.textContent = t.encoder_left;
  els.teleEncR.textContent = t.encoder_right;
  els.teleRpmL.textContent = t.rpm_left;
  els.teleRpmR.textContent = t.rpm_right;
  els.teleDist.textContent = `${t.distance} m`;

  // Compass
  els.compassNeedle.style.transform = `translate(-50%, -100%) rotate(${t.heading}deg)`;
  els.compassValue.textContent = `${t.heading}\u00B0`;

  // Distance bar
  const maxDist = 5.0;
  const pct = Math.min(100, (t.distance / maxDist) * 100);
  els.distBar.style.width = pct + '%';
  els.distBar.className = 'distance-bar';
  if (t.distance < 0.5) els.distBar.classList.add('danger');
  else if (t.distance < 1.0) els.distBar.classList.add('close');

  // Obstacle indicator
  const oi = els.obstacleIndicator;
  if (t.obstacle || t.distance < 0.5) {
    oi.classList.add('warning');
    oi.querySelector('.obstacle-text').textContent = `TO\'SIQ! ${t.distance} m`;
  } else {
    oi.classList.remove('warning');
    oi.querySelector('.obstacle-text').textContent = 'Yo\'l ochiq';
  }
}

// ─── Map Drawing ───
function drawMap(data, activeWaypoint) {
  if (!data || !data.grid || data.grid.length === 0) return;

  els.mapPlaceholder.style.display = 'none';
  const canvas = els.mapCanvas;
  const ctx = canvas.getContext('2d');

  const rows = data.rows;
  const cols = data.cols;

  const container = els.mapCanvas.parentElement;
  const maxW = container.clientWidth - 40;
  const maxH = container.clientHeight - 40;

  const cellSize = Math.min(Math.floor(maxW / cols), Math.floor(maxH / rows), 60);
  const padding = 30;

  canvas.width = cols * cellSize + padding * 2;
  canvas.height = rows * cellSize + padding * 2;

  ctx.fillStyle = '#1c1f2e';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  const ox = padding;
  const oy = padding;

  // Grid labels
  ctx.font = '10px monospace';
  ctx.fillStyle = '#5c5f73';
  ctx.textAlign = 'center';
  for (let c = 0; c < cols; c++) {
    ctx.fillText(c, ox + c * cellSize + cellSize / 2, oy - 10);
  }
  ctx.textAlign = 'right';
  for (let r = 0; r < rows; r++) {
    ctx.fillText(r, ox - 8, oy + r * cellSize + cellSize / 2 + 4);
  }

  // Draw cells
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      const x = ox + c * cellSize;
      const y = oy + r * cellSize;

      if (data.grid[r][c] === 1) {
        ctx.fillStyle = '#2a2d3e';
      } else {
        ctx.fillStyle = '#0d0f15';
      }
      ctx.fillRect(x + 1, y + 1, cellSize - 2, cellSize - 2);

      // Grid border
      ctx.strokeStyle = '#1a1d2a';
      ctx.lineWidth = 0.5;
      ctx.strokeRect(x, y, cellSize, cellSize);
    }
  }

  // Draw path
  if (data.path && data.path.length > 0) {
    // Path cells
    data.path.forEach((p, idx) => {
      const x = ox + p[1] * cellSize;
      const y = oy + p[0] * cellSize;

      let alpha = 0.4;
      if (activeWaypoint !== undefined) {
        // Highlight progress
        const wpIdx = getWaypointForPathIndex(data, idx);
        if (wpIdx < activeWaypoint) alpha = 0.15;
        else if (wpIdx === activeWaypoint) alpha = 0.8;
      }

      ctx.fillStyle = `rgba(108, 99, 255, ${alpha})`;
      ctx.fillRect(x + 1, y + 1, cellSize - 2, cellSize - 2);
    });

    // Path line
    ctx.beginPath();
    ctx.strokeStyle = 'rgba(108, 99, 255, 0.6)';
    ctx.lineWidth = 3;
    ctx.lineJoin = 'round';
    data.path.forEach((p, i) => {
      const x = ox + p[1] * cellSize + cellSize / 2;
      const y = oy + p[0] * cellSize + cellSize / 2;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();

    // Waypoint markers
    if (data.waypoints) {
      data.waypoints.forEach((wp, i) => {
        const to = wp.to;
        const x = ox + to[1] * cellSize + cellSize / 2;
        const y = oy + to[0] * cellSize + cellSize / 2;

        ctx.beginPath();
        ctx.arc(x, y, 6, 0, Math.PI * 2);
        ctx.fillStyle = (activeWaypoint !== undefined && i <= activeWaypoint) ? '#22c55e' : '#6c63ff';
        ctx.fill();
        ctx.strokeStyle = '#1c1f2e';
        ctx.lineWidth = 2;
        ctx.stroke();

        ctx.fillStyle = '#fff';
        ctx.font = 'bold 8px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText(i + 1, x, y + 3);
      });
    }
  }

  // Start marker
  drawMarker(ctx, ox, oy, data.start, cellSize, '#22c55e', 'S');

  // End marker
  drawMarker(ctx, ox, oy, data.end, cellSize, '#ef4444', 'E');
}

function drawMarker(ctx, ox, oy, pos, cellSize, color, label) {
  const x = ox + pos[1] * cellSize + cellSize / 2;
  const y = oy + pos[0] * cellSize + cellSize / 2;
  const r = Math.min(cellSize / 3, 14);

  ctx.beginPath();
  ctx.arc(x, y, r, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
  ctx.strokeStyle = '#1c1f2e';
  ctx.lineWidth = 3;
  ctx.stroke();

  ctx.fillStyle = '#fff';
  ctx.font = `bold ${Math.max(10, r)}px sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(label, x, y + 1);
}

function getWaypointForPathIndex(data, pathIdx) {
  if (!data.waypoints) return -1;
  let cumulative = 0;
  for (let i = 0; i < data.waypoints.length; i++) {
    cumulative += data.waypoints[i].steps;
    if (pathIdx <= cumulative) return i;
  }
  return data.waypoints.length - 1;
}

// ─── Log ───
function appendLog(entry) {
  const div = document.createElement('div');
  div.className = `log-entry ${entry.level || 'info'}`;
  div.innerHTML = `<span class="log-time">${entry.time}</span><span class="log-msg">${escapeHtml(entry.msg)}</span>`;
  els.logBody.appendChild(div);
  els.logBody.scrollTop = els.logBody.scrollHeight;

  // Keep max 200 entries in DOM
  while (els.logBody.children.length > 200) {
    els.logBody.removeChild(els.logBody.firstChild);
  }
}

function escapeHtml(str) {
  const div = document.createElement('div');
  div.textContent = str;
  return div.innerHTML;
}

// ─── Window resize → redraw map ───
window.addEventListener('resize', () => {
  if (mapData) drawMap(mapData);
});
