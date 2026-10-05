#pragma once

// The phone control page, embedded into the firmware. Portrait layout:
//   top:    live LiDAR point map (vehicle in the centre, front = up)
//   bottom: virtual joystick
// Talks to the vehicle over the WebSocket at /ws:
//   text   -> commands (CommandParser.h), telemetry / config JSON
//   binary -> LiDAR scans: 'S', 0, 360 x uint16 LE distances in mm

namespace web {

static const char INDEX_HTML[] = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="theme-color" content="#0f172a">
<title>Tracked Vehicle</title>
<style>
:root{--bg:#0f172a;--panel:#1e293b;--line:#334155;--text:#f8fafc;--muted:#94a3b8;
--accent:#38bdf8;--ok:#10b981;--warn:#f59e0b;--bad:#ef4444}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;height:100%;background:var(--bg);color:var(--text);
font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;overflow:hidden;
overscroll-behavior:none;user-select:none;-webkit-user-select:none;
-webkit-touch-callout:none;touch-action:none}
body{display:flex;flex-direction:column;
padding:env(safe-area-inset-top) env(safe-area-inset-right) env(safe-area-inset-bottom) env(safe-area-inset-left)}
header{display:flex;flex-direction:column;gap:8px;padding:8px 16px;background:var(--panel);
border-bottom:1px solid var(--line);font-size:14px}
.row{display:flex;align-items:center;gap:12px}
.status{display:flex;align-items:center;gap:6px;flex:1;min-width:0;white-space:nowrap}
.dot{width:10px;height:10px;border-radius:50%;background:var(--bad);flex:none}
.dot.on{background:var(--ok)}.dot.fs{background:var(--warn)}
.tracks{display:flex;gap:10px;color:var(--muted)}
.track{display:flex;align-items:center;gap:4px}
.bar{position:relative;width:40px;height:8px;border-radius:4px;background:var(--line);overflow:hidden}
.bar i{position:absolute;top:0;bottom:0;left:50%;width:0;background:var(--accent)}
.speed{display:flex;align-items:center;gap:8px;flex:1;color:var(--muted);min-width:0}
.speed input{flex:1;min-width:0;accent-color:var(--accent)}
.speed output{width:3.2em;text-align:right;color:var(--text)}
button{font:inherit}
.chip{border:1px solid var(--line);border-radius:8px;padding:7px 10px;background:transparent;
color:var(--muted);font-weight:600}
.chip.on{border-color:var(--ok);color:var(--ok)}
#stop{border:0;border-radius:8px;padding:8px 14px;font-weight:700;color:#fff;background:var(--bad)}
#mapBox{position:relative;flex:1;min-height:0;margin:12px 16px 0;border:1px solid var(--line);
border-radius:16px;background:#0b1222;overflow:hidden}
#map{position:absolute;inset:0;width:100%;height:100%}
.overlay{position:absolute;display:flex;gap:6px;font-size:13px;color:var(--muted);pointer-events:none}
#zoomBox{top:8px;right:8px;pointer-events:auto}
#zoomBox button{width:36px;height:36px;border:1px solid var(--line);border-radius:10px;
background:var(--panel);color:var(--text);font-size:18px;font-weight:700}
#rangeLabel{top:14px;left:12px}
#lidarInfo{bottom:10px;left:12px}
#clearance{bottom:10px;right:12px;font-variant-numeric:tabular-nums}
#offline{inset:0;align-items:center;justify-content:center;font-size:16px;color:var(--warn);display:none}
#joyBox{flex:none;display:flex;align-items:center;justify-content:center;padding:16px;
height:min(38vh,calc(100vw - 32px))}
#joy{position:relative;height:100%;aspect-ratio:1;max-width:100%;border-radius:50%;
background:radial-gradient(circle,#1e293b 0,#1e293b 55%,#172033 100%);border:2px solid var(--line)}
#joy::before,#joy::after{content:"";position:absolute;background:var(--line)}
#joy::before{left:50%;top:10%;bottom:10%;width:1px}
#joy::after{top:50%;left:10%;right:10%;height:1px}
#knob{position:absolute;left:30%;top:30%;width:40%;height:40%;border-radius:50%;z-index:1;
background:radial-gradient(circle at 35% 35%,#7dd3fc,var(--accent));box-shadow:0 4px 16px #0008}
#joy.active{border-color:var(--accent)}
</style>
</head>
<body>
<header>
  <div class="row">
    <div class="status"><span id="dot" class="dot"></span><span id="state">Connecting…</span></div>
    <div class="tracks">
      <div class="track">L<span class="bar"><i id="barL"></i></span></div>
      <div class="track">R<span class="bar"><i id="barR"></i></span></div>
    </div>
  </div>
  <div class="row">
    <label class="speed">Speed<input id="limit" type="range" min="10" max="100" step="5" value="80"><output id="limitOut">80%</output></label>
    <button id="guard" class="chip">Guard</button>
    <button id="stop">STOP</button>
  </div>
</header>
<div id="mapBox">
  <canvas id="map"></canvas>
  <div class="overlay" id="rangeLabel"></div>
  <div class="overlay" id="zoomBox"><button id="zoomIn" aria-label="Zoom in">+</button><button id="zoomOut" aria-label="Zoom out">&minus;</button></div>
  <div class="overlay" id="lidarInfo"></div>
  <div class="overlay" id="clearance"></div>
  <div class="overlay" id="offline">LiDAR offline</div>
</div>
<div id="joyBox"><div id="joy"><div id="knob"></div></div></div>
<script>
(() => {
  const $ = id => document.getElementById(id);
  const SEND_INTERVAL_MS = 50;
  const RANGES_M = [0.5, 1, 2, 4, 8, 12];

  const input = { jx: 0, jy: 0 };
  const cfg = { w: 185, len: 220, stop: 150, slow: 600, margin: 30 };
  const tele = { li: false, hz: 0, gd: false, cf: -1, cr: -1 };
  let scan = null;  // Uint16Array(360), vehicle frame, 0° = front, clockwise
  let ws = null;
  let connected = false;
  let failsafe = false;
  let rangeIndex = 2;
  try { const s = parseInt(localStorage.getItem('range'), 10); if (s >= 0 && s < RANGES_M.length) rangeIndex = s; } catch (_) {}

  // ---------- connection ----------
  function connect() {
    ws = new WebSocket(`ws://${location.host}/ws`);
    ws.binaryType = 'arraybuffer';
    ws.onopen = () => { connected = true; render(); sendLimit(); };
    ws.onclose = () => { connected = false; scan = null; render(); drawMap(); setTimeout(connect, 1000); };
    ws.onerror = () => ws.close();
    ws.onmessage = e => {
      if (typeof e.data === 'string') {
        try {
          const m = JSON.parse(e.data);
          if (m.cfg) { Object.assign(cfg, m.cfg); tele.gd = !!m.cfg.guard; render(); drawMap(); }
          else telemetry(m);
        } catch (_) {}
      } else {
        const v = new DataView(e.data);
        if (v.byteLength === 722 && v.getUint8(0) === 83) {
          const d = new Uint16Array(360);
          for (let i = 0; i < 360; i++) d[i] = v.getUint16(2 + 2 * i, true);
          scan = d;
          drawMap();
        }
      }
    };
  }
  function send(msg) {
    if (ws && ws.readyState === WebSocket.OPEN && ws.bufferedAmount < 256) ws.send(msg);
  }
  function render() {
    $('dot').className = 'dot' + (connected ? (failsafe ? ' fs' : ' on') : '');
    $('state').textContent = connected ? (failsafe ? 'Idle (failsafe)' : 'Connected') : 'Disconnected';
    $('guard').className = 'chip' + (tele.gd ? ' on' : '');
    $('guard').textContent = tele.gd ? 'Guard on' : 'Guard off';
    $('offline').style.display = connected && !tele.li ? 'flex' : 'none';
    $('lidarInfo').textContent = tele.li ? `LiDAR ${tele.hz.toFixed(1)} Hz` : '';
    const fmt = mm => mm < 0 ? '–' : mm >= 12000 ? 'free' : (mm / 1000).toFixed(2) + ' m';
    $('clearance').textContent = tele.li ? `▲ ${fmt(tele.cf)}   ▼ ${fmt(tele.cr)}` : '';
  }
  function setBar(el, v) {
    const pct = Math.min(Math.abs(v), 1) * 50;
    el.style.width = pct + '%';
    el.style.left = (v >= 0 ? 50 : 50 - pct) + '%';
  }
  function telemetry(t) {
    failsafe = !!t.fs;
    const guardChanged = tele.gd !== !!t.gd;
    Object.assign(tele, { li: !!t.li, hz: t.hz || 0, gd: !!t.gd, cf: t.cf, cr: t.cr });
    if (!tele.li && scan) scan = null;
    if (guardChanged || !scan) drawMap();
    setBar($('barL'), t.l || 0);
    setBar($('barR'), t.r || 0);
    render();
  }

  // ---------- point map ----------
  const canvas = $('map'), ctx = canvas.getContext('2d');
  const css = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  function drawMap() {
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const rangeMm = RANGES_M[rangeIndex] * 1000;
    const cx = w / 2, cy = h / 2;
    const s = (Math.min(w, h) / 2 - 8) / rangeMm;  // px per mm
    $('rangeLabel').textContent = `Ring = ${(RANGES_M[rangeIndex] / 4 * 100).toFixed(0)} cm`;

    // range rings and axes
    ctx.strokeStyle = css('--line'); ctx.lineWidth = 1;
    for (let r = 1; r <= 4; r++) {
      ctx.beginPath(); ctx.arc(cx, cy, rangeMm * r / 4 * s, 0, Math.PI * 2); ctx.stroke();
    }
    ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, h); ctx.moveTo(0, cy); ctx.lineTo(w, cy); ctx.stroke();

    // guard corridors ahead and behind
    const halfW = cfg.w / 2, halfL = cfg.len / 2, corridor = halfW + cfg.margin;
    if (tele.gd) {
      ctx.fillStyle = 'rgba(245,158,11,0.10)';
      ctx.fillRect(cx - corridor * s, cy - (halfL + cfg.slow) * s, 2 * corridor * s, cfg.slow * s);
      ctx.fillRect(cx - corridor * s, cy + halfL * s, 2 * corridor * s, cfg.slow * s);
      ctx.fillStyle = 'rgba(239,68,68,0.18)';
      ctx.fillRect(cx - corridor * s, cy - (halfL + cfg.stop) * s, 2 * corridor * s, cfg.stop * s);
      ctx.fillRect(cx - corridor * s, cy + halfL * s, 2 * corridor * s, cfg.stop * s);
    }

    // LiDAR points
    if (scan) {
      const size = Math.max(2, Math.min(4, 3 * (window.devicePixelRatio || 1) / 2));
      for (let i = 0; i < 360; i++) {
        const d = scan[i];
        if (!d || d > rangeMm * 1.5) continue;
        const a = (i + 0.5) * Math.PI / 180;
        const x = d * Math.sin(a), y = d * Math.cos(a);
        const inCorridor = Math.abs(x) <= corridor && Math.abs(y) > halfL;
        const gap = Math.abs(y) - halfL;
        ctx.fillStyle = inCorridor && gap <= cfg.stop ? css('--bad')
                      : inCorridor && gap <= cfg.slow ? css('--warn') : css('--accent');
        ctx.fillRect(cx + x * s - size / 2, cy - y * s - size / 2, size, size);
      }
    }

    // vehicle (front = up)
    const vw = Math.max(cfg.w * s, 10), vl = Math.max(cfg.len * s, 12);
    ctx.fillStyle = css('--panel'); ctx.strokeStyle = css('--text'); ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.rect(cx - vw / 2, cy - vl / 2, vw, vl); ctx.fill(); ctx.stroke();
    ctx.fillStyle = css('--accent');
    ctx.beginPath();
    ctx.moveTo(cx, cy - vl / 2 + 2);
    ctx.lineTo(cx - vw / 3, cy - vl / 2 + Math.min(vl / 2, vw / 2));
    ctx.lineTo(cx + vw / 3, cy - vl / 2 + Math.min(vl / 2, vw / 2));
    ctx.closePath(); ctx.fill();
  }
  function zoom(delta) {
    rangeIndex = Math.max(0, Math.min(RANGES_M.length - 1, rangeIndex + delta));
    try { localStorage.setItem('range', String(rangeIndex)); } catch (_) {}
    drawMap();
  }
  $('zoomIn').addEventListener('click', () => zoom(-1));
  $('zoomOut').addEventListener('click', () => zoom(1));
  window.addEventListener('resize', drawMap);

  // ---------- command loop (also acts as heartbeat) ----------
  setInterval(() => send(`D ${input.jy.toFixed(2)} ${input.jx.toFixed(2)}`), SEND_INTERVAL_MS);

  // ---------- joystick ----------
  const joy = $('joy'), knob = $('knob');
  let joyPointer = null;
  function moveJoy(e) {
    const rect = joy.getBoundingClientRect();
    const radius = rect.width / 2;
    const travel = radius * 0.6;  // knob is 40 % of the base
    let dx = e.clientX - (rect.left + radius);
    let dy = e.clientY - (rect.top + radius);
    const dist = Math.hypot(dx, dy);
    if (dist > travel) { dx *= travel / dist; dy *= travel / dist; }
    knob.style.transform = `translate(${dx}px,${dy}px)`;
    input.jx = dx / travel;
    input.jy = -dy / travel;
  }
  function releaseJoy() {
    joyPointer = null;
    input.jx = input.jy = 0;
    knob.style.transform = '';
    joy.classList.remove('active');
  }
  joy.addEventListener('pointerdown', e => {
    if (joyPointer !== null) return;
    joyPointer = e.pointerId;
    joy.setPointerCapture(e.pointerId);
    joy.classList.add('active');
    moveJoy(e);
  });
  joy.addEventListener('pointermove', e => { if (e.pointerId === joyPointer) moveJoy(e); });
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(ev =>
    joy.addEventListener(ev, e => { if (e.pointerId === joyPointer) releaseJoy(); }));

  // ---------- speed limit, guard & stop ----------
  const limit = $('limit');
  function sendLimit() {
    $('limitOut').textContent = limit.value + '%';
    send(`L ${(limit.value / 100).toFixed(2)}`);
  }
  limit.addEventListener('input', sendLimit);
  $('guard').addEventListener('click', () => send(tele.gd ? 'G 0' : 'G 1'));

  function stopAll() { releaseJoy(); send('S'); }
  $('stop').addEventListener('pointerdown', stopAll);
  document.addEventListener('visibilitychange', () => { if (document.hidden) stopAll(); });
  window.addEventListener('blur', stopAll);
  document.addEventListener('contextmenu', e => e.preventDefault());

  render();
  drawMap();
  connect();
})();
</script>
</body>
</html>
)rawliteral";

}  // namespace web
