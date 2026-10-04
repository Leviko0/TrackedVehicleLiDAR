#pragma once

// The phone control page, embedded into the firmware.
// Left half: virtual joystick. Right half: four direction buttons.
// Talks to the vehicle over the WebSocket at /ws (protocol: CommandParser.h).

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
header{display:flex;align-items:center;gap:14px;padding:8px 16px;background:var(--panel);
border-bottom:1px solid var(--line);font-size:14px;flex-wrap:wrap}
.status{display:flex;align-items:center;gap:6px;min-width:110px}
.dot{width:10px;height:10px;border-radius:50%;background:var(--bad);flex:none}
.dot.on{background:var(--ok)}.dot.fs{background:var(--warn)}
.tracks{display:flex;gap:10px;color:var(--muted)}
.track{display:flex;align-items:center;gap:4px}
.bar{position:relative;width:44px;height:8px;border-radius:4px;background:var(--line);overflow:hidden}
.bar i{position:absolute;top:0;bottom:0;left:50%;width:0;background:var(--accent)}
.speed{display:flex;align-items:center;gap:8px;margin-left:auto;color:var(--muted)}
.speed input{width:min(28vw,170px);accent-color:var(--accent)}
.speed output{width:3.2em;text-align:right;color:var(--text)}
#stop{border:0;border-radius:8px;padding:8px 14px;font-weight:700;color:#fff;background:var(--bad)}
main{flex:1;min-height:0;display:flex;align-items:center;justify-content:space-around;gap:16px;padding:16px}
.zone{flex:1;height:100%;display:flex;align-items:center;justify-content:center}
#joy{position:relative;width:min(42vw,72vh);aspect-ratio:1;border-radius:50%;
background:radial-gradient(circle,#1e293b 0,#1e293b 55%,#172033 100%);border:2px solid var(--line)}
#joy::before,#joy::after{content:"";position:absolute;background:var(--line)}
#joy::before{left:50%;top:10%;bottom:10%;width:1px}
#joy::after{top:50%;left:10%;right:10%;height:1px}
#knob{position:absolute;left:30%;top:30%;width:40%;height:40%;border-radius:50%;z-index:1;
background:radial-gradient(circle at 35% 35%,#7dd3fc,var(--accent));box-shadow:0 4px 16px #0008}
#joy.active{border-color:var(--accent)}
.pad{display:grid;grid-template:repeat(3,1fr)/repeat(3,1fr);gap:10px;width:min(42vw,72vh);aspect-ratio:1}
.pad button{border:2px solid var(--line);border-radius:18px;background:var(--panel);color:var(--text);
font-size:clamp(22px,7vmin,48px);display:flex;align-items:center;justify-content:center;touch-action:none}
.pad button.active{background:var(--accent);border-color:var(--accent);color:var(--bg)}
#fwd{grid-area:1/2}#ccw{grid-area:2/1}#cw{grid-area:2/3}#rev{grid-area:3/2}
</style>
</head>
<body>
<header>
  <div class="status"><span id="dot" class="dot"></span><span id="state">Connecting…</span></div>
  <div class="tracks">
    <div class="track">L<span class="bar"><i id="barL"></i></span></div>
    <div class="track">R<span class="bar"><i id="barR"></i></span></div>
  </div>
  <label class="speed">Speed<input id="limit" type="range" min="10" max="100" step="5" value="80"><output id="limitOut">80%</output></label>
  <button id="stop">STOP</button>
</header>
<main>
  <div class="zone"><div id="joy"><div id="knob"></div></div></div>
  <div class="zone">
    <div class="pad">
      <button id="fwd" aria-label="Forward">&#9650;</button>
      <button id="ccw" aria-label="Turn counter-clockwise">&#8634;</button>
      <button id="cw" aria-label="Turn clockwise">&#8635;</button>
      <button id="rev" aria-label="Backward">&#9660;</button>
    </div>
  </div>
</main>
<script>
(() => {
  const $ = id => document.getElementById(id);
  const SEND_INTERVAL_MS = 50;
  // [throttle, turn]; turn > 0 = clockwise
  const BUTTONS = { fwd: [1, 0], rev: [-1, 0], ccw: [0, -1], cw: [0, 1] };

  const input = { jx: 0, jy: 0, button: null };
  let ws = null;
  let connected = false;
  let failsafe = false;

  // ---------- connection ----------
  function connect() {
    ws = new WebSocket(`ws://${location.host}/ws`);
    ws.onopen = () => { connected = true; render(); sendLimit(); };
    ws.onclose = () => { connected = false; render(); setTimeout(connect, 1000); };
    ws.onerror = () => ws.close();
    ws.onmessage = e => { try { telemetry(JSON.parse(e.data)); } catch (_) {} };
  }
  function send(msg) {
    if (ws && ws.readyState === WebSocket.OPEN && ws.bufferedAmount < 256) ws.send(msg);
  }
  function render() {
    $('dot').className = 'dot' + (connected ? (failsafe ? ' fs' : ' on') : '');
    $('state').textContent = connected ? (failsafe ? 'Idle (failsafe)' : 'Connected') : 'Disconnected';
  }
  function setBar(el, v) {
    const pct = Math.min(Math.abs(v), 1) * 50;
    el.style.width = pct + '%';
    el.style.left = (v >= 0 ? 50 : 50 - pct) + '%';
  }
  function telemetry(t) {
    failsafe = !!t.fs;
    setBar($('barL'), t.l || 0);
    setBar($('barR'), t.r || 0);
    render();
  }

  // ---------- command loop (also acts as heartbeat) ----------
  function currentCommand() {
    if (input.button) return BUTTONS[input.button];
    return [input.jy, input.jx];
  }
  setInterval(() => {
    const [t, r] = currentCommand();
    send(`D ${t.toFixed(2)} ${r.toFixed(2)}`);
  }, SEND_INTERVAL_MS);

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

  // ---------- buttons ----------
  const buttonPointers = new Map();  // pointerId -> button id
  function releaseButton(id) {
    $(id).classList.remove('active');
    if (input.button === id) input.button = null;
  }
  Object.keys(BUTTONS).forEach(id => {
    const el = $(id);
    el.addEventListener('pointerdown', e => {
      el.setPointerCapture(e.pointerId);
      buttonPointers.set(e.pointerId, id);
      el.classList.add('active');
      input.button = id;
    });
    ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(ev =>
      el.addEventListener(ev, e => {
        if (buttonPointers.get(e.pointerId) !== id) return;
        buttonPointers.delete(e.pointerId);
        releaseButton(id);
      }));
  });

  // ---------- speed limit & stop ----------
  const limit = $('limit');
  function sendLimit() {
    $('limitOut').textContent = limit.value + '%';
    send(`L ${(limit.value / 100).toFixed(2)}`);
  }
  limit.addEventListener('input', sendLimit);

  function stopAll() {
    releaseJoy();
    buttonPointers.clear();
    Object.keys(BUTTONS).forEach(releaseButton);
    send('S');
  }
  $('stop').addEventListener('pointerdown', stopAll);
  document.addEventListener('visibilitychange', () => { if (document.hidden) stopAll(); });
  window.addEventListener('blur', stopAll);
  document.addEventListener('contextmenu', e => e.preventDefault());

  render();
  connect();
})();
</script>
</body>
</html>
)rawliteral";

}  // namespace web
