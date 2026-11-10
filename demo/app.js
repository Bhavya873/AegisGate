var STATE_NAMES = ["INIT", "ARMED", "RUNNING", "TRIPPED"];
var STATE_BIG = ["Warming up", "Ready", "Running", "Guard stopped the motor"];
var FAULT_NAMES = ["NONE", "TRACKING", "VELOCITY", "HEARTBEAT", "DISAGREE", "STALL", "NUMERIC", "INTERNAL"];
var BREAK_LABELS = ["Lose connection", "Block the motor", "Faulty speed sensor", "Runaway motor", "Sudden command change"];
var BREAK_FAULT = [3, 5, 4, 2, 1];
var FAULT_TEXT = [
  "",
  "The arm was too far from where it should be.",
  "The arm was moving too fast.",
  "The controller stopped talking to the guard.",
  "The two speed sensors did not agree.",
  "The motor was pushing but the arm did not move.",
  "A sensor value was not a number.",
  "Internal error."
];
var HISTORY_SIZE = 2000;
var TICK_MS = 5;
var MAX_SPEED = 12;

var samples = [];
var logLines = [];
var lastState = -1;
var lastTick = 0;
var pendingFault = null;
var tripMs = -1;
var target = 0;
var latest = null;
var lastFrameTime = 0;
var leftoverMs = 0;

var statusBox = document.getElementById("status");
var dial = document.getElementById("dial").getContext("2d");
var chart = document.getElementById("chart").getContext("2d");

function start() {
  document.getElementById("btn-start").onclick = function () { Module._demo_send(1); };
  document.getElementById("btn-stop").onclick = function () { Module._demo_send(2); };
  document.getElementById("btn-reset").onclick = function () { Module._demo_send(3); };
  document.getElementById("btn-start-over").onclick = startOver;
  document.getElementById("target").oninput = moveSlider;

  for (var i = 0; i < 5; i++) {
    setupBreakButton(i);
  }

  Module._demo_reset();
  requestAnimationFrame(frame);
}

function setupBreakButton(n) {
  document.getElementById("break-" + n).onclick = function () {
    breakIt(n);
  };
}

function breakIt(n) {
  if (n === 4) {
    Module._demo_jump_ref();
  } else {
    Module._demo_set_fault(n, 1);
  }
  if (n === 1) {
    var newTarget = target + 0.6;
    if (newTarget > 1.5) {
      newTarget = target - 0.6;
    }
    document.getElementById("target").value = newTarget;
    moveSlider();
  }
  addLog(BREAK_LABELS[n]);
  pendingFault = { tick: lastTick, name: BREAK_LABELS[n] };
}

function moveSlider() {
  target = parseFloat(document.getElementById("target").value);
  Module._demo_set_target(target);
  document.getElementById("target-value").textContent = Math.round(target * 180 / Math.PI) + " deg";
}

function startOver() {
  Module._demo_reset();
  samples = [];
  logLines = [];
  latest = null;
  lastState = -1;
  lastTick = 0;
  pendingFault = null;
  tripMs = -1;
  document.getElementById("target").value = 0;
  moveSlider();
  document.getElementById("log").textContent = "";
}

function addLog(text) {
  var time = (lastTick * TICK_MS / 1000).toFixed(3);
  logLines.unshift(time + " s  " + text);
  if (logLines.length > 30) {
    logLines.pop();
  }
  document.getElementById("log").textContent = logLines.join("\n");
}

function frame(now) {
  // a background tab can pause frames for a long time, so only run a few ticks to catch up
  var ticks = 0;
  if (lastFrameTime > 0) {
    leftoverMs = leftoverMs + (now - lastFrameTime);
    ticks = Math.floor(leftoverMs / TICK_MS);
    leftoverMs = leftoverMs - ticks * TICK_MS;
    if (ticks > 20) {
      ticks = 20;
      leftoverMs = 0;
    }
  }
  lastFrameTime = now;

  for (var i = 0; i < ticks; i++) {
    Module._demo_tick();
  }
  while (Module._demo_pop()) {
    readSample();
  }
  if (samples.length > HISTORY_SIZE) {
    samples.splice(0, samples.length - HISTORY_SIZE);
  }

  showStatus();
  showButtons();
  showLights();
  drawDial();
  drawChart();
  requestAnimationFrame(frame);
}

function readSample() {
  var s = {
    tick: Module._demo_tick_count(),
    angle: Module._demo_angle(),
    ref: Module._demo_ref(),
    speed: Module._demo_speed(),
    state: Module._demo_state(),
    fault: Module._demo_fault()
  };
  samples.push(s);
  latest = s;
  lastTick = s.tick;

  if (lastState !== -1 && s.state !== lastState) {
    var text = STATE_NAMES[lastState] + " -> " + STATE_NAMES[s.state];
    if (s.state === 3) {
      text = text + " (" + FAULT_NAMES[s.fault] + ")";
      for (var i = 0; i < 4; i++) {
        Module._demo_set_fault(i, 0);
      }
    }
    addLog(text);
    if (s.state === 3 && pendingFault !== null) {
      var ticks = s.tick - pendingFault.tick;
      tripMs = ticks * TICK_MS;
      addLog("tripped " + ticks + " ticks (" + tripMs + " ms) after " + pendingFault.name);
    }
    if (s.state !== 3) {
      tripMs = -1;
    }
    pendingFault = null;
  }
  lastState = s.state;
}

function showStatus() {
  var classNames = ["state-init", "state-armed", "state-running", "state-tripped"];
  var s = latest;
  if (s === null) {
    return;
  }
  statusBox.className = classNames[s.state];
  document.getElementById("state-big").textContent = STATE_BIG[s.state];
  document.getElementById("state-tech").textContent = STATE_NAMES[s.state];

  var power = document.getElementById("power");
  if (s.state === 2) {
    power.textContent = "ON";
    power.className = "power-on";
  } else {
    power.textContent = "OFF";
    power.className = s.state === 3 ? "power-red" : "power-grey";
  }

  var hint = "";
  if (s.state === 1) {
    hint = "Press Start motor.";
  } else if (s.state === 2) {
    hint = "Move the arm, or break something below.";
  } else if (s.state === 3) {
    hint = FAULT_TEXT[s.fault] + " The guard cut the power";
    if (tripMs >= 0) {
      hint = hint + " in " + tripMs + " ms (simulated time)";
    }
    hint = hint + ". " + nextStep(s);
  }
  document.getElementById("hint").textContent = hint;
}

function nextStep(s) {
  if (Math.abs(s.angle) > 2) {
    return "Press Start over.";
  }
  if (Math.abs(s.speed) > MAX_SPEED) {
    return "Wait for the arm to slow down.";
  }
  return "Press Reset, then Start motor.";
}

function showButtons() {
  var s = latest;
  if (s === null) {
    return;
  }
  document.getElementById("btn-start").disabled = s.state !== 1;
  document.getElementById("btn-stop").disabled = s.state !== 2;
  document.getElementById("btn-reset").disabled = !(s.state === 3 && Math.abs(s.speed) <= MAX_SPEED);
  for (var i = 0; i < 5; i++) {
    document.getElementById("break-" + i).disabled = s.state !== 2;
  }
}

function showLights() {
  for (var i = 0; i < 5; i++) {
    var on = latest !== null && latest.state === 3 && latest.fault === BREAK_FAULT[i];
    document.getElementById("light-" + i).className = on ? "light on" : "light";
  }
}

function drawDial() {
  var cx = 110;
  var cy = 110;
  var r = 90;
  dial.clearRect(0, 0, 220, 220);
  dial.strokeStyle = "#444";
  dial.lineWidth = 2;
  dial.beginPath();
  dial.arc(cx, cy, r, 0, 2 * Math.PI);
  dial.stroke();
  if (latest === null) {
    return;
  }

  var t = latest.ref - Math.PI / 2;
  dial.strokeStyle = "#888";
  dial.lineWidth = 2;
  dial.setLineDash([4, 4]);
  dial.beginPath();
  dial.moveTo(cx + 60 * Math.cos(t), cy + 60 * Math.sin(t));
  dial.lineTo(cx + r * Math.cos(t), cy + r * Math.sin(t));
  dial.stroke();
  dial.setLineDash([]);

  var color = "#888";
  if (latest.state === 2) {
    color = "#26a269";
  } else if (latest.state === 3) {
    color = "#c01c28";
  }
  var a = latest.angle - Math.PI / 2;
  dial.strokeStyle = color;
  dial.fillStyle = color;
  dial.lineWidth = 5;
  dial.beginPath();
  dial.moveTo(cx, cy);
  dial.lineTo(cx + 80 * Math.cos(a), cy + 80 * Math.sin(a));
  dial.stroke();
  dial.beginPath();
  dial.arc(cx, cy, 7, 0, 2 * Math.PI);
  dial.fill();
}

function chartX(tick, endTick) {
  return 50 + 550 * (tick - (endTick - HISTORY_SIZE)) / HISTORY_SIZE;
}

function drawChart() {
  var left = 50;
  var width = 550;
  var top = 10;
  var height = 160;
  var endTick = Math.max(lastTick, HISTORY_SIZE);

  chart.clearRect(0, 0, 620, 200);
  chart.font = "12px Arial";
  chart.lineWidth = 1;

  chart.fillStyle = "#eee";
  var runStart = -1;
  for (var i = 0; i < samples.length; i++) {
    var x = chartX(samples[i].tick, endTick);
    if (samples[i].state === 3 && runStart < 0) {
      runStart = x;
    }
    if (samples[i].state !== 3 && runStart >= 0) {
      chart.fillRect(runStart, top, x - runStart, height);
      runStart = -1;
    }
  }
  if (runStart >= 0) {
    chart.fillRect(runStart, top, chartX(lastTick, endTick) - runStart, height);
  }

  chart.strokeStyle = "#aaa";
  chart.strokeRect(left, top, width, height);
  chart.beginPath();
  chart.moveTo(left, top + height / 2);
  chart.lineTo(left + width, top + height / 2);
  chart.stroke();

  chart.fillStyle = "#444";
  chart.fillText("2", 36, top + 10);
  chart.fillText("0", 36, top + height / 2 + 4);
  chart.fillText("-2", 30, top + height);
  chart.fillText("rad", 6, top + height / 2 + 4);
  chart.fillText("-10 s", left, 192);
  chart.fillText("now", left + width - 22, 192);
  chart.fillStyle = "#1a5fb4";
  chart.fillText("arm", 250, 192);
  chart.fillStyle = "#c01c28";
  chart.fillText("target", 300, 192);

  drawLine("ref", "#c01c28", top, height, endTick);
  drawLine("angle", "#1a5fb4", top, height, endTick);
}

function drawLine(field, color, top, height, endTick) {
  chart.strokeStyle = color;
  chart.lineWidth = 2;
  chart.beginPath();
  for (var i = 0; i < samples.length; i++) {
    var x = chartX(samples[i].tick, endTick);
    var value = Math.max(-2, Math.min(2, samples[i][field]));
    var y = top + height / 2 - value / 2 * height / 2;
    if (i === 0) {
      chart.moveTo(x, y);
    } else {
      chart.lineTo(x, y);
    }
  }
  chart.stroke();
  chart.lineWidth = 1;
}
