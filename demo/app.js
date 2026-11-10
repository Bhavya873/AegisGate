var STATE_NAMES = ["INIT", "ARMED", "RUNNING", "TRIPPED"];
var FAULT_NAMES = ["NONE", "TRACKING", "VELOCITY", "HEARTBEAT", "DISAGREE", "STALL", "NUMERIC", "INTERNAL"];
var FAULT_LABELS = ["Cut heartbeat", "Jam shaft", "Bad gyro", "Spin too fast"];
var FAULT_TEXT = [
  "",
  "The shaft was too far from the target.",
  "The shaft was spinning too fast.",
  "The heartbeat from the host stopped.",
  "The gyro and the encoder did not agree.",
  "The motor was pushing but the shaft did not move.",
  "A sensor value was not a number.",
  "Internal error."
];
var HISTORY_SIZE = 2000;
var TICK_MS = 5;

var samples = [];
var logLines = [];
var faultOn = [false, false, false, false];
var lastState = -1;
var lastTick = 0;
var pendingFault = null;
var target = 0;
var latest = null;
var lastFrameTime = 0;
var leftoverMs = 0;

var statusBox = document.getElementById("status");
var dial = document.getElementById("dial").getContext("2d");
var chart = document.getElementById("chart").getContext("2d");

function start() {
  document.getElementById("btn-arm").onclick = function () { Module._demo_send(1); };
  document.getElementById("btn-stop").onclick = function () { Module._demo_send(2); };
  document.getElementById("btn-reset").onclick = function () { Module._demo_send(3); };
  document.getElementById("btn-start-over").onclick = startOver;
  document.getElementById("btn-jump").onclick = jumpReference;
  document.getElementById("target").oninput = moveSlider;

  for (var i = 0; i < 4; i++) {
    setupFaultButton(i);
  }

  Module._demo_reset();
  requestAnimationFrame(frame);
}

function setupFaultButton(n) {
  document.getElementById("fault-" + n).onclick = function () {
    toggleFault(n);
  };
}

function toggleFault(n) {
  faultOn[n] = !faultOn[n];
  Module._demo_set_fault(n, faultOn[n] ? 1 : 0);
  showFaultButton(n);
  if (faultOn[n]) {
    addLog(FAULT_LABELS[n] + " turned ON");
    pendingFault = { tick: lastTick, name: FAULT_LABELS[n] };
  } else {
    addLog(FAULT_LABELS[n] + " turned off");
    pendingFault = null;
  }
}

function showFaultButton(n) {
  var button = document.getElementById("fault-" + n);
  if (faultOn[n]) {
    button.className = "on";
    button.textContent = FAULT_LABELS[n] + " (ON)";
  } else {
    button.className = "";
    button.textContent = FAULT_LABELS[n];
  }
}

function jumpReference() {
  Module._demo_jump_ref();
  addLog("Jump target");
  pendingFault = { tick: lastTick, name: "Jump target" };
}

function moveSlider() {
  target = parseFloat(document.getElementById("target").value);
  Module._demo_set_target(target);
  document.getElementById("target-value").textContent = target.toFixed(2);
}

function startOver() {
  Module._demo_reset();
  samples = [];
  logLines = [];
  latest = null;
  lastState = -1;
  lastTick = 0;
  pendingFault = null;
  document.getElementById("target").value = 0;
  moveSlider();
  for (var i = 0; i < 4; i++) {
    faultOn[i] = false;
    showFaultButton(i);
  }
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
  drawDial();
  drawChart();
  requestAnimationFrame(frame);
}

function readSample() {
  var s = {
    tick: Module._demo_tick_count(),
    angle: Module._demo_angle(),
    ref: Module._demo_ref(),
    duty: Module._demo_duty(),
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
    }
    addLog(text);
    if (s.state === 3 && pendingFault !== null) {
      var ticks = s.tick - pendingFault.tick;
      addLog("tripped " + ticks + " ticks (" + ticks * TICK_MS + " ms) after " + pendingFault.name);
      pendingFault = null;
    }
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
  document.getElementById("state-name").textContent = STATE_NAMES[s.state];
  if (s.state === 3) {
    document.getElementById("fault-text").textContent = FAULT_TEXT[s.fault] + " Motor is off. Press Reset, then Arm.";
  } else {
    document.getElementById("fault-text").textContent = "";
  }
  document.getElementById("r-time").textContent = (s.tick * TICK_MS / 1000).toFixed(2);
  document.getElementById("r-angle").textContent = s.angle.toFixed(2);
  document.getElementById("r-duty").textContent = Math.round(s.duty / 10);
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
  dial.fillStyle = "#444";
  dial.font = "12px Arial";
  dial.fillText("0", cx - 3, cy - r - 4);
  if (latest === null) {
    return;
  }
  drawNeedle(latest.ref, "#888", true);
  drawNeedle(latest.angle, "#1a5fb4", false);
}

function drawNeedle(angle, color, dashed) {
  var a = angle - Math.PI / 2;
  dial.strokeStyle = color;
  dial.lineWidth = 3;
  dial.setLineDash(dashed ? [6, 4] : []);
  dial.beginPath();
  dial.moveTo(110, 110);
  dial.lineTo(110 + 85 * Math.cos(a), 110 + 85 * Math.sin(a));
  dial.stroke();
  dial.setLineDash([]);
}

function drawChart() {
  var left = 50;
  var width = 790;
  var angleTop = 10;
  var angleHeight = 150;
  var dutyTop = 190;
  var dutyHeight = 50;
  var endTick = Math.max(lastTick, HISTORY_SIZE);

  chart.clearRect(0, 0, 860, 260);
  chart.font = "12px Arial";
  chart.lineWidth = 1;
  drawAxes(left, width, angleTop, angleHeight);
  drawAxes(left, width, dutyTop, dutyHeight);

  chart.fillStyle = "#444";
  chart.fillText("2", 30, angleTop + 10);
  chart.fillText("0", 30, angleTop + angleHeight / 2 + 4);
  chart.fillText("-2", 24, angleTop + angleHeight);
  chart.fillText("rad", 4, angleTop + angleHeight / 2 + 4);
  chart.fillText("1000", 14, dutyTop + 10);
  chart.fillText("-1000", 8, dutyTop + dutyHeight);
  chart.fillText("duty", 14, dutyTop + dutyHeight / 2 + 4);
  chart.fillText("-10 s", left, 256);
  chart.fillText("now", left + width - 22, 256);
  chart.fillStyle = "#1a5fb4";
  chart.fillText("blue = angle", 330, 256);
  chart.fillStyle = "#c01c28";
  chart.fillText("red = target", 420, 256);
  chart.fillStyle = "#26a269";
  chart.fillText("green = duty", 520, 256);

  drawLine("ref", "#c01c28", left, width, angleTop, angleHeight, 2, endTick);
  drawLine("angle", "#1a5fb4", left, width, angleTop, angleHeight, 2, endTick);
  drawLine("duty", "#26a269", left, width, dutyTop, dutyHeight, 1000, endTick);
}

function drawAxes(left, width, top, height) {
  chart.strokeStyle = "#aaa";
  chart.strokeRect(left, top, width, height);
  chart.beginPath();
  chart.moveTo(left, top + height / 2);
  chart.lineTo(left + width, top + height / 2);
  chart.stroke();
}

function drawLine(field, color, left, width, top, height, range, endTick) {
  chart.strokeStyle = color;
  chart.lineWidth = 2;
  chart.beginPath();
  for (var i = 0; i < samples.length; i++) {
    var x = left + width * (samples[i].tick - (endTick - HISTORY_SIZE)) / HISTORY_SIZE;
    var value = Math.max(-range, Math.min(range, samples[i][field]));
    var y = top + height / 2 - value / range * height / 2;
    if (i === 0) {
      chart.moveTo(x, y);
    } else {
      chart.lineTo(x, y);
    }
  }
  chart.stroke();
  chart.lineWidth = 1;
}
