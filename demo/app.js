var TICK_MS = 5;
var TRAIL_SIZE = 120;
var CHECK_NAMES = ["", "TRACKING", "VELOCITY", "HEARTBEAT", "DISAGREE", "STALL", "NUMERIC", "INTERNAL"];
var CHECK_TEXT = [
  "",
  "the joint was too far from where it should be",
  "the joint was moving too fast",
  "the controller stopped talking",
  "the two sensors did not agree",
  "motor was pushing but the joint did not move",
  "internal check",
  "internal check"
];

var contexts = [];
var trails = [[], []];
var canvasSize = 0;
var link1 = 0;
var link2 = 0;
var lastFrameTime = 0;
var leftoverMs = 0;

function start() {
  contexts[0] = document.getElementById("canvas-0").getContext("2d");
  contexts[1] = document.getElementById("canvas-1").getContext("2d");
  resizeCanvases();
  window.onresize = resizeCanvases;
  document.getElementById("btn-break").onclick = breakIt;
  document.getElementById("btn-start-over").onclick = startOver;
  Module._demo_arm_reset();
  requestAnimationFrame(frame);
}

function resizeCanvases() {
  var panelWidth = document.getElementById("panel-0").clientWidth - 16;
  var size = panelWidth;
  if (window.innerWidth > 700) {
    size = Math.min(panelWidth, window.innerHeight - 430);
  }
  if (size < 200) {
    size = 200;
  }
  canvasSize = size;
  document.getElementById("canvas-0").width = size;
  document.getElementById("canvas-0").height = size;
  document.getElementById("canvas-1").width = size;
  document.getElementById("canvas-1").height = size;

  var scale = 0.9 * (canvasSize / 2) / (Module._demo_arm_link(0) + Module._demo_arm_link(1));
  link1 = Module._demo_arm_link(0) * scale;
  link2 = Module._demo_arm_link(1) * scale;
  trails = [[], []];
}

function breakIt() {
  Module._demo_arm_break(parseInt(document.getElementById("fault").value, 10));
}

function startOver() {
  Module._demo_arm_reset();
  trails = [[], []];
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
    Module._demo_arm_tick();
  }

  var fault = Module._demo_arm_fault();
  var tripped = Module._demo_arm_tripped() === 1;
  for (var side = 0; side < 2; side++) {
    var color = "#26a269";
    if (side === 0 && fault > 0) {
      color = "#e66100";
    } else if (side === 1 && tripped) {
      color = "#c01c28";
    }
    drawArm(side, color);
    showPower(side);
  }
  showStatus(fault, tripped);
  document.getElementById("btn-break").disabled = fault > 0;
  requestAnimationFrame(frame);
}

function jointPoints(side) {
  var a1 = Module._demo_arm_angle(side, 0);
  var a2 = a1 + Module._demo_arm_angle(side, 1);
  var baseX = canvasSize / 2;
  var baseY = canvasSize / 2;
  var elbowX = baseX + link1 * Math.sin(a1);
  var elbowY = baseY - link1 * Math.cos(a1);
  var tipX = elbowX + link2 * Math.sin(a2);
  var tipY = elbowY - link2 * Math.cos(a2);
  return { baseX: baseX, baseY: baseY, elbowX: elbowX, elbowY: elbowY, tipX: tipX, tipY: tipY };
}

function drawArm(side, color) {
  var c = contexts[side];
  var p = jointPoints(side);
  var trail = trails[side];
  trail.push([p.tipX, p.tipY]);
  if (trail.length > TRAIL_SIZE) {
    trail.shift();
  }

  c.clearRect(0, 0, canvasSize, canvasSize);
  c.strokeStyle = "#cccccc";
  c.lineWidth = 2;
  c.beginPath();
  for (var i = 0; i < trail.length; i++) {
    if (i === 0) {
      c.moveTo(trail[i][0], trail[i][1]);
    } else {
      c.lineTo(trail[i][0], trail[i][1]);
    }
  }
  c.stroke();

  c.strokeStyle = color;
  c.fillStyle = color;
  c.lineWidth = 8;
  c.lineCap = "round";
  c.beginPath();
  c.moveTo(p.baseX, p.baseY);
  c.lineTo(p.elbowX, p.elbowY);
  c.lineTo(p.tipX, p.tipY);
  c.stroke();
  drawDot(c, p.baseX, p.baseY, 10, "#444444");
  drawDot(c, p.elbowX, p.elbowY, 7, "#444444");
  drawDot(c, p.tipX, p.tipY, 6, "#444444");
}

function drawDot(c, x, y, radius, color) {
  c.fillStyle = color;
  c.beginPath();
  c.arc(x, y, radius, 0, 2 * Math.PI);
  c.fill();
}

function powerPercent(side) {
  var d0 = Math.abs(Module._demo_arm_duty(side, 0));
  var d1 = Math.abs(Module._demo_arm_duty(side, 1));
  return Math.round(Math.max(d0, d1) / 10);
}

function showPower(side) {
  var percent = powerPercent(side);
  document.getElementById("power-" + side).textContent = percent + "%";
  document.getElementById("bar-" + side).value = percent;
}

function showStatus(fault, tripped) {
  var left = "Working normally";
  var leftDetail = "";
  var right = "Working normally";
  var rightDetail = "";

  if (fault > 0) {
    left = fault === 1 ? "No gate: motor still pushing" : "No gate: still moving";
    leftDetail = "shoulder speed " + Module._demo_arm_speed(0, 0).toFixed(1) +
      " rad/s, power " + powerPercent(0) + "%";
    right = "Fault injected...";
  }
  if (tripped) {
    var ms = (Module._demo_arm_trip_tick() - Module._demo_arm_fault_tick()) * TICK_MS;
    var check = Module._demo_arm_trip_fault();
    right = "Gate stopped the arm in " + ms + " ms (simulated time)";
    rightDetail = CHECK_NAMES[check] + ": " + CHECK_TEXT[check];
  }

  document.getElementById("status-0").textContent = left;
  document.getElementById("detail-0").textContent = leftDetail;
  document.getElementById("status-1").textContent = right;
  document.getElementById("detail-1").textContent = rightDetail;
}
