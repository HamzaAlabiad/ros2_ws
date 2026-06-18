#!/usr/bin/env python3
"""Headless web dashboard for Audix control and monitoring."""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

import cv2
import rclpy
from audix_interfaces.msg import EspTelemetry, IrState
from audix_interfaces.srv import (
    AuditMission,
    DirectionCommand,
    LiftMoveSteps,
    RotateCommand,
    SetRobotMode,
    ShelfScan,
    TwistCommand,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


INDEX_HTML = r"""<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>Audix Control</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #0b0d10;
      --panel: #151b22;
      --line: #2e3a46;
      --text: #f2f5f8;
      --muted: #9aa8b4;
      --accent: #38c6a3;
      --warn: #efb75c;
      --danger: #e05757;
      --blue: #73a9ff;
    }
    * { box-sizing: border-box; letter-spacing: 0; }
    body { margin: 0; background: var(--bg); color: var(--text); font-family: Segoe UI, Arial, sans-serif; }
    main { max-width: 1220px; margin: 0 auto; padding: 18px; }
    header { display: flex; align-items: center; justify-content: space-between; gap: 12px; margin-bottom: 14px; }
    h1 { margin: 0; font-size: 26px; font-weight: 650; }
    h2 { margin: 0 0 10px; font-size: 16px; font-weight: 650; }
    .grid { display: grid; grid-template-columns: repeat(12, 1fr); gap: 12px; }
    section { grid-column: span 6; border-top: 1px solid var(--line); padding-top: 12px; }
    .wide { grid-column: span 12; }
    .panel { background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 12px; }
    .metrics { display: grid; grid-template-columns: repeat(4, minmax(0,1fr)); gap: 8px; }
    .metric { border: 1px solid var(--line); border-radius: 6px; padding: 8px; min-height: 56px; }
    .metric span { display: block; color: var(--muted); font-size: 12px; }
    .metric strong { display: block; font-size: 20px; margin-top: 4px; overflow-wrap: anywhere; }
    .row { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; margin-bottom: 8px; }
    button, input, select {
      border: 1px solid var(--line); border-radius: 6px; background: #11171e; color: var(--text);
      padding: 9px 10px; font: inherit;
    }
    button { cursor: pointer; min-width: 72px; }
    button:hover { border-color: var(--accent); }
    .danger { background: var(--danger); border-color: var(--danger); color: white; font-weight: 700; }
    .accent { background: #123b34; border-color: var(--accent); }
    .warn { background: #3f301b; border-color: var(--warn); }
    input[type=number] { width: 110px; }
    .pad { display: grid; grid-template-columns: repeat(3, 88px); grid-auto-rows: 48px; gap: 8px; }
    .pad button { min-width: 0; }
    .manual-grid { display: grid; grid-template-columns: minmax(260px, 1fr) 180px; gap: 14px; align-items: start; }
    .joystick-wrap { display: grid; gap: 8px; justify-items: center; }
    .joystick {
      position: relative; width: 160px; height: 160px; border-radius: 50%;
      border: 1px solid #35506a; background:
        radial-gradient(circle at center, rgba(115,169,255,0.16), rgba(17,23,30,0.25) 42%, rgba(7,9,12,0.92) 72%),
        linear-gradient(135deg, rgba(255,255,255,0.08), rgba(255,255,255,0.01));
      touch-action: none; user-select: none; box-shadow: inset 0 0 0 1px rgba(255,255,255,0.04);
    }
    .joystick::before, .joystick::after {
      content: ""; position: absolute; background: rgba(115,169,255,0.18); border-radius: 999px;
    }
    .joystick::before { left: 18px; right: 18px; top: 50%; height: 1px; }
    .joystick::after { top: 18px; bottom: 18px; left: 50%; width: 1px; }
    .stick {
      position: absolute; left: 50%; top: 50%; width: 54px; height: 54px; border-radius: 50%;
      transform: translate(-50%, -50%); border: 1px solid #73a9ff;
      background: radial-gradient(circle at 35% 28%, #d8e7ff, #73a9ff 42%, #1f4c86 100%);
      box-shadow: 0 10px 28px rgba(31,76,134,0.45);
      pointer-events: none;
    }
    .joystick-state { min-height: 20px; color: var(--muted); font-family: Consolas, monospace; }
    .joystick-state strong { color: var(--blue); }
    .log { min-height: 110px; max-height: 220px; overflow: auto; color: var(--muted); font-family: Consolas, monospace; white-space: pre-wrap; }
    .ir { display: grid; grid-template-columns: repeat(6, minmax(0,1fr)); gap: 8px; }
    .pill { border: 1px solid var(--line); border-radius: 6px; padding: 8px; text-align: center; color: var(--muted); }
    .pill.on { color: white; background: #4a1c22; border-color: var(--danger); }
    .audit-rows { display: grid; gap: 8px; }
    .audit-row {
      display: grid; grid-template-columns: minmax(120px, 1fr) minmax(190px, 1.3fr) 120px;
      gap: 8px; align-items: center; border: 1px solid var(--line); border-radius: 8px; padding: 10px;
      background: #111820;
    }
    .audit-row label { display: flex; align-items: center; gap: 8px; color: var(--text); font-size: 14px; }
    .audit-row select, .audit-row input { width: 100%; }
    .vision { display: grid; grid-template-columns: repeat(3, minmax(0,1fr)); gap: 8px; margin-top: 10px; }
    .camera-feed { width: 100%; max-height: 320px; object-fit: contain; border: 1px solid var(--line); border-radius: 6px; background: #07090c; margin-bottom: 10px; }
    .scan-image { display: none; width: 100%; max-height: 260px; object-fit: contain; border: 1px solid var(--line); border-radius: 6px; margin-top: 10px; background: #07090c; }
    .scan-results { display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); gap: 10px; }
    .scan-card { border: 1px solid var(--line); border-radius: 8px; background: #10161d; overflow: hidden; }
    .scan-card img { width: 100%; height: 150px; object-fit: contain; background: #07090c; border-bottom: 1px solid var(--line); display: block; cursor: zoom-in; }
    .scan-card-body { padding: 10px; }
    .scan-card-title { display: flex; justify-content: space-between; gap: 8px; font-weight: 700; margin-bottom: 6px; }
    .scan-card-meta { color: var(--muted); font-size: 13px; line-height: 1.45; overflow-wrap: anywhere; }
    .empty { color: var(--muted); border: 1px dashed var(--line); border-radius: 8px; padding: 14px; text-align: center; }
    .section-head { display: flex; justify-content: space-between; align-items: center; gap: 10px; margin-bottom: 10px; }
    .section-head h2 { margin: 0; }
    .count-badge { color: var(--muted); border: 1px solid var(--line); border-radius: 999px; padding: 4px 8px; font-size: 12px; }
    .image-modal { position: fixed; inset: 0; display: none; align-items: center; justify-content: center; padding: 24px; background: rgba(0,0,0,0.78); z-index: 20; }
    .image-modal.open { display: flex; }
    .image-modal-inner { width: min(980px, 100%); max-height: 92vh; border: 1px solid var(--line); border-radius: 10px; background: #090d12; overflow: hidden; }
    .image-modal-bar { display: flex; justify-content: space-between; align-items: center; gap: 10px; padding: 10px 12px; border-bottom: 1px solid var(--line); color: var(--muted); }
    .image-modal img { width: 100%; max-height: 78vh; object-fit: contain; display: block; background: #05070a; }
    label { color: var(--muted); font-size: 13px; }
    @media (max-width: 860px) { section { grid-column: span 12; } .metrics { grid-template-columns: repeat(2, minmax(0,1fr)); } .manual-grid, .audit-row, .vision { grid-template-columns: repeat(1, minmax(0,1fr)); } }
  </style>
</head>
<body>
<main>
  <header>
    <div>
      <h1>Audix Control</h1>
      <div id="event" style="color:var(--muted)">connecting...</div>
    </div>
    <button class="danger" onclick="stopRobot()">STOP</button>
  </header>

  <div class="grid">
    <section class="wide">
      <div class="metrics">
        <div class="metric"><span>Mode</span><strong id="mode">-</strong></div>
        <div class="metric"><span>Telemetry age</span><strong id="age">-</strong></div>
        <div class="metric"><span>Forward cm</span><strong id="forward">0</strong></div>
        <div class="metric"><span>Strafe cm</span><strong id="strafe">0</strong></div>
        <div class="metric"><span>Yaw deg</span><strong id="yaw">0</strong></div>
        <div class="metric"><span>IMU</span><strong id="imu">-</strong></div>
        <div class="metric"><span>Move</span><strong id="move">-</strong></div>
        <div class="metric"><span>Last seq</span><strong id="seq">-</strong></div>
      </div>
    </section>

    <section>
      <h2>Manual Jog</h2>
      <div class="panel">
        <div class="row">
          <label>Distance cm <input id="dist" type="number" min="1" max="300" step="1" value="20" /></label>
          <button onclick="setMode('manual')">Manual</button>
          <button onclick="setMode('mission')">Mission</button>
        </div>
        <div class="manual-grid">
          <div class="pad">
            <button onclick="moveDir('FL')">FL</button>
            <button onclick="moveDir('F')">F</button>
            <button onclick="moveDir('FR')">FR</button>
            <button onclick="moveDir('L')">L</button>
            <button class="danger" onclick="stopRobot()">STOP</button>
            <button onclick="moveDir('R')">R</button>
            <button onclick="moveDir('BL')">BL</button>
            <button onclick="moveDir('B')">B</button>
            <button onclick="moveDir('BR')">BR</button>
          </div>
          <div class="joystick-wrap">
            <div id="joystick" class="joystick" aria-label="manual velocity joystick">
              <div id="stick" class="stick"></div>
            </div>
            <div id="joystickState" class="joystick-state">joystick <strong>idle</strong> · 40 rpm</div>
          </div>
        </div>
        <div class="row" style="margin-top:10px">
          <label>Rotate deg <input id="rotdeg" type="number" min="1" max="360" step="5" value="90" /></label>
          <button onclick="rotate('left')">Rotate Left</button>
          <button onclick="rotate('right')">Rotate Right</button>
        </div>
        <div class="row">
          <button onclick="trigger('/api/init_imu')">Init IMU</button>
          <button onclick="trigger('/api/reset_odom')">Reset Odom</button>
          <button onclick="homeRobot()">Home</button>
          <button onclick="buzzer(true)">Buzzer On</button>
          <button onclick="buzzer(false)">Buzzer Off</button>
        </div>
      </div>
    </section>

    <section>
      <h2>IR Sensors</h2>
      <div class="panel ir">
        <div id="ir_front_left" class="pill">FL</div>
        <div id="ir_front" class="pill">Front</div>
        <div id="ir_front_right" class="pill">FR</div>
        <div id="ir_left" class="pill">Left</div>
        <div id="ir_right" class="pill">Right</div>
        <div id="ir_back" class="pill">Back</div>
      </div>
    </section>

    <section class="wide">
      <h2>Mission Audit</h2>
      <div class="panel">
        <div class="audit-rows">
          <div class="audit-row" data-lane="1">
            <label><input type="checkbox" id="lane1Enabled" class="side" value="1" checked /> Lane 1 side</label>
            <select id="lane1Product">
              <option value="indomie" selected>Indomie</option>
              <option value="fruit_rings_cereal">Fruit Rings Cereal</option>
              <option value="beans_can">Beans Can</option>
            </select>
            <label>Quantity <input id="lane1Qty" type="number" min="1" max="20" step="1" value="2" /></label>
          </div>
          <div class="audit-row" data-lane="2">
            <label><input type="checkbox" id="lane2Enabled" class="side" value="2" /> Lane 2 side</label>
            <select id="lane2Product">
              <option value="indomie">Indomie</option>
              <option value="fruit_rings_cereal" selected>Fruit Rings Cereal</option>
              <option value="beans_can">Beans Can</option>
            </select>
            <label>Quantity <input id="lane2Qty" type="number" min="1" max="20" step="1" value="2" /></label>
          </div>
        </div>
        <div class="row" style="margin-top:10px">
          <button class="accent" onclick="startAudit()">Start Audit</button>
          <button class="warn" onclick="lift(500)">Jog +500</button>
          <button class="warn" onclick="lift(-500)">Jog -500</button>
        </div>
      </div>
    </section>

    <section class="wide">
      <h2>Vision</h2>
      <div class="panel">
        <img id="cameraFeed" class="camera-feed" alt="camera feed" />
        <div class="row">
          <select id="scanShelf">
            <option value="indomie">Indomie</option>
            <option value="beans_can">Beans Can</option>
            <option value="fruit_rings_cereal">Fruit Rings Cereal</option>
          </select>
          <label>Quantity <input id="scanQty" type="number" min="1" max="20" step="1" value="2" /></label>
          <button class="accent" onclick="scanShelf()">Scan</button>
        </div>
        <div class="vision">
          <div class="metric"><span>Status</span><strong id="scanStatus">-</strong></div>
          <div class="metric"><span>Count</span><strong id="scanCount">-</strong></div>
          <div class="metric"><span>Confidence</span><strong id="scanConfidence">-</strong></div>
        </div>
        <div id="scanMessage" style="color:var(--muted); margin-top:8px; overflow-wrap:anywhere"></div>
        <img id="scanImage" class="scan-image" alt="latest scan" onclick="openImageModal(this.src, 'Latest scan')" />
      </div>
    </section>

    <section class="wide">
      <div class="panel">
        <div class="section-head">
          <h2>Last Mission Results</h2>
          <span id="missionScanTotal" class="count-badge">0 scans</span>
        </div>
        <div id="missionScans" class="scan-results">
          <div class="empty">No mission scans yet.</div>
        </div>
      </div>
    </section>

    <section class="wide">
      <h2>Log</h2>
      <div id="log" class="panel log"></div>
    </section>
  </div>
</main>
<div id="imageModal" class="image-modal" onclick="closeImageModal(event)">
  <div class="image-modal-inner">
    <div class="image-modal-bar">
      <span id="imageModalTitle">Scan image</span>
      <button onclick="closeImageModal()">Close</button>
    </div>
    <img id="imageModalImg" alt="annotated scan preview" />
  </div>
</div>
<script>
const logEl = document.getElementById('log');
const PRODUCT_LABELS = {
  indomie: 'Indomie',
  fruit_rings_cereal: 'Fruit Rings Cereal',
  beans_can: 'Beans Can',
  Indomie: 'Indomie',
  'Fruit Rings Cereal': 'Fruit Rings Cereal',
  'Beans Can': 'Beans Can'
};
let lastSeenEvent = '';
let lastCameraOk = false;
let lastMissionScanKey = '';
const JOYSTICK_RPM = 40;
const JOYSTICK_TIMEOUT_S = 0.3;
let joystickPointerId = null;
let joystickCommand = {forward_rpm: 0, strafe_rpm: 0, label: 'idle'};
let joystickTimer = null;
let joystickBlockedUntilRelease = false;
function log(msg) {
  const t = new Date().toLocaleTimeString();
  logEl.textContent = `[${t}] ${msg}\n` + logEl.textContent;
}
async function post(path, body={}) {
  const res = await fetch(path, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
  const data = await res.json();
  log(`${path}: ${JSON.stringify(data)}`);
  return data;
}
function dist() { return Number(document.getElementById('dist').value || 0); }
function rotdeg() { return Number(document.getElementById('rotdeg').value || 0); }
function moveDir(direction) { post('/api/move', {direction, distance_cm: dist()}); }
function rotate(direction) { post('/api/rotate', {direction, degrees: rotdeg()}); }
function stopRobot() { post('/api/stop'); }
function homeRobot() { post('/api/home'); }
function setMode(mode) { post('/api/mode', {mode}); }
function trigger(path) { post(path); }
function buzzer(on) { post('/api/buzzer', {on}); }
function lift(steps) { post('/api/lift', {steps}); }
async function sendJoystick(command) {
  try {
    const res = await fetch('/api/joystick', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({
      forward_rpm: command.forward_rpm,
      strafe_rpm: command.strafe_rpm,
      turn_rpm: 0,
      timeout_s: JOYSTICK_TIMEOUT_S
      })
    });
    const data = await res.json();
    if (!data.ok && command.label !== 'idle') {
      log(`joystick rejected: ${data.message}`);
      cancelJoystickUntilRelease('blocked');
    }
  } catch (e) {
    log(`joystick failed: ${e}`);
  }
}
async function sendJoystickStop() {
  try {
    await fetch('/api/joystick', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({forward_rpm: 0, strafe_rpm: 0, turn_rpm: 0, timeout_s: JOYSTICK_TIMEOUT_S})
    });
  } catch (e) {
    log(`joystick stop failed: ${e}`);
  }
}
function setJoystickState(label) {
  document.getElementById('joystickState').innerHTML = `joystick <strong>${escapeHtml(label)}</strong> · ${JOYSTICK_RPM} rpm`;
}
function moveStick(dx, dy) {
  document.getElementById('stick').style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
}
function joystickFromPoint(event) {
  const pad = document.getElementById('joystick');
  const rect = pad.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  let dx = event.clientX - cx;
  let dy = event.clientY - cy;
  const max = rect.width * 0.31;
  const mag = Math.hypot(dx, dy);
  if (mag > max) {
    dx = dx / mag * max;
    dy = dy / mag * max;
  }
  moveStick(dx, dy);
  const deadzone = rect.width * 0.10;
  if (Math.hypot(dx, dy) < deadzone) {
    return {forward_rpm: 0, strafe_rpm: 0, label: 'idle'};
  }
  const forward = dy < -deadzone ? JOYSTICK_RPM : dy > deadzone ? -JOYSTICK_RPM : 0;
  const strafe = dx < -deadzone ? -JOYSTICK_RPM : dx > deadzone ? JOYSTICK_RPM : 0;
  const parts = [];
  if (forward > 0) parts.push('F');
  if (forward < 0) parts.push('B');
  if (strafe < 0) parts.push('L');
  if (strafe > 0) parts.push('R');
  return {forward_rpm: forward, strafe_rpm: strafe, label: parts.join('') || 'idle'};
}
function cancelJoystickUntilRelease(label) {
  clearInterval(joystickTimer);
  joystickTimer = null;
  joystickPointerId = null;
  joystickBlockedUntilRelease = true;
  joystickCommand = {forward_rpm: 0, strafe_rpm: 0, label: 'idle'};
  moveStick(0, 0);
  setJoystickState(`${label} - release`);
  sendJoystickStop();
}
function startJoystick(event) {
  event.preventDefault();
  joystickBlockedUntilRelease = false;
  const pad = document.getElementById('joystick');
  joystickPointerId = event.pointerId;
  pad.setPointerCapture(joystickPointerId);
  joystickCommand = joystickFromPoint(event);
  setJoystickState(joystickCommand.label);
  sendJoystick(joystickCommand);
  clearInterval(joystickTimer);
  joystickTimer = setInterval(() => sendJoystick(joystickCommand), 100);
}
function updateJoystick(event) {
  if (joystickBlockedUntilRelease || joystickPointerId !== event.pointerId) return;
  event.preventDefault();
  joystickCommand = joystickFromPoint(event);
  setJoystickState(joystickCommand.label);
}
function stopJoystick() {
  clearInterval(joystickTimer);
  joystickTimer = null;
  joystickPointerId = null;
  joystickBlockedUntilRelease = false;
  joystickCommand = {forward_rpm: 0, strafe_rpm: 0, label: 'idle'};
  moveStick(0, 0);
  setJoystickState('idle');
  sendJoystick(joystickCommand);
}
function productLabel(value) {
  const raw = String(value ?? '');
  return PRODUCT_LABELS[raw] || raw.replaceAll('_', ' ').replace(/\b\w/g, ch => ch.toUpperCase());
}
function cleanList(values) {
  return (values || []).map(productLabel).join(', ');
}
async function scanShelf() {
  const shelf_id = document.getElementById('scanShelf').value;
  const expected_count = Math.max(1, Number(document.getElementById('scanQty').value || 2));
  const data = await post('/api/scan', {shelf_id, expected_count});
  if (data.scan) renderScan(data.scan);
}
function startAudit() {
  const shelves = [];
  const shelf_ids = [];
  const expected_counts = [];
  for (const side of [1, 2]) {
    if (!document.getElementById(`lane${side}Enabled`).checked) continue;
    shelves.push(side);
    shelf_ids.push(document.getElementById(`lane${side}Product`).value);
    expected_counts.push(Math.max(1, Number(document.getElementById(`lane${side}Qty`).value || 2)));
  }
  post('/api/audit', {shelves, shelf_ids, expected_counts, level_1: true, level_2: false});
}
function setText(id, value) { document.getElementById(id).textContent = value; }
function setIr(id, active) { document.getElementById(id).classList.toggle('on', !!active); }
function escapeHtml(value) {
  return String(value ?? '').replace(/[&<>"']/g, ch => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch]));
}
function renderScan(scan) {
  setText('scanStatus', scan?.status || '-');
  const count = scan ? `${scan.detected_count ?? 0}/${scan.expected_count ?? 0}` : '-';
  setText('scanCount', count);
  const confidence = Number(scan?.confidence ?? 0);
  setText('scanConfidence', scan ? confidence.toFixed(2) : '-');
  const wrong = cleanList(scan?.wrong_products || []);
  const location = scan?.side ? `side ${scan.side} | ` : '';
  document.getElementById('scanMessage').textContent = scan ? `${location}${productLabel(scan.shelf_id)}: ${scan.message || ''}${wrong ? ' | wrong: ' + wrong : ''}` : '';
  const img = document.getElementById('scanImage');
  if (scan?.image_path) {
    img.src = `/api/audit_image?path=${encodeURIComponent(scan.image_path)}&t=${Date.now()}`;
    img.style.display = 'block';
  } else {
    img.removeAttribute('src');
    img.style.display = 'none';
  }
}
function imageUrl(path) {
  return `/api/audit_image?path=${encodeURIComponent(path)}&t=${Date.now()}`;
}
function openImageModal(src, title) {
  if (!src) return;
  document.getElementById('imageModalImg').src = src;
  document.getElementById('imageModalTitle').textContent = title || 'Scan image';
  document.getElementById('imageModal').classList.add('open');
}
function closeImageModal(event) {
  if (event) {
    event.stopPropagation();
    if (event.target.closest && event.target.closest('.image-modal-inner') && event.target.id !== 'imageModal') return;
  }
  document.getElementById('imageModal').classList.remove('open');
  document.getElementById('imageModalImg').removeAttribute('src');
}
function renderMissionScans(scans) {
  const root = document.getElementById('missionScans');
  const items = Array.isArray(scans) ? scans : [];
  setText('missionScanTotal', `${items.length} scan${items.length === 1 ? '' : 's'}`);
  const key = JSON.stringify(items.map(scan => [
    scan?.side, scan?.shelf_id, scan?.status,
    scan?.detected_count, scan?.expected_count, scan?.confidence, scan?.image_path
  ]));
  if (key === lastMissionScanKey) return;
  lastMissionScanKey = key;
  if (!items.length) {
    root.innerHTML = '<div class="empty">No mission scans yet.</div>';
    return;
  }
  root.innerHTML = items.map((scan, index) => {
    const wrong = cleanList(scan?.wrong_products || []);
    const detected = cleanList(scan?.detected_products || []);
    const url = scan?.image_path ? imageUrl(scan.image_path) : '';
    const title = `Lane ${scan?.side || '-'} ${productLabel(scan?.shelf_id || 'scan')}`;
    const img = url
      ? `<img alt="${escapeHtml(title)}" src="${url}" onclick="openImageModal(this.src, this.alt)" />`
      : '<div class="empty">No image</div>';
    return `<article class="scan-card">
      ${img}
      <div class="scan-card-body">
        <div class="scan-card-title">
          <span>${escapeHtml(productLabel(scan?.shelf_id || 'scan'))}</span>
          <span>${escapeHtml(scan?.status || '-')}</span>
        </div>
        <div class="scan-card-meta">
          Lane ${escapeHtml(scan?.side || '-')}<br>
          Count ${escapeHtml(scan?.detected_count ?? 0)}/${escapeHtml(scan?.expected_count ?? 0)}
          &middot; Confidence ${Number(scan?.confidence ?? 0).toFixed(2)}<br>
          Expected ${escapeHtml(productLabel(scan?.expected_product || '-'))}<br>
          Detected ${escapeHtml(detected || '-')}<br>
          ${wrong ? `Wrong ${escapeHtml(wrong)}<br>` : ''}
          ${escapeHtml(scan?.message || '')}
        </div>
      </div>
    </article>`;
  }).join('');
}
function refreshCamera() {
  const img = document.getElementById('cameraFeed');
  img.src = `/api/camera.jpg?t=${Date.now()}`;
}
async function refresh() {
  try {
    const res = await fetch('/api/status');
    const s = await res.json();
    setText('mode', s.mode || '-');
    setText('event', s.last_event || '');
    if (s.last_event && s.last_event !== lastSeenEvent) {
      lastSeenEvent = s.last_event;
      log(`event: ${s.last_event}`);
    }
    setText('age', Number(s.telemetry_age_s ?? 0).toFixed(2) + 's');
    setText('forward', Number(s.telemetry?.forward_cm ?? 0).toFixed(1));
    setText('strafe', Number(s.telemetry?.strafe_cm ?? 0).toFixed(1));
    setText('yaw', Number(s.telemetry?.yaw_deg ?? 0).toFixed(1));
    setText('imu', s.telemetry?.imu_ok ? 'OK' : 'check');
    setText('move', s.telemetry?.mode || '-');
    setText('seq', s.telemetry?.seq ?? '-');
    for (const [name, active] of Object.entries(s.ir || {})) setIr('ir_' + name, active);
    if (s.latest_scan) renderScan(s.latest_scan);
    renderMissionScans(s.mission_scans || []);
  } catch (e) {
    setText('event', 'dashboard disconnected');
  }
}
setInterval(refresh, 250);
setInterval(refreshCamera, 300);
document.getElementById('joystick').addEventListener('pointerdown', startJoystick);
document.getElementById('joystick').addEventListener('pointermove', updateJoystick);
document.getElementById('joystick').addEventListener('pointerup', stopJoystick);
document.getElementById('joystick').addEventListener('pointercancel', stopJoystick);
document.getElementById('joystick').addEventListener('lostpointercapture', stopJoystick);
window.addEventListener('blur', stopJoystick);
refresh();
refreshCamera();
</script>
</body>
</html>
"""


class DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("web_dashboard")
        self.callback_group = ReentrantCallbackGroup()
        self.host = str(self.declare_parameter("host", "0.0.0.0").value)
        self.port = int(self.declare_parameter("port", 8080).value)
        self.mode = "manual"
        self.last_event = "ready"
        self.latest_ir: dict[str, bool] = {}
        self.latest_telemetry: dict[str, Any] = {}
        self.latest_scan: dict[str, Any] | None = None
        self.mission_scans: list[dict[str, Any]] = []
        self.telemetry_stamp = 0.0
        self.audit_image_dir = str(self.declare_parameter("audit_image_dir", "~/audix/audit_images").value)
        self.camera_jpeg_quality = int(self.declare_parameter("camera_jpeg_quality", 75).value)
        self.camera_min_period_s = float(self.declare_parameter("camera_min_period_s", 0.20).value)
        self.camera_lock = threading.Lock()
        self.camera_bridge = CvBridge()
        self.latest_camera_jpeg: bytes | None = None
        self.camera_stamp = 0.0
        self.last_camera_encode_s = 0.0

        self.direction_client = self.create_client(DirectionCommand, "manager/direction_move", callback_group=self.callback_group)
        self.twist_client = self.create_client(TwistCommand, "manager/twist", callback_group=self.callback_group)
        self.rotate_client = self.create_client(RotateCommand, "manager/rotate", callback_group=self.callback_group)
        self.mode_client = self.create_client(SetRobotMode, "manager/set_mode", callback_group=self.callback_group)
        self.audit_client = self.create_client(AuditMission, "manager/start_audit", callback_group=self.callback_group)
        self.home_client = self.create_client(Trigger, "manager/go_home", callback_group=self.callback_group)
        self.stop_client = self.create_client(Trigger, "manager/stop", callback_group=self.callback_group)
        self.init_imu_client = self.create_client(Trigger, "esp/init_imu", callback_group=self.callback_group)
        self.reset_odom_client = self.create_client(Trigger, "esp/reset_odom", callback_group=self.callback_group)
        self.buzzer_client = self.create_client(SetBool, "gpio/set_buzzer", callback_group=self.callback_group)
        self.lift_client = self.create_client(LiftMoveSteps, "lift/move_steps", callback_group=self.callback_group)
        self.scan_client = self.create_client(ShelfScan, "scan_shelf", callback_group=self.callback_group)

        self.create_subscription(IrState, "ir/state", self._on_ir, 10, callback_group=self.callback_group)
        self.create_subscription(EspTelemetry, "esp/telemetry", self._on_telemetry, 10, callback_group=self.callback_group)
        self.create_subscription(Image, "image_raw", self._on_camera, 5, callback_group=self.callback_group)
        self.create_subscription(String, "mission/event", self._on_event, 20, callback_group=self.callback_group)
        self.create_subscription(String, "vision/scan_result", self._on_scan_result, 10, callback_group=self.callback_group)

        handler = self._make_handler()
        self.httpd = ThreadingHTTPServer((self.host, self.port), handler)
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(f"Web dashboard ready at http://{self.host}:{self.port}")

    def _on_ir(self, msg: IrState) -> None:
        self.latest_ir = {
            "front_left": bool(msg.front_left),
            "front": bool(msg.front),
            "front_right": bool(msg.front_right),
            "left": bool(msg.left),
            "right": bool(msg.right),
            "back": bool(msg.back),
        }

    def _on_telemetry(self, msg: EspTelemetry) -> None:
        self.telemetry_stamp = self.get_clock().now().nanoseconds / 1e9
        self.latest_telemetry = {
            "mode": msg.mode,
            "seq": int(msg.seq),
            "imu_ok": bool(msg.imu_ok),
            "yaw_deg": float(msg.yaw_deg),
            "forward_cm": float(msg.forward_cm),
            "strafe_cm": float(msg.strafe_cm),
            "progress_cm": float(msg.progress_cm),
            "remaining_cm": float(msg.remaining_cm),
            "move_done": bool(msg.move_done),
        }

    def _on_event(self, msg: String) -> None:
        self.last_event = msg.data

    def _on_scan_result(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            scan = {
                "success": bool(data.get("success", False)),
                "side": int(data.get("side", 0)),
                "level": int(data.get("level", 0)),
                "shelf_id": str(data.get("shelf_id", "")),
                "expected_product": str(data.get("expected_product", "")),
                "expected_count": int(data.get("expected_count", 0)),
                "detected_count": int(data.get("detected_count", 0)),
                "detected_products": list(data.get("detected_products", [])),
                "wrong_products": list(data.get("wrong_products", [])),
                "confidence": float(data.get("confidence", 0.0)),
                "status": str(data.get("status", "")),
                "message": str(data.get("message", "")),
                "image_path": str(data.get("image_path", "")),
            }
            self.latest_scan = scan
            self.mission_scans.append(scan)
        except Exception as exc:
            self.get_logger().warning(f"bad scan result message: {exc}")

    def _on_camera(self, msg: Image) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_camera_encode_s < max(0.05, self.camera_min_period_s):
            return
        try:
            frame = self.camera_bridge.imgmsg_to_cv2(msg, "bgr8")
            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), max(25, min(95, self.camera_jpeg_quality))],
            )
            if not ok:
                return
        except Exception as exc:
            self.get_logger().warning(f"camera encode failed: {exc}")
            return
        with self.camera_lock:
            self.latest_camera_jpeg = encoded.tobytes()
            self.camera_stamp = now
            self.last_camera_encode_s = now

    def _call_sync(self, client, request, timeout_s: float = 10.0):
        if not client.wait_for_service(timeout_sec=max(0.1, float(timeout_s))):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        event = threading.Event()
        holder = {}
        future = client.call_async(request)
        future.add_done_callback(lambda done: (holder.setdefault("future", done), event.set()))
        if not event.wait(timeout_s):
            raise TimeoutError(f"timed out waiting for {client.srv_name}")
        return holder["future"].result()

    def _status(self) -> dict[str, Any]:
        now = self.get_clock().now().nanoseconds / 1e9
        return {
            "mode": self.mode,
            "last_event": self.last_event,
            "ir": self.latest_ir,
            "telemetry": self.latest_telemetry,
            "telemetry_age_s": max(0.0, now - self.telemetry_stamp) if self.telemetry_stamp else None,
            "latest_scan": self.latest_scan,
            "mission_scans": self.mission_scans,
            "camera_age_s": max(0.0, now - self.camera_stamp) if self.camera_stamp else None,
        }

    @staticmethod
    def _scan_to_dict(res) -> dict[str, Any]:
        return {
            "success": bool(res.success),
            "shelf_id": str(res.shelf_id),
            "expected_product": str(res.expected_product),
            "expected_count": int(res.expected_count),
            "detected_count": int(res.detected_count),
            "detected_products": list(res.detected_products),
            "wrong_products": list(res.wrong_products),
            "confidence": float(res.confidence),
            "status": str(res.status),
            "message": str(res.message),
            "image_path": str(res.image_path),
        }

    def _make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, _format: str, *_args) -> None:
                return

            def _send(self, status: int, body: bytes, content_type: str) -> None:
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _json(self, payload: dict[str, Any], status: int = 200) -> None:
                self._send(status, json.dumps(payload).encode("utf-8"), "application/json")

            def _read_json(self) -> dict[str, Any]:
                length = int(self.headers.get("Content-Length", "0") or "0")
                if length <= 0:
                    return {}
                return json.loads(self.rfile.read(length).decode("utf-8"))

            def do_GET(self) -> None:
                parsed = urlparse(self.path)
                if parsed.path == "/" or parsed.path == "/index.html":
                    self._send(200, INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
                elif parsed.path == "/api/status":
                    self._json(node._status())
                elif parsed.path == "/api/audit_image":
                    params = parse_qs(parsed.query)
                    image_path = params.get("path", [""])[0]
                    self._send_audit_image(image_path)
                elif parsed.path == "/api/camera.jpg":
                    self._send_camera_frame()
                else:
                    self._json({"ok": False, "message": "not found"}, 404)

            def _send_camera_frame(self) -> None:
                with node.camera_lock:
                    frame = node.latest_camera_jpeg
                if frame is None:
                    self._json({"ok": False, "message": "no camera frame yet"}, 404)
                    return
                self._send(200, frame, "image/jpeg")

            def _send_audit_image(self, image_path: str) -> None:
                base = Path(node.audit_image_dir).expanduser().resolve()
                try:
                    requested = Path(image_path).expanduser().resolve()
                    if not requested.is_file() or (requested != base and base not in requested.parents):
                        self._json({"ok": False, "message": "image not found"}, 404)
                        return
                    self._send(200, requested.read_bytes(), "image/jpeg")
                except Exception as exc:
                    self._json({"ok": False, "message": str(exc)}, 404)

            def do_POST(self) -> None:
                try:
                    data = self._read_json()
                    if self.path == "/api/move":
                        req = DirectionCommand.Request()
                        req.direction = str(data.get("direction", ""))
                        req.distance_cm = float(data.get("distance_cm", 0.0))
                        req.timeout_s = float(data.get("timeout_s", 0.0))
                        res = node._call_sync(node.direction_client, req, 200.0)
                        self._json({"ok": res.ok, "result": res.result, "message": res.message})
                    elif self.path == "/api/joystick":
                        req = TwistCommand.Request()
                        req.forward_rpm = float(data.get("forward_rpm", 0.0))
                        req.strafe_rpm = float(data.get("strafe_rpm", 0.0))
                        req.turn_rpm = float(data.get("turn_rpm", 0.0))
                        req.timeout_s = float(data.get("timeout_s", 0.3))
                        res = node._call_sync(node.twist_client, req, 2.0)
                        self._json({"ok": res.ok, "message": res.message, "raw_json": res.raw_json})
                    elif self.path == "/api/rotate":
                        req = RotateCommand.Request()
                        req.direction = str(data.get("direction", ""))
                        req.degrees = float(data.get("degrees", 0.0))
                        req.timeout_s = float(data.get("timeout_s", 10.0))
                        res = node._call_sync(node.rotate_client, req, 30.0)
                        self._json({"ok": res.ok, "result": res.result, "message": res.message, "heading_deg": res.heading_deg})
                    elif self.path == "/api/mode":
                        req = SetRobotMode.Request()
                        req.mode = str(data.get("mode", "manual"))
                        res = node._call_sync(node.mode_client, req, 5.0)
                        if res.ok:
                            node.mode = res.active_mode
                        self._json({"ok": res.ok, "message": res.message, "mode": res.active_mode})
                    elif self.path == "/api/audit":
                        req = AuditMission.Request()
                        req.shelves = [int(v) for v in data.get("shelves", [])]
                        req.level_1 = bool(data.get("level_1", True))
                        req.level_2 = bool(data.get("level_2", True))
                        req.shelf_ids = [str(v) for v in data.get("shelf_ids", [])]
                        req.expected_counts = [max(1, int(v)) for v in data.get("expected_counts", [])]
                        node.mission_scans = []
                        node.latest_scan = None
                        res = node._call_sync(node.audit_client, req, 5.0)
                        self._json({"ok": res.accepted, "message": res.message})
                    elif self.path == "/api/stop":
                        res = node._call_sync(node.stop_client, Trigger.Request(), 5.0)
                        node.mode = "manual"
                        self._json({"ok": res.success, "message": res.message})
                    elif self.path == "/api/home":
                        res = node._call_sync(node.home_client, Trigger.Request(), 220.0)
                        node.mode = "manual"
                        self._json({"ok": res.success, "message": res.message})
                    elif self.path == "/api/init_imu":
                        res = node._call_sync(node.init_imu_client, Trigger.Request(), 12.0)
                        self._json({"ok": res.success, "message": res.message})
                    elif self.path == "/api/reset_odom":
                        res = node._call_sync(node.reset_odom_client, Trigger.Request(), 5.0)
                        self._json({"ok": res.success, "message": res.message})
                    elif self.path == "/api/buzzer":
                        req = SetBool.Request()
                        req.data = bool(data.get("on", False))
                        res = node._call_sync(node.buzzer_client, req, 3.0)
                        self._json({"ok": res.success, "message": res.message})
                    elif self.path == "/api/lift":
                        raw_steps = int(data.get("steps", 0))
                        if "direction" in data:
                            direction = 1 if int(data.get("direction", 1)) >= 0 else -1
                            signed_steps = abs(raw_steps) * direction
                        else:
                            signed_steps = raw_steps
                        req = LiftMoveSteps.Request()
                        req.steps = abs(signed_steps)
                        req.direction = 1 if signed_steps >= 0 else -1
                        req.speed_sps = float(data.get("speed_sps", 500.0))
                        res = node._call_sync(node.lift_client, req, 15.0)
                        self._json({"ok": res.ok, "message": res.message})
                    elif self.path == "/api/scan":
                        req = ShelfScan.Request()
                        req.shelf_id = str(data.get("shelf_id", ""))
                        req.expected_count = max(1, int(data.get("expected_count", 2)))
                        res = node._call_sync(node.scan_client, req, 30.0)
                        scan = node._scan_to_dict(res)
                        node.latest_scan = scan
                        self._json({"ok": res.success, "message": res.message, "scan": scan})
                    else:
                        self._json({"ok": False, "message": "not found"}, 404)
                except Exception as exc:
                    self._json({"ok": False, "message": str(exc)}, 500)

        return Handler

    def destroy_node(self) -> bool:
        try:
            self.httpd.shutdown()
            self.httpd.server_close()
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = DashboardNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
