<script setup lang="ts">
import { computed, onMounted, onUnmounted, ref } from 'vue';
import { invoke } from '@tauri-apps/api/core';
import { listen, type UnlistenFn } from '@tauri-apps/api/event';

type Port = { name: string; description: string };
type Status = {
  connected: boolean;
  armed: boolean;
  armed_confirmed: boolean;
  session_active: boolean;
  pending_action: string | null;
  payload_peer_id: number;
  controller_device_id: number | null;
  peer_connected: boolean;
  last_error: string | null;
};

// Manual-control commands are physical velocities (m/s), not normalized joystick values.
// Keep the requested translation below the simulator's 0.2 m/s wheel surface-speed limit.
const MANUAL_TRANSLATION_SPEED_M_S = 0.15;

const ports = ref<Port[]>([]);
const selectedPort = ref('/tmp/manual-controller-node');
const payloadPeerId = ref('18');
const baudRate = ref('460800');
const status = ref<Status>({ connected: false, armed: false, armed_confirmed: false, session_active: false, pending_action: null, payload_peer_id: 18, controller_device_id: null, peer_connected: false, last_error: null });
const busy = ref(false);
const error = ref('');
const pressed = ref(new Set<string>());
let heartbeat: number | undefined;
let poller: number | undefined;
let positionAgeTimer: number | undefined;
let unlistenStatus: UnlistenFn | undefined;
let unlistenPosition: UnlistenFn | undefined;
let unlistenPositionStatus: UnlistenFn | undefined;
let unlistenPositionFrame: UnlistenFn | undefined;
let lastPositionReceivedAt = 0;
let lastPositionSequence: number | null = null;
let lastPositionDeviceUs: number | null = null;
let lastPositionUiReceivedAt = 0;
let uiPositionMessages = 0;
let uiPositionWindowStartedAt = 0;
type AnchorPosition = { id: string; x: number; y: number };
type PositionFrameTelemetry = {
  anchors_xy_m: AnchorPosition[];
  payload_to_tag_xy_m: [number, number];
  tag_height_m: number;
  origin_world_m: [number, number, number];
  x_axis_world: [number, number, number];
  y_axis_world: [number, number, number];
};
type PositioningStatusTelemetry = {
  status: string;
  range_count: number;
  rejected_ranges: number;
};
type PositionTelemetry = {
  sequence: number;
  position_m: { x: number; y: number };
  tag_position_m: { x: number; y: number };
  residual_m: number | null;
  covariance_xy_m2: [number, number, number] | null;
  device_monotonic_us: number | null;
  heading_rad: number;
  heading_confidence: number;
  heading_observations: number;
  range_count?: number;
  rejected_ranges?: number;
  quality?: string;
  controller_received_unix_ms?: number;
};
const positioning = ref<PositionTelemetry | null>(null);
const positioningAvailability = ref<PositioningStatusTelemetry | null>(null);
const positioningFrame = ref<PositionFrameTelemetry | null>(null);
const positioningAgeMs = ref<number | null>(null);
const formatMetres = (value: number) => `${value.toFixed(3)} m`;
const formatHeading = (value: number) => `${(value * 180 / Math.PI).toFixed(1)}°`;
const formatVector = (value: [number, number, number]) => `(${value.map(component => component.toFixed(3)).join(', ')}) m`;
const formatOffset = (value: [number, number]) => `(${value.map(component => component.toFixed(3)).join(', ')}) m`;
const positionSigma = (covariance: [number, number, number] | null) => {
  if (!covariance) return '—';
  const [xx, xy, yy] = covariance;
  const largestEigenvalue = (xx + yy + Math.sqrt((xx - yy) ** 2 + 4 * xy ** 2)) / 2;
  return `±${Math.sqrt(Math.max(0, largestEigenvalue)).toFixed(3)} m`;
};
const positioningStale = computed(() =>
  positioningAvailability.value !== null && positioningAvailability.value.status !== 'tracking'
    ? true
    : positioning.value === null || positioningAgeMs.value === null || positioningAgeMs.value > 1200
);
const positioningStatusLabel = computed(() => {
  if (positioningAvailability.value && positioningAvailability.value.status !== 'tracking') {
    return `Waiting for fresh ranges (${positioningAvailability.value.range_count}/3)`;
  }
  if (!positioning.value) return 'Waiting for position telemetry';
  if (positioningStale.value) return 'Telemetry stale';
  return 'Live estimate';
});
const headingInitialized = computed(() => (positioning.value?.heading_observations ?? 0) > 0);
const worldPayloadPosition = computed(() => {
  const point = positioning.value?.position_m;
  const frame = positioningFrame.value;
  if (!point || !frame) return null;
  const [ox, oy, oz] = frame.origin_world_m;
  const [xx, xy, xz] = frame.x_axis_world;
  const [yx, yy, yz] = frame.y_axis_world;
  const nx = xy * yz - xz * yy;
  const ny = xz * yx - xx * yz;
  const nz = xx * yy - xy * yx;
  if (Math.abs(nz) < 1e-6) return null;
  const baseZ = oz + point.x * xz + point.y * yz;
  const normalDistance = (frame.tag_height_m - baseZ) / nz;
  return {
    x: ox + point.x * xx + point.y * yx + normalDistance * nx,
    y: oy + point.x * xy + point.y * yy + normalDistance * ny,
    z: frame.tag_height_m,
  };
});
const plotLayout = computed(() => {
  const anchors = positioningFrame.value?.anchors_xy_m ?? [];
  const points = anchors.map(anchor => ({ x: anchor.x, y: anchor.y }));
  if (positioning.value) points.push({ x: positioning.value.position_m.x, y: positioning.value.position_m.y });
  if (positioning.value?.tag_position_m) {
    points.push({ x: positioning.value.tag_position_m.x, y: positioning.value.tag_position_m.y });
  }
  if (!points.length) points.push({ x: -1, y: -1 }, { x: 1, y: 1 });
  const minX = Math.min(...points.map(point => point.x));
  const maxX = Math.max(...points.map(point => point.x));
  const minY = Math.min(...points.map(point => point.y));
  const maxY = Math.max(...points.map(point => point.y));
  const pad = Math.max(0.35, Math.max(maxX - minX, maxY - minY) * 0.16);
  const x0 = minX - pad, x1 = maxX + pad, y0 = minY - pad, y1 = maxY + pad;
  const left = 44, right = 676, top = 24, bottom = 356;
  const scale = Math.min((right - left) / Math.max(0.01, x1 - x0), (bottom - top) / Math.max(0.01, y1 - y0));
  return {
    scale,
    mapX: (x: number) => left + ((right - left) - (x1 - x0) * scale) / 2 + (x - x0) * scale,
    mapY: (y: number) => bottom - ((bottom - top) - (y1 - y0) * scale) / 2 - (y - y0) * scale,
  };
});
const mapX = (x: number) => plotLayout.value.mapX(x);
const mapY = (y: number) => plotLayout.value.mapY(y);
const anchorPolygonPoints = computed(() => {
  const anchors = positioningFrame.value?.anchors_xy_m ?? [];
  if (anchors.length < 3) return '';
  return [...anchors, anchors[0]].map(anchor => `${mapX(anchor.x)},${mapY(anchor.y)}`).join(' ');
});
const headingArrow = computed(() => {
  const position = positioning.value?.position_m;
  if (!position) return null;
  const length = 0.32;
  const angle = positioning.value!.heading_rad;
  return {
    x1: mapX(position.x),
    y1: mapY(position.y),
    x2: mapX(position.x + length * Math.cos(angle)),
    y2: mapY(position.y + length * Math.sin(angle)),
  };
});
const uncertaintyEllipse = computed(() => {
  const covariance = positioning.value?.covariance_xy_m2;
  // The solver covariance belongs to the UWB tag estimate, not the
  // payload reference point obtained by applying the heading-dependent lever arm.
  const position = positioning.value?.tag_position_m;
  if (!covariance || !position) return null;
  const [xx, xy, yy] = covariance;
  const root = Math.sqrt(Math.max(0, (xx - yy) ** 2 + 4 * xy ** 2));
  const first = Math.max(0, (xx + yy + root) / 2);
  const second = Math.max(0, (xx + yy - root) / 2);
  return {
    cx: mapX(position.x),
    cy: mapY(position.y),
    rx: Math.max(1.5, 2 * Math.sqrt(first) * plotLayout.value.scale),
    ry: Math.max(1.5, 2 * Math.sqrt(second) * plotLayout.value.scale),
    rotation: -0.5 * Math.atan2(2 * xy, xx - yy) * 180 / Math.PI,
  };
});
type QueuedCommand = {
  type: string;
  vx: number;
  vy: number;
  yaw: number;
  resolve: () => void;
};
const commandQueue: QueuedCommand[] = [];
let commandInFlight = false;
let refreshingStatus = false;
let lastStatusRequestAt = 0;

function isBleTransmitBusy(error: unknown): boolean {
  return String(error).includes('D3K command 0x55 rejected with status 4');
}

const linkLabel = computed(() => !status.value.connected ? 'Controller disconnected' : status.value.peer_connected ? `Payload ${status.value.payload_peer_id} connected` : `Controller connected · waiting for payload ${status.value.payload_peer_id}`);
const armedLabel = computed(() => status.value.pending_action === 'arm' ? 'Arming…' : status.value.pending_action === 'disarm' ? 'Disarming…' : !status.value.armed_confirmed ? 'State unknown' : status.value.armed && !status.value.session_active ? 'Armed by another controller' : status.value.armed ? 'Armed' : 'Disarmed');
const isArmed = computed(() => status.value.armed && status.value.armed_confirmed && status.value.session_active && status.value.peer_connected && !status.value.pending_action);

async function refreshPorts() {
  ports.value = await invoke<Port[]>('list_ports');
  if (!selectedPort.value && ports.value.length) selectedPort.value = ports.value[0].name;
}
async function refreshStatus() {
  if (!status.value.connected || refreshingStatus) return;
  refreshingStatus = true;
  try {
    status.value = await invoke<Status>('refresh_peer_status');
    if (status.value.last_error) error.value = status.value.last_error;
  } catch (e) { error.value = String(e); }
  finally { refreshingStatus = false; }
}
async function connect() {
  const peer = Number(payloadPeerId.value);
  if (!selectedPort.value) { error.value = 'Select the controller D3K serial port.'; return; }
  if (!Number.isInteger(peer) || peer < 1 || peer > 65534) { error.value = 'Payload peer ID must be between 1 and 65534.'; return; }
  busy.value = true; error.value = '';
  try {
    await invoke('connect_controller', { portName: selectedPort.value, baudRate: Number(baudRate.value), payloadPeerId: peer });
    status.value = await invoke<Status>('get_status');
    await refreshStatus();
  } catch (e) { error.value = String(e); }
  finally { busy.value = false; }
}
function drainCommandQueue() {
  if (commandInFlight || !commandQueue.length) return;
  commandInFlight = true;
  void (async () => {
    try {
      while (commandQueue.length && status.value.connected) {
        const command = commandQueue.shift()!;
        try {
          await invoke('send_control', {
            commandType: command.type,
            vxMS: command.vx,
            vyMS: command.vy,
            yawRateRadS: command.yaw,
          });
          error.value = '';
          command.resolve();
        } catch (e) {
          if (isBleTransmitBusy(e)) {
            // D3K allows one active outgoing BLE message per peer. BUSY is
            // backpressure while the previous message is still being consumed.
            // A drive setpoint is refreshed by the heartbeat, so drop this copy
            // but retain the held keys and retry the newest setpoint shortly.
            // Safety transitions must remain queued until they are accepted.
            error.value = '';
            if (command.type === 'drive' || command.type === 'status') {
              command.resolve();
              await new Promise(resolve => window.setTimeout(resolve, 60));
              break;
            }
            commandQueue.unshift(command);
            await new Promise(resolve => window.setTimeout(resolve, 60));
            continue;
          }

          command.resolve();
          error.value = String(e);
          if (command.type === 'drive') pressed.value.clear();
          await refreshStatus();
          if (command.type === 'drive') {
            commandQueue.splice(0, commandQueue.length);
            break;
          }
        }
      }
    } finally {
      commandInFlight = false;
      if (commandQueue.length && status.value.connected) drainCommandQueue();
    }
  })();
}
function send(type: string, vx = 0, vy = 0, yaw = 0): Promise<void> {
  if (!status.value.connected) return Promise.resolve();
  return new Promise<void>((resolve, reject) => {
    const command: QueuedCommand = { type, vx, vy, yaw, resolve };
    if (type === 'drive') {
      let lastDrive = -1;
      for (let index = 0; index < commandQueue.length; index++) {
        if (commandQueue[index].type === 'drive') lastDrive = index;
      }
      if (lastDrive >= 0) {
        commandQueue[lastDrive].resolve();
        commandQueue.splice(lastDrive, 1, command);
      } else {
        commandQueue.push(command);
      }
    } else if (type === 'disarm') {
      for (const queued of commandQueue) queued.resolve();
      commandQueue.splice(0, commandQueue.length);
      commandQueue.unshift(command);
    } else if (type === 'stop') {
      for (let index = commandQueue.length - 1; index >= 0; index--) {
        if (commandQueue[index].type === 'drive') {
          commandQueue[index].resolve();
          commandQueue.splice(index, 1);
        }
      }
      commandQueue.push(command);
    } else {
      commandQueue.push(command);
    }
    drainCommandQueue();
  });
}
async function arm() {
  if (!status.value.connected || !status.value.peer_connected || !status.value.armed_confirmed || status.value.armed || status.value.pending_action || busy.value) return;
  busy.value = true;
  try { await send('arm'); }
  finally { busy.value = false; }
}
async function disarm() {
  pressed.value.clear();
  busy.value = true;
  try { await send('disarm'); }
  finally { busy.value = false; }
}
async function disconnect() {
  pressed.value.clear();
  busy.value = true;
  try {
    await invoke('disconnect_controller');
    status.value = await invoke<Status>('get_status');
  } catch (e) { error.value = String(e); }
  finally { busy.value = false; }
}
function velocity() {
  const keys = pressed.value;
  let vx = (keys.has('w') ? 1 : 0) - (keys.has('s') ? 1 : 0);
  let vy = (keys.has('a') ? 1 : 0) - (keys.has('d') ? 1 : 0);
  const length = Math.hypot(vx, vy);
  if (length > 1) { vx /= length; vy /= length; }
  const yaw = (keys.has('q') ? 1 : 0) - (keys.has('e') ? 1 : 0);
  return {
    vx: vx * MANUAL_TRANSLATION_SPEED_M_S,
    vy: vy * MANUAL_TRANSLATION_SPEED_M_S,
    yaw,
  };
}
function updateDrive() {
  if (!isArmed.value) return;
  const v = velocity();
  void send('drive', v.vx, v.vy, v.yaw);
}
function keyDown(event: KeyboardEvent) {
  if (event.repeat || event.ctrlKey || event.altKey || event.metaKey) return;
  const key = event.key.toLowerCase();
  if (!['w','a','s','d','q','e'].includes(key)) return;
  if ((event.target as HTMLElement | null)?.matches('input,textarea,select')) return;
  event.preventDefault();
  if (!isArmed.value || !status.value.peer_connected) return;
  pressed.value.add(key);
  updateDrive();
}
function keyUp(event: KeyboardEvent) {
  const key = event.key.toLowerCase();
  if (!['w','a','s','d','q','e'].includes(key)) return;
  if (pressed.value.delete(key)) updateDrive();
}
function clearKeys() {
  if (!pressed.value.size) return;
  pressed.value.clear();
  if (isArmed.value) void send('stop');
}
function stopControls() {
  pressed.value.clear();
  if (isArmed.value) void send('stop');
}
function pointerDown(key: string) {
  if (!isArmed.value || !status.value.peer_connected) return;
  pressed.value.add(key); updateDrive();
}
function pointerUp(key: string) {
  if (pressed.value.delete(key)) updateDrive();
}

onMounted(async () => {
  await refreshPorts();
  status.value = await invoke<Status>('get_status');
  unlistenStatus = await listen<Status>('controller-status', event => { status.value = event.payload; });
  unlistenPositionFrame = await listen<PositionFrameTelemetry>('payload-position-frame', event => {
    console.info('[telemetry-ui] reference frame received', event.payload);
    positioningFrame.value = event.payload;
  });
  unlistenPosition = await listen<PositionTelemetry>('payload-position', event => {
    const now = Date.now();
    const message = event.payload;
    const sequenceGap = lastPositionSequence === null ? null : message.sequence - lastPositionSequence;
    const deviceDeltaUs = message.device_monotonic_us !== null && lastPositionDeviceUs !== null
      ? message.device_monotonic_us - lastPositionDeviceUs
      : null;
    console.info('[telemetry-ui] position received', {
      sequence: message.sequence,
      sequenceGap,
      uiReceiveIntervalMs: lastPositionUiReceivedAt ? now - lastPositionUiReceivedAt : null,
      deviceMonotonicUs: message.device_monotonic_us,
      deviceDeltaUs,
      controllerToUiMs: message.controller_received_unix_ms ? now - message.controller_received_unix_ms : null,
      position: message.position_m,
      tagPosition: message.tag_position_m,
      residualM: message.residual_m,
      quality: message.quality,
      headingRad: message.heading_rad,
      rangeCount: message.range_count,
      rejectedRanges: message.rejected_ranges,
    });
    lastPositionSequence = message.sequence;
    lastPositionDeviceUs = message.device_monotonic_us;
    lastPositionUiReceivedAt = now;
    uiPositionMessages += 1;
    if (!uiPositionWindowStartedAt) uiPositionWindowStartedAt = now;
    if (now - uiPositionWindowStartedAt >= 1000) {
      console.info('[telemetry-ui] 1s receive rate', {
        messages: uiPositionMessages,
        rateHz: uiPositionMessages * 1000 / (now - uiPositionWindowStartedAt),
      });
      uiPositionMessages = 0;
      uiPositionWindowStartedAt = now;
    }
    positioning.value = message;
    positioningAvailability.value = {
      status: 'tracking',
      range_count: event.payload.range_count ?? 3,
      rejected_ranges: event.payload.rejected_ranges ?? 0,
    };
    lastPositionReceivedAt = Date.now();
    positioningAgeMs.value = 0;
  });
  unlistenPositionStatus = await listen<PositioningStatusTelemetry>('payload-position-status', event => {
    console.info('[telemetry-ui] positioning status received', event.payload);
    positioningAvailability.value = event.payload;
  });
  window.addEventListener('keydown', keyDown);
  window.addEventListener('keyup', keyUp);
  window.addEventListener('blur', clearKeys);
  heartbeat = window.setInterval(() => { if (isArmed.value) updateDrive(); }, 100);
  positionAgeTimer = window.setInterval(() => {
    positioningAgeMs.value = lastPositionReceivedAt ? Date.now() - lastPositionReceivedAt : null;
  }, 250);
  poller = window.setInterval(() => {
    if (!status.value.connected) return;
    // Keep draining D3K BLE events while commands are being queued. The Rust
    // controller serializes UART access; suppressing polls during a sustained
    // drive heartbeat can leave delivery events unprocessed and inbound data
    // unread until the firmware expires it.
    void refreshStatus();
    // Only query until the payload has acknowledged this UI session. After
    // that, arm/disarm/watchdog status messages are sent by the payload itself.
    if (status.value.peer_connected && !status.value.armed_confirmed && Date.now() - lastStatusRequestAt >= 900) {
      lastStatusRequestAt = Date.now();
      void send('status');
    }
    // Peer-link status changes much more slowly than motor setpoints. Polling it
    // every 100 ms contends with the command serial channel without improving
    // control responsiveness. Keep the 100 ms drive heartbeat separate.
  }, 250);
});
onUnmounted(() => {
  clearKeys();
  window.removeEventListener('keydown', keyDown);
  window.removeEventListener('keyup', keyUp);
  window.removeEventListener('blur', clearKeys);
  if (heartbeat) clearInterval(heartbeat);
  if (poller) clearInterval(poller);
  if (positionAgeTimer) clearInterval(positionAgeTimer);
  unlistenStatus?.();
  unlistenPosition?.();
  unlistenPositionStatus?.();
  unlistenPositionFrame?.();
  if (isArmed.value || status.value.pending_action === 'arm') void invoke('send_control', { commandType: 'disarm', vxMS: 0, vyMS: 0, yawRateRadS: 0 });
});
</script>

<template>
  <main class="shell">
    <header class="topbar">
      <div><div class="eyebrow">CIA · PAYLOAD V0</div><h1>Manual control</h1></div>
      <div class="indicators">
        <span class="indicator" :class="status.connected ? 'good' : 'bad'"><i />{{ status.connected ? `Controller ${status.controller_device_id ?? 'online'}` : 'Controller offline' }}</span>
        <span class="indicator" :class="status.peer_connected ? 'good' : 'waiting'"><i />{{ status.peer_connected ? 'Payload BLE linked' : 'Payload not linked' }}</span>
        <span class="armed-pill" :class="isArmed ? 'armed' : ''">{{ armedLabel }}</span>
      </div>
    </header>

    <section class="connection panel">
      <div class="section-title"><div><span class="eyebrow">CONNECTION</span><h2>Controller D3K node</h2></div><span class="status-line">{{ linkLabel }}</span></div>
      <div class="connection-fields">
        <label>Controller serial port
          <div class="inline-field"><input v-model="selectedPort" list="controller-ports" placeholder="/dev/ttyACM0 or /tmp/manual-controller-node"><datalist id="controller-ports"><option v-for="p in ports" :key="p.name" :value="p.name">{{ p.description }}</option></datalist><button class="secondary" @click="refreshPorts">Refresh</button></div>
        </label>
        <label>Payload BLE peer ID<input v-model="payloadPeerId" type="number" min="1" max="65534" :disabled="status.connected"></label>
        <label>Serial speed<select v-model="baudRate" :disabled="status.connected"><option value="115200">115200 · USB CDC</option><option value="460800">460800 · UART / simulator PTY</option></select></label>
        <div class="connect-actions"><button v-if="!status.connected" class="primary" :disabled="busy || !selectedPort" @click="connect">Connect</button><button v-else class="secondary" :disabled="busy" @click="disconnect">Disconnect</button></div>
      </div>
      <p class="hint">Connect to the separate controller D3K node (USB CDC: 115200; UART / simulator PTY: 460800). Configure it as BLE HUB/Central, enable BLE, and include the payload node ID in its peer allow-list. The UI reads the payload's arm state acknowledgement; it does not configure the controller node's BLE settings.</p>
    </section>

    <section class="control-layout">
      <div class="panel controls-panel">
        <div class="section-title"><div><span class="eyebrow">DRIVE</span><h2>Movement controls</h2></div><span class="key-hint">Hold keys or buttons</span></div>
        <div class="controller">
          <div class="dpad" aria-label="Translation controls">
            <button class="pad up" :class="{active:pressed.has('w')}" @pointerdown.prevent="pointerDown('w')" @pointerup="pointerUp('w')" @pointerleave="pointerUp('w')" @pointercancel="pointerUp('w')"><b>W</b><small>+X</small></button>
            <button class="pad left" :class="{active:pressed.has('a')}" @pointerdown.prevent="pointerDown('a')" @pointerup="pointerUp('a')" @pointerleave="pointerUp('a')" @pointercancel="pointerUp('a')"><b>A</b><small>+Y</small></button>
            <div class="pad center"><span>REFERENCE</span><small>PLANE</small></div>
            <button class="pad right" :class="{active:pressed.has('d')}" @pointerdown.prevent="pointerDown('d')" @pointerup="pointerUp('d')" @pointerleave="pointerUp('d')" @pointercancel="pointerUp('d')"><b>D</b><small>−Y</small></button>
            <button class="pad down" :class="{active:pressed.has('s')}" @pointerdown.prevent="pointerDown('s')" @pointerup="pointerUp('s')" @pointerleave="pointerUp('s')" @pointercancel="pointerUp('s')"><b>S</b><small>−X</small></button>
          </div>
          <div class="rotation">
            <div class="rotation-label">YAW ROTATION</div>
            <div class="rotation-buttons">
              <button class="rotate" :class="{active:pressed.has('q')}" @pointerdown.prevent="pointerDown('q')" @pointerup="pointerUp('q')" @pointerleave="pointerUp('q')" @pointercancel="pointerUp('q')"><b>Q</b><span>↶</span><small>Counter-clockwise</small></button>
              <button class="rotate" :class="{active:pressed.has('e')}" @pointerdown.prevent="pointerDown('e')" @pointerup="pointerUp('e')" @pointerleave="pointerUp('e')" @pointercancel="pointerUp('e')"><b>E</b><span>↷</span><small>Clockwise</small></button>
            </div>
          </div>
        </div>
        <div class="limits"><span>Wheel speed <b>Actuator-limited</b></span><span>Command refresh <b>100 ms</b></span></div>
      </div>

      <aside class="panel safety-panel">
        <span class="eyebrow">MOTOR CONTROL</span><h2>Safety state</h2>
        <div class="state-card" :class="isArmed ? 'state-armed' : status.armed && !status.session_active ? 'state-other-session' : ''"><span class="state-dot"/><div><strong>{{ armedLabel }}</strong><small>{{ isArmed ? 'Payload confirmed armed for this controller session' : status.armed && !status.session_active ? 'Movement is locked until this session disarms and arms again' : status.armed_confirmed ? 'Payload confirmed disarmed' : 'Waiting for payload state acknowledgement' }}</small></div></div>
        <button v-if="!status.armed" class="arm-button" :disabled="!status.connected || !status.peer_connected || !status.armed_confirmed || !!status.pending_action || busy" @click="arm">ARM MOTORS</button>
        <button v-else class="disarm-button" :disabled="!status.connected || !status.peer_connected || !!status.pending_action || busy" @click="disarm">DISARM</button>
        <button class="stop-button" :disabled="!isArmed" @click="stopControls">STOP / RELEASE</button>
        <p class="hint">Arm only after confirming the payload is clear to move. The armed indicator changes only after a status message is received back from the payload. Releasing all controls sends a zero setpoint. A 1000 ms payload watchdog stops and disarms if drive commands stop arriving.</p>
      </aside>
    </section>

    <section class="panel positioning-panel">
      <div class="section-title">
        <div><span class="eyebrow">UWB POSITIONING · LIVE AUDIT</span><h2>Anchor reference frame</h2></div>
        <span class="position-live" :class="positioningStale ? 'stale' : 'live'">{{ positioningStatusLabel }}</span>
      </div>
      <p v-if="positioningFrame" class="frame-definition">
        Local XY origin (world): <b>{{ formatVector(positioningFrame.origin_world_m) }}</b>
        · +X world axis: <b>{{ formatVector(positioningFrame.x_axis_world) }}</b>
        · +Y world axis: <b>{{ formatVector(positioningFrame.y_axis_world) }}</b>
      </p>
      <div class="position-audit">
        <div class="position-map-wrap">
          <svg class="position-map" viewBox="0 0 720 390" role="img" aria-label="Top-down anchor reference frame with estimated payload position and heading">
            <defs>
              <marker id="heading-arrowhead" markerWidth="7" markerHeight="7" refX="5" refY="3.5" orient="auto">
                <path d="M0,0 L7,3.5 L0,7 z" />
              </marker>
            </defs>
            <line class="map-axis" x1="42" :y1="mapY(0)" x2="678" :y2="mapY(0)" />
            <line class="map-axis" :x1="mapX(0)" y1="22" :x2="mapX(0)" y2="358" />
            <polygon v-if="anchorPolygonPoints" class="anchor-triangle" :points="anchorPolygonPoints" />
            <g v-for="anchor in (positioningFrame?.anchors_xy_m ?? [])" :key="anchor.id">
              <circle class="anchor-marker" :cx="mapX(anchor.x)" :cy="mapY(anchor.y)" r="7" />
              <text class="anchor-label" :x="mapX(anchor.x) + 11" :y="mapY(anchor.y) - 10">{{ anchor.id }}</text>
              <text class="anchor-coordinate" :x="mapX(anchor.x) + 11" :y="mapY(anchor.y) + 5">({{ anchor.x.toFixed(2) }}, {{ anchor.y.toFixed(2) }})</text>
            </g>
            <ellipse
              v-if="uncertaintyEllipse && positioning && !positioningStale"
              class="position-uncertainty"
              :cx="uncertaintyEllipse.cx"
              :cy="uncertaintyEllipse.cy"
              :rx="uncertaintyEllipse.rx"
              :ry="uncertaintyEllipse.ry"
              :transform="`rotate(${uncertaintyEllipse.rotation} ${uncertaintyEllipse.cx} ${uncertaintyEllipse.cy})`"
            />
            <line
              v-if="positioning && positioning.tag_position_m"
              class="tag-offset-line"
              :x1="mapX(positioning.position_m.x)"
              :y1="mapY(positioning.position_m.y)"
              :x2="mapX(positioning.tag_position_m.x)"
              :y2="mapY(positioning.tag_position_m.y)"
            />
            <line
              v-if="headingArrow && positioning && headingInitialized && positioning.heading_confidence > 0"
              class="heading-vector"
              :class="{ stale: positioningStale }"
              :x1="headingArrow.x1"
              :y1="headingArrow.y1"
              :x2="headingArrow.x2"
              :y2="headingArrow.y2"
              marker-end="url(#heading-arrowhead)"
            />
            <circle
              v-if="positioning"
              class="payload-marker"
              :class="{ stale: positioningStale }"
              :cx="mapX(positioning.position_m.x)"
              :cy="mapY(positioning.position_m.y)"
              r="8"
            />
            <circle
              v-if="positioning?.tag_position_m"
              class="tag-marker"
              :cx="mapX(positioning.tag_position_m.x)"
              :cy="mapY(positioning.tag_position_m.y)"
              r="4"
            />
            <text class="axis-label" x="660" y="179">+X</text>
            <text class="axis-label" x="370" y="34">+Y</text>
            <text v-if="!positioning" class="map-empty" x="360" y="195" text-anchor="middle">Waiting for a valid three-anchor solution</text>
          </svg>
          <div class="map-legend">
            <span><i class="legend-anchor" /> UWB anchor</span>
            <span><i class="legend-payload" /> Payload reference point</span>
            <span><i class="legend-heading" /> Heading vector</span>
            <span><i class="legend-uncertainty" /> 2σ tag XY uncertainty</span>
          </div>
        </div>
        <div class="position-readout">
          <div class="readout-group">
            <span class="readout-label">Payload position · local plane XY</span>
            <div class="coordinate-row"><span>Local X</span><strong>{{ positioning ? formatMetres(positioning.position_m.x) : '—' }}</strong></div>
            <div class="coordinate-row"><span>Local Y</span><strong>{{ positioning ? formatMetres(positioning.position_m.y) : '—' }}</strong></div>
            <div class="coordinate-row"><span>World X</span><strong>{{ worldPayloadPosition ? formatMetres(worldPayloadPosition.x) : '—' }}</strong></div>
            <div class="coordinate-row"><span>World Y</span><strong>{{ worldPayloadPosition ? formatMetres(worldPayloadPosition.y) : '—' }}</strong></div>
            <div class="coordinate-row"><span>Tag height · ground Z</span><strong>{{ positioningFrame ? formatMetres(positioningFrame.tag_height_m) : '—' }}</strong></div>
            <div class="coordinate-row"><span>Body → tag offset</span><strong>{{ positioningFrame ? formatOffset(positioningFrame.payload_to_tag_xy_m) : '—' }}</strong></div>
          </div>
          <div class="readout-group heading-readout">
            <span class="readout-label">Heading estimate · from +X</span>
            <strong class="heading-value">{{ !positioning ? '—' : headingInitialized ? formatHeading(positioning.heading_rad) : 'Uninitialized (0° default)' }}</strong>
            <div class="heading-confidence"><span>Confidence</span><b>{{ positioning ? `${(positioning.heading_confidence * 100).toFixed(0)}%` : '—' }}</b></div>
            <div class="heading-confidence"><span>Accepted observations</span><b>{{ positioning?.heading_observations ?? 0 }}</b></div>
          </div>
          <div class="readout-group quality-readout">
            <span class="readout-label">Position quality</span>
            <div class="quality-row"><span>Range RMS residual</span><b>{{ positioning?.residual_m === null || !positioning ? '—' : formatMetres(positioning.residual_m) }}</b></div>
            <div class="quality-row"><span>Estimated tag XY σ</span><b>{{ positionSigma(positioning?.covariance_xy_m2 ?? null) }}</b></div>
            <div class="quality-row"><span>Solver quality</span><b>{{ positioning?.quality ?? (positioning?.residual_m === null || !positioning ? '—' : positioning.residual_m < 0.08 ? 'good' : 'degraded') }}</b></div>
            <div class="quality-row"><span>Fresh anchor ranges</span><b>{{ positioning?.range_count ?? '—' }} / {{ positioningFrame?.anchors_xy_m.length ?? '—' }}</b></div>
            <div class="quality-row"><span>Rejected range outliers</span><b>{{ positioningAvailability?.rejected_ranges ?? positioning?.rejected_ranges ?? '—' }}</b></div>
            <div class="quality-row"><span>Range anchors</span><b>{{ positioningFrame?.anchors_xy_m.map(anchor => anchor.id).join(', ') ?? '—' }}</b></div>
            <div class="quality-row"><span>Telemetry age</span><b>{{ positioningAgeMs === null ? '—' : `${(positioningAgeMs / 1000).toFixed(1)} s` }}</b></div>
          </div>
        </div>
      </div>
      <p class="positioning-note">Position is re-solved after each accepted individual range using the latest fresh range from each anchor; measurements are never grouped into synchronized sweeps. Heading is inferred from stable translation only. Stops and in-place rotation do not provide heading observations.</p>
    </section>

    <footer class="footer"><span>WASD · fixed anchor reference plane (+X / −X, +Y / −Y)</span><span>Q / E · rotate about plane normal</span><span>Heading inferred during translation</span></footer>
    <div v-if="error" class="error" role="alert">{{ error }}<button @click="error=''">Dismiss</button></div>
  </main>
</template>
