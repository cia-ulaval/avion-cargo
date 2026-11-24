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

const ports = ref<Port[]>([]);
const selectedPort = ref('/tmp/manual-controller-node');
const payloadPeerId = ref('18');
const baudRate = ref('460800');
const status = ref<Status>({ connected: false, armed: false, armed_confirmed: false, session_active: false, pending_action: null, payload_peer_id: 18, controller_device_id: null, peer_connected: false, last_error: null });
const busy = ref(false);
const error = ref('');
const pressed = ref(new Set<string>());
const maxYaw = 0.8;
let heartbeat: number | undefined;
let poller: number | undefined;
let unlistenStatus: UnlistenFn | undefined;
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
  return { vx, vy, yaw: yaw * maxYaw };
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
  window.addEventListener('keydown', keyDown);
  window.addEventListener('keyup', keyUp);
  window.addEventListener('blur', clearKeys);
  heartbeat = window.setInterval(() => { if (isArmed.value) updateDrive(); }, 100);
  poller = window.setInterval(() => {
    if (!status.value.connected) return;
    if (!commandInFlight) void refreshStatus();
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
  unlistenStatus?.();
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
            <button class="pad up" :class="{active:pressed.has('w')}" @pointerdown.prevent="pointerDown('w')" @pointerup="pointerUp('w')" @pointerleave="pointerUp('w')" @pointercancel="pointerUp('w')"><b>W</b><small>Forward</small></button>
            <button class="pad left" :class="{active:pressed.has('a')}" @pointerdown.prevent="pointerDown('a')" @pointerup="pointerUp('a')" @pointerleave="pointerUp('a')" @pointercancel="pointerUp('a')"><b>A</b><small>Left</small></button>
            <div class="pad center"><span>BODY</span><small>FRAME</small></div>
            <button class="pad right" :class="{active:pressed.has('d')}" @pointerdown.prevent="pointerDown('d')" @pointerup="pointerUp('d')" @pointerleave="pointerUp('d')" @pointercancel="pointerUp('d')"><b>D</b><small>Right</small></button>
            <button class="pad down" :class="{active:pressed.has('s')}" @pointerdown.prevent="pointerDown('s')" @pointerup="pointerUp('s')" @pointerleave="pointerUp('s')" @pointercancel="pointerUp('s')"><b>S</b><small>Reverse</small></button>
          </div>
          <div class="rotation">
            <div class="rotation-label">YAW ROTATION</div>
            <div class="rotation-buttons">
              <button class="rotate" :class="{active:pressed.has('q')}" @pointerdown.prevent="pointerDown('q')" @pointerup="pointerUp('q')" @pointerleave="pointerUp('q')" @pointercancel="pointerUp('q')"><b>Q</b><span>↶</span><small>Counter-clockwise</small></button>
              <button class="rotate" :class="{active:pressed.has('e')}" @pointerdown.prevent="pointerDown('e')" @pointerup="pointerUp('e')" @pointerleave="pointerUp('e')" @pointercancel="pointerUp('e')"><b>E</b><span>↷</span><small>Clockwise</small></button>
            </div>
          </div>
        </div>
        <div class="limits"><span>Wheel speed <b>Actuator-limited</b></span><span>Rotation cap <b>0.80 rad/s</b></span><span>Command refresh <b>100 ms</b></span></div>
      </div>

      <aside class="panel safety-panel">
        <span class="eyebrow">MOTOR CONTROL</span><h2>Safety state</h2>
        <div class="state-card" :class="isArmed ? 'state-armed' : status.armed && !status.session_active ? 'state-other-session' : ''"><span class="state-dot"/><div><strong>{{ armedLabel }}</strong><small>{{ isArmed ? 'Payload confirmed armed for this controller session' : status.armed && !status.session_active ? 'Movement is locked until this session disarms and arms again' : status.armed_confirmed ? 'Payload confirmed disarmed' : 'Waiting for payload state acknowledgement' }}</small></div></div>
        <button v-if="!status.armed" class="arm-button" :disabled="!status.connected || !status.peer_connected || !status.armed_confirmed || !!status.pending_action || busy" @click="arm">ARM MOTORS</button>
        <button v-else class="disarm-button" :disabled="!status.connected || !status.peer_connected || !!status.pending_action || busy" @click="disarm">DISARM</button>
        <button class="stop-button" :disabled="!isArmed" @click="stopControls">STOP / RELEASE</button>
        <p class="hint">Arm only after confirming the payload is clear to move. The armed indicator changes only after a status message is received back from the payload. Releasing all controls sends a zero setpoint. A 300 ms payload watchdog stops and disarms if drive commands stop arriving.</p>
      </aside>
    </section>

    <footer class="footer"><span>WASD · translation relative to payload orientation</span><span>Q / E · yaw</span><span>Open-loop control · no IMU feedback</span></footer>
    <div v-if="error" class="error" role="alert">{{ error }}<button @click="error=''">Dismiss</button></div>
  </main>
</template>
