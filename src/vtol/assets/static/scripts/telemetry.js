import { refs } from './dom.js';
import { isPlainObject } from './utils.js';

export function hasDroneFields(candidate) {
  if (!isPlainObject(candidate)) {
    return false;
  }

  return [
    'mode',
    'alt_m',
    'groundspeed_mps',
    'battery_voltage_v',
    'battery_remaining_pct',
    'gps_fix_type',
    'armed',
    'last_heartbeat_s',
    'last_signal_gpio_s',
    'relative_altitude',
    'latitude',
    'longitude',
    'heading_deg',
  ].some((key) => key in candidate);
}

export function extractDroneTelemetry(payload) {
  const directCandidates = [
    payload?.drone,
    payload?.drone_status,
    payload?.droneStatus,
    payload?.vehicle,
    payload?.vehicle_status,
  ];

  for (const candidate of directCandidates) {
    if (hasDroneFields(candidate)) {
      return candidate;
    }
  }

  if (hasDroneFields(payload?.status)) {
    return payload.status;
  }

  if (hasDroneFields(payload)) {
    return payload;
  }

  return {};
}

export function extractTrackingTelemetry(payload) {
  return {
    poseCamera:
      payload?.poses?.estimated_pose_from_camera ??
      payload?.estimated_pose_from_camera ??
      payload?.pose ??
      null,
    poseUav: payload?.poses?.estimated_pose_to_uav ?? payload?.estimated_pose_to_uav ?? payload?.uav_pose ?? null,
    markerId: payload?.marker_id ?? payload?.markerId ?? null,
    status: payload?.status,
    attitude: payload?.attitude ?? {},
  };
}

function telemetryValueClass(value) {
  if (typeof value === 'string') {
    return 'telemetry-type-string';
  }

  if (typeof value === 'number') {
    return 'telemetry-type-number';
  }

  if (typeof value === 'boolean') {
    return 'telemetry-type-boolean';
  }

  return 'telemetry-type-null';
}

function formatTelemetryLeaf(value) {
  if (typeof value === 'string') {
    return `"${value}"`;
  }

  if (value === null) {
    return 'null';
  }

  return String(value);
}

function buildTelemetryNode(key, value, level = 0) {
  const wrapper = document.createElement('div');
  wrapper.className = 'telemetry-node';

  const row = document.createElement('div');
  row.className = `telemetry-entry${level === 0 ? ' telemetry-entry-root' : ''}`;

  if (key !== null) {
    const keyNode = document.createElement('span');
    keyNode.className = 'telemetry-key';
    keyNode.textContent = key;
    row.appendChild(keyNode);
  }

  const isObject = isPlainObject(value);
  const isArray = Array.isArray(value);
  const isBranch = isObject || isArray;

  if (!isBranch) {
    const valueNode = document.createElement('span');
    valueNode.className = telemetryValueClass(value);
    valueNode.textContent = formatTelemetryLeaf(value);
    row.appendChild(valueNode);
    wrapper.appendChild(row);
    return wrapper;
  }

  const entries = isArray
    ? value.map((item, index) => [String(index), item])
    : Object.entries(value);

  const toggle = document.createElement('button');
  toggle.type = 'button';
  toggle.className = 'telemetry-toggle';
  toggle.setAttribute('aria-expanded', 'true');

  const brace = document.createElement('span');
  brace.className = 'telemetry-brace';
  brace.textContent = `${isArray ? '[' : '{'} ${entries.length}`;
  toggle.appendChild(brace);
  row.appendChild(toggle);
  wrapper.appendChild(row);

  const children = document.createElement('div');
  children.className = 'telemetry-children';

  for (const [childKey, childValue] of entries) {
    children.appendChild(buildTelemetryNode(childKey, childValue, level + 1));
  }

  const footer = document.createElement('div');
  footer.className = 'telemetry-entry';
  const footerBrace = document.createElement('span');
  footerBrace.className = 'telemetry-brace';
  footerBrace.textContent = isArray ? ']' : '}';
  footer.appendChild(footerBrace);
  children.appendChild(footer);

  toggle.addEventListener('click', () => {
    const expanded = toggle.getAttribute('aria-expanded') === 'true';
    toggle.setAttribute('aria-expanded', expanded ? 'false' : 'true');
    children.hidden = expanded;
  });

  wrapper.appendChild(children);
  return wrapper;
}

export function renderTelemetryTree(payload) {
  refs.telemetry.innerHTML = '';

  if (!isPlainObject(payload) || Object.keys(payload).length === 0) {
    const emptyNode = document.createElement('div');
    emptyNode.className = 'telemetry-empty';
    emptyNode.textContent = '(no data)';
    refs.telemetry.appendChild(emptyNode);
    return;
  }

  refs.telemetry.appendChild(buildTelemetryNode(null, payload));
}
