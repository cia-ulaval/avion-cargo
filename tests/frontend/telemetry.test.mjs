import assert from 'node:assert/strict';
import { test } from 'node:test';

const nodes = new Map();
function element() {
  return {
    textContent: '', style: {}, dataset: {}, children: [],
    classList: { toggle() {} },
    setAttribute() {}, addEventListener() {},
    appendChild(child) { this.children.push(child); },
  };
}
globalThis.document = {
  getElementById(id) {
    if (!nodes.has(id)) nodes.set(id, element());
    return nodes.get(id);
  },
  querySelector: element,
  querySelectorAll: () => [],
  createElement: element,
};
const { formatMetric, formatLatLon, gpsFixLabel } = await import('../../src/assets/static/scripts/utils.js');
const { renderPayload, renderTracking, refreshRuntimeLabels, setStreamState } =
  await import('../../src/assets/static/scripts/ui.js');
const { appState } = await import('../../src/assets/static/scripts/state.js');

test('unknown numeric fields stay absent while measured zero remains visible', () => {
  for (const value of [null, undefined, '', false, '0', NaN, Infinity]) {
    assert.equal(formatMetric(value), '--');
    assert.equal(formatLatLon(value, 'lat'), '--');
    assert.equal(gpsFixLabel(value), '--');
  }
  assert.equal(formatMetric(0, ' m'), '0.00 m');
  assert.equal(gpsFixLabel(0), 'NO GPS');
});

test('missing tracking data is searching and does not place a false target on the map', () => {
  renderTracking({});
  assert.equal(nodes.get('tracking-status').textContent, 'Searching');
  assert.equal(nodes.get('marker-id').textContent, '--');
  assert.equal(nodes.get('map-drone').hidden, true);
});

test('each telemetry snapshot replaces the previous drone and respects heartbeat freshness', () => {
  setStreamState('connected', 'Connected');
  renderPayload({ status: 2, drone: { mode: 'LAND', connected: true, last_heartbeat_s: Date.now() / 1000 } });
  assert.equal(nodes.get('mode-value').textContent, 'LAND');
  assert.equal(nodes.get('link-state-label').textContent, 'Drone link live');
  renderPayload({ status: 2 });
  assert.equal(nodes.get('mode-value').textContent, 'UNKNOWN');
  assert.equal(nodes.get('link-state-label').textContent, 'Drone link stale');
  renderPayload({ status: 2, drone: { connected: true, last_heartbeat_s: Date.now() / 1000 - 10 } });
  assert.equal(nodes.get('link-state-label').textContent, 'Drone link stale');
});

test('stopped or silent telemetry clears the previously locked target', () => {
  setStreamState('connected', 'Connected');
  const payload = { status: 1, marker_id: 0, poses: {
    estimated_pose_from_camera: { x: 0, y: 0, z: 2 },
    estimated_pose_to_uav: { x: 0, y: 0, z: 2 },
  } };
  renderPayload(payload);
  assert.equal(nodes.get('tracking-status').textContent, 'Locked');
  appState.lastTelemetryAtMs = Date.now() - 3100;
  refreshRuntimeLabels();
  assert.equal(nodes.get('tracking-status').textContent, 'Searching');
  assert.equal(nodes.get('pose-z').textContent, '--');
  renderPayload(payload);
  setStreamState('stopped', 'Stopped');
  assert.equal(nodes.get('tracking-status').textContent, 'Searching');
});
