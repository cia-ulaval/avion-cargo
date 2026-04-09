import { refs, modeButtons } from './dom.js';
import { appState } from './state.js';
import {
  formatAgeFromSeconds,
  formatDegrees,
  formatFreshnessFromMs,
  formatLatLon,
  formatMetric,
  gpsFixLabel,
  isPlainObject,
} from './utils.js';
import { extractDroneTelemetry, extractTrackingTelemetry, renderTelemetryTree } from './telemetry.js';

export function setText(node, value) {
  if (!node) {
    return;
  }

  node.textContent = value ?? '--';
}

export function setLinkDots(isOnline) {
  const color = isOnline ? '#8cd65d' : '#ff7676';
  const shadow = isOnline ? '0 0 0 6px rgba(140, 214, 93, 0.12)' : '0 0 0 6px rgba(255, 118, 118, 0.12)';
  refs.headerLinkDot.style.background = color;
  refs.connectionDot.style.background = color;
  refs.headerLinkDot.style.boxShadow = shadow;
  refs.connectionDot.style.boxShadow = shadow;
}

export function setStreamState(state, label) {
  appState.connectionState = state;
  setText(refs.statusText, label);
  setText(refs.streamStatePill, label);
  setText(refs.channelStateLabel, state);
  setText(refs.linkStateLabel, state === 'connected' ? 'Telemetry live' : 'Offline link');
  setLinkDots(state === 'connected');
}

export function highlightMode(mode) {
  modeButtons.forEach((button) => {
    button.classList.toggle('mode-button-active', button.dataset.mode === mode);
  });
}

export function updateCompass(headingDeg) {
  const safeHeading = Number.isFinite(Number(headingDeg)) ? ((Number(headingDeg) % 360) + 360) % 360 : null;
  const rotation = safeHeading ?? 0;
  if (refs.compassNeedle) {
    refs.compassNeedle.style.transform = `translate(-50%, -50%) rotate(${rotation}deg)`;
  }
}

export function updateMap(poseUav) {
  const x = Number(poseUav?.x);
  const y = Number(poseUav?.y);
  const maxOffset = 70;

  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    refs.mapDrone.style.transform = 'translate(-50%, -50%)';
    return;
  }

  const offsetX = Math.max(-maxOffset, Math.min(maxOffset, x * 28));
  const offsetY = Math.max(-maxOffset, Math.min(maxOffset, y * 28));
  refs.mapDrone.style.transform = `translate(calc(-50% + ${offsetX}px), calc(-50% + ${offsetY}px))`;
}

export function renderTracking(tracking) {
  const poseCamera = isPlainObject(tracking.poseCamera) ? tracking.poseCamera : null;
  const poseUav = isPlainObject(tracking.poseUav) ? tracking.poseUav : null;
  const attitude = isPlainObject(tracking.attitude) ? tracking.attitude : {};
  const hasTarget = Boolean(poseCamera) || tracking.markerId !== null;

  setText(refs.markerId, tracking.markerId === null ? '--' : String(tracking.markerId));
  setText(refs.trackingStatus, hasTarget ? 'Locked' : 'Searching');
  setText(refs.markerLockChip, hasTarget ? 'Locked' : 'Searching');
  setText(refs.targetStatePill, hasTarget ? 'Target locked' : 'Target lost');
  refs.targetStatePill.classList.toggle('status-pill-accent', hasTarget);

  setText(refs.poseX, formatMetric(poseCamera?.x, ' m'));
  setText(refs.poseY, formatMetric(poseCamera?.y, ' m'));
  setText(refs.poseZ, formatMetric(poseCamera?.z, ' m'));
  setText(refs.uavPoseX, formatMetric(poseUav?.x, ' m'));
  setText(refs.uavPoseY, formatMetric(poseUav?.y, ' m'));
  setText(refs.uavPoseZ, formatMetric(poseUav?.z, ' m'));

  setText(refs.attRoll, formatDegrees(attitude.roll ?? tracking.roll));
  setText(refs.attPitch, formatDegrees(attitude.pitch ?? tracking.pitch));
  setText(refs.attYaw, formatDegrees(attitude.yaw ?? tracking.yaw));
  setText(refs.cameraFramePill, poseCamera ? 'Camera frame' : 'No pose');

  updateMap(poseUav);
}

export function renderDrone(drone) {
  const mode = typeof drone.mode === 'string' ? drone.mode : 'UNKNOWN';
  const batteryPct = Number(drone.battery_remaining_pct);
  const voltage = Number(drone.battery_voltage_v);
  const altitude = Number(drone.alt_m);
  const groundspeed = Number(drone.groundspeed_mps);
  const relativeAltitude = Number(drone.relative_altitude);
  const verticalSpeed = Number(drone.speed);
  const latitude = Number(drone.latitude);
  const longitude = Number(drone.longitude);
  const headingDeg = Number(drone.heading_deg);

  setText(refs.modeValue, mode);
  setText(refs.armedValue, typeof drone.armed === 'boolean' ? (drone.armed ? 'YES' : 'NO') : '--');
  setText(refs.speedValue, formatMetric(groundspeed, ' m/s', 1));
  setText(refs.altitudeValue, formatMetric(altitude, ' m', 1));
  setText(refs.batteryValue, Number.isFinite(batteryPct) ? `${batteryPct}%` : '--');
  setText(refs.voltageValue, formatMetric(voltage, ' V', 1));
  setText(refs.gpsValue, gpsFixLabel(drone.gps_fix_type));
  setText(refs.heartbeatValue, formatAgeFromSeconds(drone.last_heartbeat_s));
  setText(refs.signalValue, formatAgeFromSeconds(drone.last_signal_gpio_s));
  setText(refs.latitudeValue, formatLatLon(latitude, 'lat'));
  setText(refs.longitudeValue, formatLatLon(longitude, 'lon'));
  setText(refs.batteryChip, Number.isFinite(batteryPct) ? `${batteryPct}%` : '--');

  setText(refs.relativeAltPill, formatMetric(relativeAltitude, ' m', 1));
  setText(refs.relativeAltValue, formatMetric(relativeAltitude, ' m', 1));
  setText(refs.verticalSpeedValue, formatMetric(verticalSpeed, ' m/s', 1));
  setText(
    refs.coordinatesValue,
    Number.isFinite(latitude) && Number.isFinite(longitude)
      ? `${formatLatLon(latitude, 'lat')} / ${formatLatLon(longitude, 'lon')}`
      : '--'
  );
  setText(refs.headingValue, formatDegrees(headingDeg));

  highlightMode(mode);
  updateCompass(headingDeg);
}

export function renderPayload(payload) {
  const tracking = extractTrackingTelemetry(payload);
  const drone = extractDroneTelemetry(payload);

  appState.lastPayload = payload;
  appState.lastTelemetryAtMs = Date.now();
  appState.lastTracking = tracking;
  if (Object.keys(drone).length > 0) {
    appState.lastDrone = drone;
  }

  renderTracking(tracking);
  renderDrone(appState.lastDrone);
  renderTelemetryTree(payload);
  refreshRuntimeLabels();
}

export function refreshRuntimeLabels() {
  const hasTelemetry = appState.lastTelemetryAtMs > 0;
  const ageMs = hasTelemetry ? Date.now() - appState.lastTelemetryAtMs : Number.NaN;
  const isFresh = hasTelemetry && ageMs < 3000;

  setText(refs.telemetryAge, hasTelemetry ? formatFreshnessFromMs(ageMs) : 'No signal');
  setText(refs.lastUpdateLabel, hasTelemetry ? formatFreshnessFromMs(ageMs) : 'never');

  if (appState.connectionState === 'connected' && isFresh) {
    setText(refs.linkStateLabel, 'Telemetry live');
  } else if (appState.connectionState === 'connected') {
    setText(refs.linkStateLabel, 'Waiting telemetry');
  }
}
