let peerConnection = null;
let dataChannel = null;

const appState = {
  lastPayload: null,
  lastTelemetryAtMs: 0,
  lastDrone: {},
  lastTracking: {},
  connectionState: "stopped",
};

const refs = {
  video: document.getElementById("v"),
  videoStage: document.querySelector(".video-stage"),
  telemetry: document.getElementById("telemetry"),
  statusText: document.getElementById("st"),
  startButton: document.getElementById("startButton"),
  stopButton: document.getElementById("stopButton"),
  streamStatePill: document.getElementById("stream-state-pill"),
  targetStatePill: document.getElementById("target-state-pill"),
  linkStateLabel: document.getElementById("link-state-label"),
  headerLinkDot: document.getElementById("header-link-dot"),
  connectionDot: document.getElementById("connection-dot"),
  channelStateLabel: document.getElementById("channel-state-label"),
  telemetryAge: document.getElementById("telemetry-age"),
  lastUpdateLabel: document.getElementById("last-update-label"),
  markerId: document.getElementById("marker-id"),
  trackingStatus: document.getElementById("tracking-status"),
  markerLockChip: document.getElementById("marker-lock-chip"),
  poseX: document.getElementById("pose-x"),
  poseY: document.getElementById("pose-y"),
  poseZ: document.getElementById("pose-z"),
  uavPoseX: document.getElementById("uav-pose-x"),
  uavPoseY: document.getElementById("uav-pose-y"),
  uavPoseZ: document.getElementById("uav-pose-z"),
  attRoll: document.getElementById("att-roll"),
  attPitch: document.getElementById("att-pitch"),
  attYaw: document.getElementById("att-yaw"),
  altitudeMain: document.getElementById("altitude-main"),
  speedMain: document.getElementById("speed-main"),
  batteryMain: document.getElementById("battery-main"),
  gpsMain: document.getElementById("gps-main"),
  latMain: document.getElementById("lat-main"),
  lonMain: document.getElementById("lon-main"),
  modeChip: document.getElementById("mode-chip"),
  batteryChip: document.getElementById("battery-chip"),
  cameraFramePill: document.getElementById("camera-frame-pill"),
  headingChip: document.getElementById("heading-chip"),
  modeValue: document.getElementById("mode-value"),
  armedValue: document.getElementById("armed-value"),
  speedValue: document.getElementById("speed-value"),
  altitudeValue: document.getElementById("altitude-value"),
  batteryValue: document.getElementById("battery-value"),
  voltageValue: document.getElementById("voltage-value"),
  gpsValue: document.getElementById("gps-value"),
  heartbeatValue: document.getElementById("heartbeat-value"),
  signalValue: document.getElementById("signal-value"),
  relativeAltPill: document.getElementById("relative-alt-pill"),
  relativeAltValue: document.getElementById("relative-alt-value"),
  verticalSpeedValue: document.getElementById("vertical-speed-value"),
  coordinatesValue: document.getElementById("coordinates-value"),
  headingValue: document.getElementById("heading-value"),
  headingReadout: document.getElementById("heading-readout"),
  compassNeedle: document.getElementById("compass-needle"),
  mapDrone: document.getElementById("map-drone"),
};

const modeButtons = Array.from(document.querySelectorAll("[data-mode]"));

function isPlainObject(value) {
  return Boolean(value) && typeof value === "object" && !Array.isArray(value);
}

function hasDroneFields(candidate) {
  if (!isPlainObject(candidate)) {
    return false;
  }

  return [
    "mode",
    "alt_m",
    "groundspeed_mps",
    "battery_voltage_v",
    "battery_remaining_pct",
    "gps_fix_type",
    "armed",
    "last_heartbeat_s",
    "last_signal_gpio_s",
    "relative_altitude",
    "latitude",
    "longitude",
    "heading_deg",
  ].some((key) => key in candidate);
}

function extractDroneTelemetry(payload) {
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

function extractTrackingTelemetry(payload) {
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

function formatMetric(value, unit = "", digits = 2) {
  if (!Number.isFinite(Number(value))) {
    return "--";
  }

  return `${Number(value).toFixed(digits)}${unit}`;
}

function formatDegrees(value) {
  return formatMetric(value, " deg", 1);
}

function formatLatLon(value, axis) {
  if (!Number.isFinite(Number(value))) {
    return "--";
  }

  const positive = axis === "lat" ? "N" : "E";
  const negative = axis === "lat" ? "S" : "W";
  const suffix = Number(value) >= 0 ? positive : negative;
  return `${Math.abs(Number(value)).toFixed(5)} deg ${suffix}`;
}

function formatAgeFromSeconds(timestampSeconds) {
  if (!Number.isFinite(Number(timestampSeconds)) || Number(timestampSeconds) <= 0) {
    return "--";
  }

  const ageSeconds = Math.max(0, Date.now() / 1000 - Number(timestampSeconds));
  if (ageSeconds < 1) {
    return "now";
  }

  return `${ageSeconds.toFixed(1)} s ago`;
}

function formatFreshnessFromMs(ageMs) {
  if (!Number.isFinite(ageMs) || ageMs <= 0) {
    return "just now";
  }

  if (ageMs < 1000) {
    return `${Math.round(ageMs)} ms`;
  }

  return `${(ageMs / 1000).toFixed(1)} s`;
}

function gpsFixLabel(value) {
  const fixType = Number(value);
  if (!Number.isFinite(fixType)) {
    return "--";
  }

  const labels = {
    0: "NO GPS",
    1: "NO FIX",
    2: "2D",
    3: "3D",
    4: "DGPS",
    5: "RTK FLOAT",
    6: "RTK FIX",
  };

  return labels[fixType] ?? String(fixType);
}

function setText(node, value) {
  if (!node) {
    return;
  }

  node.textContent = value ?? "--";
}

function setLinkDots(isOnline) {
  const color = isOnline ? "#8cd65d" : "#ff7676";
  const shadow = isOnline ? "0 0 0 6px rgba(140, 214, 93, 0.12)" : "0 0 0 6px rgba(255, 118, 118, 0.12)";
  refs.headerLinkDot.style.background = color;
  refs.connectionDot.style.background = color;
  refs.headerLinkDot.style.boxShadow = shadow;
  refs.connectionDot.style.boxShadow = shadow;
}

function setStreamState(state, label) {
  appState.connectionState = state;
  setText(refs.statusText, label);
  setText(refs.streamStatePill, label);
  setText(refs.channelStateLabel, state);
  setText(refs.linkStateLabel, state === "connected" ? "Telemetry live" : "Offline link");
  setLinkDots(state === "connected");
}

function highlightMode(mode) {
  modeButtons.forEach((button) => {
    button.classList.toggle("mode-button-active", button.dataset.mode === mode);
  });
}

function updateCompass(headingDeg) {
  const safeHeading = Number.isFinite(Number(headingDeg)) ? ((Number(headingDeg) % 360) + 360) % 360 : null;
  const rotation = safeHeading ?? 0;
  refs.compassNeedle.style.transform = `translate(-50%, -50%) rotate(${rotation}deg)`;
  setText(refs.headingReadout, safeHeading === null ? "---" : `${String(Math.round(rotation)).padStart(3, "0")} deg`);
  setText(refs.headingChip, safeHeading === null ? "--" : `${Math.round(rotation)} deg`);
}

function updateMap(poseUav) {
  const x = Number(poseUav?.x);
  const y = Number(poseUav?.y);
  const maxOffset = 70;

  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    refs.mapDrone.style.transform = "translate(-50%, -50%)";
    return;
  }

  const offsetX = Math.max(-maxOffset, Math.min(maxOffset, x * 28));
  const offsetY = Math.max(-maxOffset, Math.min(maxOffset, y * 28));
  refs.mapDrone.style.transform = `translate(calc(-50% + ${offsetX}px), calc(-50% + ${offsetY}px))`;
}

function renderTracking(tracking) {
  const poseCamera = isPlainObject(tracking.poseCamera) ? tracking.poseCamera : null;
  const poseUav = isPlainObject(tracking.poseUav) ? tracking.poseUav : null;
  const attitude = isPlainObject(tracking.attitude) ? tracking.attitude : {};
  const hasTarget = Boolean(poseCamera) || tracking.markerId !== null;

  setText(refs.markerId, tracking.markerId === null ? "--" : String(tracking.markerId));
  setText(refs.trackingStatus, hasTarget ? "Locked" : "Searching");
  setText(refs.markerLockChip, hasTarget ? "Locked" : "Searching");
  setText(refs.targetStatePill, hasTarget ? "Target locked" : "Target lost");
  refs.targetStatePill.classList.toggle("status-pill-accent", hasTarget);

  setText(refs.poseX, formatMetric(poseCamera?.x, " m"));
  setText(refs.poseY, formatMetric(poseCamera?.y, " m"));
  setText(refs.poseZ, formatMetric(poseCamera?.z, " m"));
  setText(refs.uavPoseX, formatMetric(poseUav?.x, " m"));
  setText(refs.uavPoseY, formatMetric(poseUav?.y, " m"));
  setText(refs.uavPoseZ, formatMetric(poseUav?.z, " m"));

  setText(refs.attRoll, formatDegrees(attitude.roll ?? tracking.roll));
  setText(refs.attPitch, formatDegrees(attitude.pitch ?? tracking.pitch));
  setText(refs.attYaw, formatDegrees(attitude.yaw ?? tracking.yaw));
  setText(refs.cameraFramePill, poseCamera ? "Camera frame" : "No pose");

  updateMap(poseUav);
}

function renderDrone(drone) {
  const mode = typeof drone.mode === "string" ? drone.mode : "UNKNOWN";
  const batteryPct = Number(drone.battery_remaining_pct);
  const voltage = Number(drone.battery_voltage_v);
  const altitude = Number(drone.alt_m);
  const groundspeed = Number(drone.groundspeed_mps);
  const relativeAltitude = Number(drone.relative_altitude);
  const verticalSpeed = Number(drone.speed);
  const latitude = Number(drone.latitude);
  const longitude = Number(drone.longitude);
  const headingDeg = Number(drone.heading_deg);

  setText(refs.modeChip, mode);
  setText(refs.modeValue, mode);
  setText(refs.armedValue, typeof drone.armed === "boolean" ? (drone.armed ? "YES" : "NO") : "--");
  setText(refs.speedValue, formatMetric(groundspeed, " m/s", 1));
  setText(refs.altitudeValue, formatMetric(altitude, " m", 1));
  setText(refs.batteryValue, Number.isFinite(batteryPct) ? `${batteryPct}%` : "--");
  setText(refs.voltageValue, formatMetric(voltage, " V", 1));
  setText(refs.gpsValue, gpsFixLabel(drone.gps_fix_type));
  setText(refs.heartbeatValue, formatAgeFromSeconds(drone.last_heartbeat_s));
  setText(refs.signalValue, formatAgeFromSeconds(drone.last_signal_gpio_s));

  setText(refs.altitudeMain, formatMetric(altitude, " m", 1));
  setText(refs.speedMain, formatMetric(groundspeed, " m/s", 1));
  setText(
    refs.batteryMain,
    Number.isFinite(batteryPct) && Number.isFinite(voltage)
      ? `${batteryPct}% / ${voltage.toFixed(1)} V`
      : Number.isFinite(batteryPct)
        ? `${batteryPct}%`
        : "--"
  );
  setText(refs.gpsMain, gpsFixLabel(drone.gps_fix_type));
  setText(refs.latMain, formatLatLon(latitude, "lat"));
  setText(refs.lonMain, formatLatLon(longitude, "lon"));
  setText(refs.batteryChip, Number.isFinite(batteryPct) ? `${batteryPct}%` : "--");

  setText(refs.relativeAltPill, formatMetric(relativeAltitude, " m", 1));
  setText(refs.relativeAltValue, formatMetric(relativeAltitude, " m", 1));
  setText(refs.verticalSpeedValue, formatMetric(verticalSpeed, " m/s", 1));
  setText(
    refs.coordinatesValue,
    Number.isFinite(latitude) && Number.isFinite(longitude)
      ? `${formatLatLon(latitude, "lat")} / ${formatLatLon(longitude, "lon")}`
      : "--"
  );
  setText(refs.headingValue, formatDegrees(headingDeg));

  highlightMode(mode);
  updateCompass(headingDeg);
}

function renderPayload(payload) {
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
  refs.telemetry.textContent = JSON.stringify(payload, null, 2);
  refreshRuntimeLabels();
}

function refreshRuntimeLabels() {
  const hasTelemetry = appState.lastTelemetryAtMs > 0;
  const ageMs = hasTelemetry ? Date.now() - appState.lastTelemetryAtMs : NaN;
  const isFresh = hasTelemetry && ageMs < 3000;

  setText(refs.telemetryAge, hasTelemetry ? formatFreshnessFromMs(ageMs) : "No signal");
  setText(refs.lastUpdateLabel, hasTelemetry ? formatFreshnessFromMs(ageMs) : "never");

  if (appState.connectionState === "connected" && isFresh) {
    setText(refs.linkStateLabel, "Telemetry live");
  } else if (appState.connectionState === "connected") {
    setText(refs.linkStateLabel, "Waiting telemetry");
  }
}

async function start() {
  if (peerConnection) {
    return;
  }

  setStreamState("connecting", "Connecting");

  peerConnection = new RTCPeerConnection({
    iceServers: [{ urls: "stun:stun.l.google.com:19302" }],
  });

  peerConnection.addTransceiver("video", { direction: "recvonly" });

  peerConnection.onconnectionstatechange = () => {
    if (!peerConnection) {
      return;
    }

    const state = peerConnection.connectionState;
    if (state === "connected") {
      setStreamState("connected", "Connected");
    } else if (state === "connecting") {
      setStreamState("connecting", "Connecting");
    } else if (state === "failed" || state === "disconnected" || state === "closed") {
      setStreamState(state, state.charAt(0).toUpperCase() + state.slice(1));
    }
  };

  dataChannel = peerConnection.createDataChannel("telemetry");
  dataChannel.onopen = () => setText(refs.channelStateLabel, "open");
  dataChannel.onclose = () => setText(refs.channelStateLabel, "closed");
  dataChannel.onerror = () => setText(refs.channelStateLabel, "error");
  dataChannel.onmessage = (event) => {
    try {
      renderPayload(JSON.parse(event.data));
    } catch {
      refs.telemetry.textContent = String(event.data);
    }
  };

  peerConnection.ontrack = (event) => {
    refs.video.srcObject = event.streams[0];
    refs.videoStage.classList.add("video-live");
  };

  try {
    const offer = await peerConnection.createOffer();
    await peerConnection.setLocalDescription(offer);

    const response = await fetch("/offer", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        sdp: peerConnection.localDescription.sdp,
        type: peerConnection.localDescription.type,
      }),
    });

    const answer = await response.json();
    await peerConnection.setRemoteDescription(answer);
    setText(refs.channelStateLabel, "negotiated");
  } catch (error) {
    console.error(error);
    setStreamState("error", "Connection failed");
  }
}

async function stop() {
  if (!peerConnection) {
    setStreamState("stopped", "Stopped");
    return;
  }

  try {
    if (dataChannel) {
      dataChannel.close();
    }
  } catch (error) {
    console.error(error);
  }

  try {
    await peerConnection.close();
  } catch (error) {
    console.error(error);
  }

  peerConnection = null;
  dataChannel = null;
  refs.video.srcObject = null;
  refs.videoStage.classList.remove("video-live");
  setStreamState("stopped", "Stopped");
}

refs.startButton.addEventListener("click", start);
refs.stopButton.addEventListener("click", stop);
window.addEventListener("beforeunload", stop);
window.setInterval(refreshRuntimeLabels, 1000);

highlightMode("UNKNOWN");
updateCompass(null);
renderTracking({});
renderDrone({});
setStreamState("stopped", "Stopped");
