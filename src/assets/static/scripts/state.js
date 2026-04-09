export let peerConnection = null;
export let dataChannel = null;

export const appState = {
  lastPayload: null,
  lastTelemetryAtMs: 0,
  lastDrone: {},
  lastTracking: {},
  connectionState: 'stopped',
};

export function setPeerConnection(value) {
  peerConnection = value;
}

export function setDataChannel(value) {
  dataChannel = value;
}
