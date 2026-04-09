import { refs } from './dom.js';
import { dataChannel, peerConnection, setDataChannel, setPeerConnection } from './state.js';
import { renderPayload, setStreamState, setText } from './ui.js';

export async function start() {
  if (peerConnection) {
    return;
  }

  setStreamState('connecting', 'Connecting');

  const connection = new RTCPeerConnection({
    iceServers: [{ urls: 'stun:stun.l.google.com:19302' }],
  });
  setPeerConnection(connection);

  connection.addTransceiver('video', { direction: 'recvonly' });

  connection.onconnectionstatechange = () => {
    if (!peerConnection) {
      return;
    }

    const state = peerConnection.connectionState;
    if (state === 'connected') {
      setStreamState('connected', 'Connected');
    } else if (state === 'connecting') {
      setStreamState('connecting', 'Connecting');
    } else if (state === 'failed' || state === 'disconnected' || state === 'closed') {
      setStreamState(state, state.charAt(0).toUpperCase() + state.slice(1));
    }
  };

  const channel = connection.createDataChannel('telemetry');
  setDataChannel(channel);
  channel.onopen = () => setText(refs.channelStateLabel, 'open');
  channel.onclose = () => setText(refs.channelStateLabel, 'closed');
  channel.onerror = () => setText(refs.channelStateLabel, 'error');
  channel.onmessage = (event) => {
    try {
      renderPayload(JSON.parse(event.data));
    } catch {
      refs.telemetry.textContent = String(event.data);
    }
  };

  connection.ontrack = (event) => {
    refs.video.removeAttribute('controls');
    refs.video.controls = false;
    refs.video.srcObject = event.streams[0];
    refs.videoStage.classList.add('video-live');
  };

  try {
    const offer = await connection.createOffer();
    await connection.setLocalDescription(offer);

    const response = await fetch('/offer', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        sdp: connection.localDescription.sdp,
        type: connection.localDescription.type,
      }),
    });

    const answer = await response.json();
    await connection.setRemoteDescription(answer);
    setText(refs.channelStateLabel, 'negotiated');
  } catch (error) {
    console.error(error);
    setStreamState('error', 'Connection failed');
  }
}

export async function stop() {
  if (!peerConnection) {
    setStreamState('stopped', 'Stopped');
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

  setPeerConnection(null);
  setDataChannel(null);
  refs.video.srcObject = null;
  refs.videoStage.classList.remove('video-live');
  setStreamState('stopped', 'Stopped');
}
