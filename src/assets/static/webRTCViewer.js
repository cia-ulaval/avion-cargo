let peerConnection = null;
let dataChannel = null;

async function start() {
  if (peerConnection) return;

  document.getElementById("st").textContent = "connecting...";
  peerConnection = new RTCPeerConnection({
    iceServers: [{urls: "stun:stun.l.google.com:19302"}]
  });

  peerConnection.addTransceiver("video", { direction: "recvonly" });

  dataChannel = peerConnection.createDataChannel("telemetry");
  dataChannel.onmessage = (ev) => {
    try {
      const obj = JSON.parse(ev.data);
      document.getElementById("telemetry").textContent = JSON.stringify(obj, null, 2);
    } catch {
      document.getElementById("telemetry").textContent = String(ev.data);
    }
  };

  peerConnection.ontrack = (ev) => {
    const v = document.getElementById("v");
    v.srcObject = ev.streams[0];
  };

  const offer = await peerConnection.createOffer();
  await peerConnection.setLocalDescription(offer);

  const r = await fetch("/offer", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({sdp: peerConnection.localDescription.sdp, type: peerConnection.localDescription.type})
  });

  const ans = await r.json();
  await peerConnection.setRemoteDescription(ans);

  document.getElementById("st").textContent = "connected";
}

async function stop() {
  if (!peerConnection) return;
  try { dataChannel && dataChannel.close(); } catch {}
  try { await peerConnection.close(); } catch {}
  peerConnection = null;
  dataChannel = null;
  document.getElementById("st").textContent = "stopped";
}