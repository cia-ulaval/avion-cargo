import asyncio
import json
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Set

import numpy as np
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.mediastreams import VideoStreamTrack
from av import VideoFrame

from domain.camera import LastestFrameBuffer
from domain.content_diffuser import ContentDiffuser

INDEX_HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>WebRTC Viewer</title>
  <style>
    body { font-family: sans-serif; margin: 16px; }
    video { width: 100%; max-width: 960px; background: #111; border-radius: 12px; border-radius: 12px; }
    pre { max-width: 960px; background: #f6f6f6; padding: 12px; border-radius: 12px; }
    .row { margin-top: 12px; display: flex; gap: 12px; align-items: center; flex-wrap: wrap; }
    button { padding: 10px 14px; border-radius: 10px; border: 1px solid #ccc; cursor: pointer; }
  </style>
</head>
<body>
  <h2>WebRTC stream</h2>

  <video id="v" autoplay playsinline controls muted></video>

  <div class="row">
    <button onclick="start()">Start</button>
    <button onclick="stop()">Stop</button>
    <span id="st"></span>
  </div>

  <h3>Telemetry (DataChannel)</h3>
  <pre id="telemetry">(no data)</pre>

<script>
let pc = null;
let dc = null;

async function start() {
  if (pc) return;

  document.getElementById("st").textContent = "connecting...";
  pc = new RTCPeerConnection({
    iceServers: [{urls: "stun:stun.l.google.com:19302"}]
  });

  pc.addTransceiver("video", { direction: "recvonly" });

  dc = pc.createDataChannel("telemetry");
  dc.onmessage = (ev) => {
    try {
      const obj = JSON.parse(ev.data);
      document.getElementById("telemetry").textContent = JSON.stringify(obj, null, 2);
    } catch {
      document.getElementById("telemetry").textContent = String(ev.data);
    }
  };

  pc.ontrack = (ev) => {
    const v = document.getElementById("v");
    v.srcObject = ev.streams[0];
  };

  const offer = await pc.createOffer();
  await pc.setLocalDescription(offer);

  const r = await fetch("/offer", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({sdp: pc.localDescription.sdp, type: pc.localDescription.type})
  });

  const ans = await r.json();
  await pc.setRemoteDescription(ans);

  document.getElementById("st").textContent = "connected";
}

async function stop() {
  if (!pc) return;
  try { dc && dc.close(); } catch {}
  try { await pc.close(); } catch {}
  pc = null;
  dc = null;
  document.getElementById("st").textContent = "stopped";
}
</script>
</body>
</html>
"""


class _BufferVideoTrack(VideoStreamTrack):
    """
    Track aiortc: lit la dernière frame dans LastestFrameBuffer.
    IMPORTANT: aucun OpenCV ici, seulement lecture buffer + pacing.
    """

    def __init__(self, frame_buffer: LastestFrameBuffer, target_fps: int, fallback_hw=(480, 640)):
        super().__init__()
        self._buf = frame_buffer
        self._period = 1.0 / max(1, int(target_fps))
        self._last = 0.0
        self._fh = int(fallback_hw[0])
        self._fw = int(fallback_hw[1])

    async def recv(self) -> VideoFrame:
        now = time.time()
        dt = now - self._last
        if dt < self._period:
            await asyncio.sleep(self._period - dt)
        self._last = time.time()

        frame, _meta = self._buf.get_copy()
        if frame is None:
            img = np.zeros((self._fh, self._fw, 3), dtype=np.uint8)
            vf = VideoFrame.from_ndarray(img, format="bgr24")
        else:
            # Ici on suppose que la frame dans le buffer est BGR (cohérent OpenCV).
            # Si tu stockes RGB, change format/cvtColor.
            vf = VideoFrame.from_ndarray(frame, format="bgr24")

        vf.pts, vf.time_base = await self.next_timestamp()
        return vf


@dataclass(slots=True)
class WebRTCConfig:
    host: str = "0.0.0.0"
    port: int = 8080
    stream_fps: int = 30


class WebRTCContentDiffuser(ContentDiffuser):
    """
    Implémentation WebRTC qui réutilise ton LastestFrameBuffer.
    - diffuse_video(): démarre un serveur aiohttp + aiortc (bloquant, "script style")
    - diffuse_data(): envoie via DataChannel (thread-safe)
    """

    def __init__(self, frame_buffer: LastestFrameBuffer, config: WebRTCConfig = WebRTCConfig()):
        self._buf = frame_buffer
        self._cfg = config

        self._pcs: Set[RTCPeerConnection] = set()
        self._telemetry_channels: Set[Any] = set()  # RTCDataChannel

        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def diffuse_data(self, data: Dict[str, Any]) -> None:
        # Peut être appelé depuis ton thread pipeline
        if self._loop is None:
            return
        payload = json.dumps(data, separators=(",", ":"), ensure_ascii=False)
        self._loop.call_soon_threadsafe(self._send_telemetry_now, payload)

    def diffuse_video(self) -> None:
        """
        Mode "script": bloque le thread courant (comme web.run_app).
        Le pipeline caméra tourne dans son thread séparé.
        """
        asyncio.run(self._run_server())

    async def _run_server(self) -> None:
        self._loop = asyncio.get_running_loop()

        app = web.Application()
        app["diffuser"] = self
        app.router.add_get("/", self._index)
        app.router.add_post("/offer", self._offer)
        app.on_shutdown.append(self._on_shutdown)

        print(f"[WebRTC] Open http://<PI_IP>:{self._cfg.port}/")
        await web._run_app(app, host=self._cfg.host, port=self._cfg.port)  # internal helper used by web.run_app

    async def _index(self, _request: web.Request) -> web.Response:
        return web.Response(text=INDEX_HTML, content_type="text/html")

    async def _offer(self, request: web.Request) -> web.Response:
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        pc = RTCPeerConnection()
        self._pcs.add(pc)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            if pc.connectionState in ("failed", "closed", "disconnected"):
                await pc.close()
                self._pcs.discard(pc)

        # Le client crée le DataChannel telemetry
        @pc.on("datachannel")
        def on_datachannel(channel):
            self._telemetry_channels.add(channel)

            @channel.on("close")
            def on_close():
                self._telemetry_channels.discard(channel)

        await pc.setRemoteDescription(offer)

        # vidéo depuis buffer
        pc.addTrack(_BufferVideoTrack(self._buf, target_fps=self._cfg.stream_fps))

        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}),
        )

    async def _on_shutdown(self, _app: web.Application) -> None:
        await self._close_all_peers()
        self._loop = None

    async def _close_all_peers(self) -> None:
        coros = [pc.close() for pc in list(self._pcs)]
        if coros:
            await asyncio.gather(*coros, return_exceptions=True)
        self._pcs.clear()
        self._telemetry_channels.clear()

    def _send_telemetry_now(self, payload: str) -> None:
        dead = []
        for ch in self._telemetry_channels:
            try:
                ch.send(payload)
            except Exception:
                dead.append(ch)
        for ch in dead:
            self._telemetry_channels.discard(ch)
