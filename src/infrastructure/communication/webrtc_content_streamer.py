import asyncio
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Set

import numpy as np
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.mediastreams import VideoStreamTrack
from av import VideoFrame
from loguru import logger

from domain.content_streamer import ContentStreamer
from infrastructure.camera.frame_buffer import FrameBuffer


class _VideoBufferTrack(VideoStreamTrack):
    def __init__(self, frame_buffer: FrameBuffer, target_fps: int, fallback_hw=(480, 640)):
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

        frame, _meta = self._buf.get_value()
        if frame is None:
            img = np.zeros((self._fh, self._fw, 3), dtype=np.uint8)
            vf = VideoFrame.from_ndarray(img, format="bgr24")
        else:
            vf = VideoFrame.from_ndarray(frame, format="bgr24")

        vf.pts, vf.time_base = await self.next_timestamp()
        return vf


@dataclass(slots=True)
class WebRTCConfig:
    host: str = "0.0.0.0"
    port: int = 8080
    stream_fps: int = 30

SRC_ASSETS_BASE_DIR = Path(__file__).resolve().parent.parent.parent / "assets"
SRC_ASSETS_STATIC_DIR = SRC_ASSETS_BASE_DIR / "static"
INDEX_HTML_FILE = SRC_ASSETS_BASE_DIR / "index.html"

class WebRTCContentStreamer(ContentStreamer):
    def __init__(self, frame_buffer: FrameBuffer, config: WebRTCConfig = WebRTCConfig()):
        self.buffer = frame_buffer
        self.configuration = config

        self.peer_connections: Set[RTCPeerConnection] = set()
        self.telemetry_channels: Set[Any] = set()

        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def send_data(self, data: Dict[str, Any]) -> None:
        if self._loop is None:
            return
        payload = json.dumps(data, separators=(",", ":"), ensure_ascii=False)
        self._loop.call_soon_threadsafe(self._send_telemetry, payload)

    def stream_video(self) -> None:
        asyncio.run(self._run_server())

    async def _run_server(self) -> None:
        self._loop = asyncio.get_running_loop()

        app = web.Application()
        app["contentStreamer"] = self
        app.router.add_static("/static/", path=SRC_ASSETS_STATIC_DIR, name="static")
        app.router.add_get("/", self._index)
        app.router.add_post("/offer", self._offer)
        app.on_shutdown.append(self._shutdown)

        logger.info(f"Starting WebRTCContentStreamer on : http://{self.configuration.host}:{self.configuration.port}")
        await web._run_app(app, host=self.configuration.host, port=self.configuration.port)

    @staticmethod
    async def _index(_request: web.Request) -> web.FileResponse:
        return web.FileResponse(path=INDEX_HTML_FILE)

    async def _offer(self, request: web.Request) -> web.Response:
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        peer_connection = RTCPeerConnection()
        self.peer_connections.add(peer_connection)

        @peer_connection.on("connectionstatechange")
        async def on_connectionstatechange():
            if peer_connection.connectionState in ("failed", "closed", "disconnected"):
                await peer_connection.close()
                self.peer_connections.discard(peer_connection)

        @peer_connection.on("datachannel")
        def on_datachannel(channel):
            self.telemetry_channels.add(channel)

            @channel.on("close")
            def on_close():
                self.telemetry_channels.discard(channel)

        await peer_connection.setRemoteDescription(offer)

        peer_connection.addTrack(_VideoBufferTrack(self.buffer, target_fps=self.configuration.stream_fps))

        answer = await peer_connection.createAnswer()
        await peer_connection.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps(
                {"sdp": peer_connection.localDescription.sdp, "type": peer_connection.localDescription.type}
            ),
        )

    async def _shutdown(self, _app: web.Application) -> None:
        await self._close_all_peers()
        self._loop = None

    async def _close_all_peers(self) -> None:
        coroutines = [pc.close() for pc in list(self.peer_connections)]
        if coroutines:
            await asyncio.gather(*coroutines, return_exceptions=True)

        self.peer_connections.clear()
        self.telemetry_channels.clear()

    def _send_telemetry(self, payload: str) -> None:
        dead_channels = []
        for channel in self.telemetry_channels:
            try:
                channel.send(payload)
            except Exception as e:
                dead_channels.append(channel)
                logger.error(f"Failed to send telemetry: {e}")

        for channel in dead_channels:
            self.telemetry_channels.discard(channel)
