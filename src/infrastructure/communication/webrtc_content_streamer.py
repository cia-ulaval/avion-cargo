import asyncio
import json
import hmac
import ssl
import time
from dataclasses import dataclass
from pathlib import Path
from threading import Event
from typing import Any, Dict, Optional, Set

import numpy as np
from aiohttp import BasicAuth, web
from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription
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
        now = time.monotonic()
        dt = now - self._last
        if dt < self._period:
            await asyncio.sleep(self._period - dt)
        self._last = time.monotonic()

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
    host: str = "127.0.0.1"
    port: int = 8080
    stream_fps: int = 30
    password: str | None = None
    tls_cert: str | None = None
    tls_key: str | None = None
    max_peers: int = 4


SRC_ASSETS_BASE_DIR = Path(__file__).resolve().parent.parent.parent / "assets"
SRC_ASSETS_STATIC_DIR = SRC_ASSETS_BASE_DIR / "static"
INDEX_HTML_FILE = SRC_ASSETS_BASE_DIR / "index.html"


class WebRTCContentStreamer(ContentStreamer):
    def __init__(self, frame_buffer: FrameBuffer, config: WebRTCConfig | None = None):
        self.buffer = frame_buffer
        self.configuration = config if config is not None else WebRTCConfig()

        self.peer_connections: Set[RTCPeerConnection] = set()
        self.telemetry_channels: Set[Any] = set()

        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._stop_requested = Event()
        self._shutdown_event: asyncio.Event | None = None
        self._async_error: BaseException | None = None

    def send_data(self, data: Dict[str, Any]) -> None:
        loop = self._loop
        if loop is None or self._stop_requested.is_set():
            return
        payload = json.dumps(data, separators=(",", ":"), ensure_ascii=False)
        try:
            loop.call_soon_threadsafe(self._send_telemetry, payload)
        except RuntimeError:
            if not self._stop_requested.is_set():
                raise

    def stream_video(self) -> None:
        if self._stop_requested.is_set():
            return
        asyncio.run(self._run_server())

    def stop(self) -> None:
        self._stop_requested.set()
        loop, shutdown_event = self._loop, self._shutdown_event
        if loop is not None and shutdown_event is not None:
            try:
                loop.call_soon_threadsafe(shutdown_event.set)
            except RuntimeError:
                # The event loop may have finished between lookup and notification.
                pass

    def _handle_async_error(self, _loop: asyncio.AbstractEventLoop, context: dict[str, Any]) -> None:
        error = context.get("exception") or RuntimeError(context.get("message", "Unknown asyncio failure"))
        logger.opt(exception=error).error("Unhandled WebRTC asynchronous error")
        self._async_error = error
        self.stop()

    async def _run_server(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._shutdown_event = asyncio.Event()
        self._loop.set_exception_handler(self._handle_async_error)

        app = web.Application(middlewares=[self._authenticate])
        app.router.add_static("/static/", path=SRC_ASSETS_STATIC_DIR, name="static")
        app.router.add_get("/", self._index)
        app.router.add_post("/offer", self._offer)
        app.on_shutdown.append(self._shutdown)

        runner = web.AppRunner(app, shutdown_timeout=2)
        try:
            if self._stop_requested.is_set():
                return
            await runner.setup()
            tls = None
            if bool(self.configuration.tls_cert) != bool(self.configuration.tls_key):
                raise ValueError("Both TLS certificate and key must be provided")
            if self.configuration.tls_cert:
                tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
                tls.load_cert_chain(self.configuration.tls_cert, self.configuration.tls_key)
            site = web.TCPSite(runner, host=self.configuration.host, port=self.configuration.port, ssl_context=tls)
            await site.start()
            logger.info("WebRTC server ready at {}://{}:{}", "https" if tls else "http", self.configuration.host,
                        self.configuration.port)
            await self._shutdown_event.wait()
            if self._async_error is not None:
                raise RuntimeError("WebRTC asynchronous task failed") from self._async_error
        finally:
            self._stop_requested.set()
            try:
                await asyncio.wait_for(runner.cleanup(), timeout=3)
            finally:
                self._loop = None
                self._shutdown_event = None
                logger.info("WebRTC server stopped")

    @staticmethod
    async def _index(_request: web.Request) -> web.FileResponse:
        return web.FileResponse(path=INDEX_HTML_FILE)

    @web.middleware
    async def _authenticate(self, request, handler):
        if self.configuration.password:
            try:
                auth = BasicAuth.decode(request.headers.get("Authorization", ""))
                valid = auth.login == "autolander" and hmac.compare_digest(
                    auth.password.encode(), self.configuration.password.encode()
                )
            except ValueError:
                valid = False
            if not valid:
                raise web.HTTPUnauthorized(headers={"WWW-Authenticate": 'Basic realm="Autolander"'})
        return await handler(request)

    async def _offer(self, request: web.Request) -> web.Response:
        try:
            params = await request.json()
            if not isinstance(params, dict) or params.get("type") != "offer" or not isinstance(params.get("sdp"), str):
                raise ValueError("Expected an SDP offer")
            offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        except (ValueError, TypeError) as error:
            raise web.HTTPBadRequest(text="Invalid WebRTC offer") from error

        if len(self.peer_connections) >= self.configuration.max_peers:
            raise web.HTTPServiceUnavailable(text="Too many WebRTC sessions")
        peer_connection = RTCPeerConnection(RTCConfiguration(iceServers=[]))
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

        try:
            await peer_connection.setRemoteDescription(offer)
            peer_connection.addTrack(_VideoBufferTrack(self.buffer, target_fps=self.configuration.stream_fps))
            answer = await peer_connection.createAnswer()
            await peer_connection.setLocalDescription(answer)
        except Exception as error:
            logger.opt(exception=error).warning("WebRTC negotiation failed")
            await peer_connection.close()
            self.peer_connections.discard(peer_connection)
            raise web.HTTPBadRequest(text="WebRTC negotiation failed") from error

        logger.info("WebRTC session negotiated ({} peers)", len(self.peer_connections))

        return web.Response(
            content_type="application/json",
            text=json.dumps(
                {"sdp": peer_connection.localDescription.sdp, "type": peer_connection.localDescription.type}
            ),
        )

    async def _shutdown(self, _app: web.Application) -> None:
        await self._close_all_peers()

    async def _close_all_peers(self) -> None:
        coroutines = [pc.close() for pc in list(self.peer_connections)]
        if coroutines:
            results = await asyncio.gather(*coroutines, return_exceptions=True)
            for result in results:
                if isinstance(result, BaseException):
                    logger.opt(exception=result).error("Failed to close a WebRTC peer")

        self.peer_connections.clear()
        self.telemetry_channels.clear()

    def _send_telemetry(self, payload: str) -> None:
        dead_channels = []
        for channel in self.telemetry_channels:
            if channel.readyState != "open":
                continue
            try:
                channel.send(payload)
            except Exception as e:
                dead_channels.append(channel)
                logger.error(f"Failed to send telemetry: {e}")

        for channel in dead_channels:
            self.telemetry_channels.discard(channel)
