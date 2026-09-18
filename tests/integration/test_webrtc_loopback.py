"""Exercise real HTTP, SDP, video, data channel and cleanup over loopback."""

import asyncio

import numpy as np
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription

from infrastructure.camera.frame_buffer import FrameBuffer
from infrastructure.communication.webrtc_content_streamer import WebRTCContentStreamer


def test_real_webrtc_session_delivers_video_and_telemetry_and_closes():
    async def scenario():
        buffer = FrameBuffer(max_age_s=10)
        buffer.set_value(np.full((48, 64, 3), 100, np.uint8))
        streamer = WebRTCContentStreamer(buffer)
        app = web.Application()
        app.router.add_get("/", streamer._index)
        app.router.add_post("/offer", streamer._offer)
        app.on_shutdown.append(streamer._shutdown)
        client = TestClient(TestServer(app))
        peer = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        video_received = asyncio.get_running_loop().create_future()
        data_received = asyncio.get_running_loop().create_future()
        channel_open = asyncio.Event()
        try:
            await client.start_server()
            response = await client.get("/")
            assert response.status == 200
            assert "AUTOLANDER" in (await response.text()).upper()
            peer.addTransceiver("video", direction="recvonly")
            channel = peer.createDataChannel("telemetry")
            channel.on("open", channel_open.set)
            channel.on(
                "message", lambda payload: data_received.set_result(payload) if not data_received.done() else None
            )

            @peer.on("track")
            async def receive_video(track):
                try:
                    frame = await track.recv()
                    if not video_received.done():
                        video_received.set_result((frame.width, frame.height))
                except Exception as error:
                    if not video_received.done():
                        video_received.set_exception(error)

            await peer.setLocalDescription(await peer.createOffer())
            response = await client.post("/offer", json={"sdp": peer.localDescription.sdp, "type": "offer"})
            assert response.status == 200
            answer = await response.json()
            await peer.setRemoteDescription(RTCSessionDescription(**answer))
            await asyncio.wait_for(channel_open.wait(), 5)
            streamer._send_telemetry('{"marker_id":29}')
            assert await asyncio.wait_for(data_received, 5) == '{"marker_id":29}'
            assert await asyncio.wait_for(video_received, 5) == (64, 48)
        finally:
            await peer.close()
            await client.close()
        assert not streamer.peer_connections
        assert not streamer.telemetry_channels

    asyncio.run(scenario())
