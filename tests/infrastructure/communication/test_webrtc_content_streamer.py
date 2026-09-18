import asyncio
from threading import Event, Thread
from unittest.mock import AsyncMock, Mock

import pytest
from aiohttp import web

from infrastructure.camera.frame_buffer import FrameBuffer
from infrastructure.communication import webrtc_content_streamer as streamer_module


def fake_server(monkeypatch, *, start=None):
    runner = Mock(setup=AsyncMock(), cleanup=AsyncMock())
    site = Mock(start=start or AsyncMock())
    monkeypatch.setattr(streamer_module.web, "AppRunner", Mock(return_value=runner))
    monkeypatch.setattr(streamer_module.web, "TCPSite", Mock(return_value=site))
    return runner


def test_stop_wakes_server_loop_from_another_thread_and_cleans_up(monkeypatch):
    started = Event()

    async def start():
        started.set()

    runner = fake_server(monkeypatch, start=start)
    streamer = streamer_module.WebRTCContentStreamer(FrameBuffer())
    thread = Thread(target=streamer.stream_video, daemon=True)
    thread.start()
    try:
        assert started.wait(2), "WebRTC loop did not start"
    finally:
        streamer.stop()
        thread.join(timeout=2)

    assert not thread.is_alive()
    runner.cleanup.assert_awaited_once()
    assert streamer._loop is None
    streamer.stop()
    streamer.send_data({"after_shutdown": True})


def test_stop_before_start_prevents_late_server_start(monkeypatch):
    runner = fake_server(monkeypatch)
    streamer = streamer_module.WebRTCContentStreamer(FrameBuffer())
    streamer.stop()
    streamer.stream_video()
    runner.setup.assert_not_called()


def test_http_startup_failure_is_propagated_and_runner_is_cleaned(monkeypatch):
    runner = fake_server(monkeypatch, start=AsyncMock(side_effect=OSError("address already in use")))
    streamer = streamer_module.WebRTCContentStreamer(FrameBuffer())
    with pytest.raises(OSError, match="address already in use"):
        streamer.stream_video()
    runner.cleanup.assert_awaited_once()
    assert streamer._loop is None


def test_unhandled_async_error_stops_server_and_reaches_caller(monkeypatch):
    original_error = RuntimeError("background task failed")

    async def start():
        asyncio.get_running_loop().call_exception_handler({"exception": original_error})

    runner = fake_server(monkeypatch, start=start)
    streamer = streamer_module.WebRTCContentStreamer(FrameBuffer())
    with pytest.raises(RuntimeError, match="asynchronous task failed") as failure:
        streamer.stream_video()
    assert failure.value.__cause__ is original_error
    runner.cleanup.assert_awaited_once()


@pytest.mark.parametrize("params", [{}, [], {"sdp": 4, "type": "offer"}, {"sdp": "bad", "type": "answer"}])
def test_malformed_offer_returns_bad_request(params):
    streamer = streamer_module.WebRTCContentStreamer(FrameBuffer())
    request = Mock(json=AsyncMock(return_value=params))
    with pytest.raises(web.HTTPBadRequest):
        asyncio.run(streamer._offer(request))
    assert not streamer.peer_connections


def test_failed_offer_closes_and_removes_its_peer(monkeypatch):
    peer = Mock(setRemoteDescription=AsyncMock(side_effect=ValueError("invalid SDP")), close=AsyncMock())
    monkeypatch.setattr(streamer_module, "RTCPeerConnection", Mock(return_value=peer))
    streamer = streamer_module.WebRTCContentStreamer(FrameBuffer())
    request = Mock(json=AsyncMock(return_value={"sdp": "invalid", "type": "offer"}))
    with pytest.raises(web.HTTPBadRequest):
        asyncio.run(streamer._offer(request))
    peer.close.assert_awaited_once()
    assert not streamer.peer_connections


@pytest.mark.parametrize("authorized", [True, False])
def test_http_password_protects_page_assets_and_offer(authorized):
    from aiohttp import BasicAuth

    config = streamer_module.WebRTCConfig(password="test-password")
    streamer = streamer_module.WebRTCContentStreamer(FrameBuffer(), config)
    request = Mock(
        headers={"Authorization": BasicAuth("autolander", "test-password" if authorized else "wrong").encode()}
    )
    handler = AsyncMock(return_value="page")
    if authorized:
        assert asyncio.run(streamer._authenticate(request, handler)) == "page"
    else:
        with pytest.raises(web.HTTPUnauthorized):
            asyncio.run(streamer._authenticate(request, handler))
        handler.assert_not_called()


def test_sessions_are_bounded_before_creating_peer():
    streamer = streamer_module.WebRTCContentStreamer(FrameBuffer())
    streamer.peer_connections = {object() for _ in range(streamer.configuration.max_peers)}
    request = Mock(json=AsyncMock(return_value={"sdp": "test", "type": "offer"}))
    with pytest.raises(web.HTTPServiceUnavailable):
        asyncio.run(streamer._offer(request))
