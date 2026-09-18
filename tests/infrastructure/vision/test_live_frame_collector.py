from unittest.mock import Mock

import pytest

from infrastructure.vision.live_frame_collector import LiveFrameCollector, LiveFrameCollectorConfig


def test_capture_failure_always_releases_camera_and_window(monkeypatch):
    camera = Mock()
    camera.get_frame.side_effect = OSError("camera failed")
    destroy = Mock()
    monkeypatch.setattr("infrastructure.vision.live_frame_collector.cv2.destroyAllWindows", destroy)
    collector = LiveFrameCollector(camera, Mock(), Mock(), LiveFrameCollectorConfig())
    with pytest.raises(OSError, match="camera failed"):
        collector.collect()
    camera.close.assert_called_once()
    destroy.assert_called_once()
