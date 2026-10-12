from unittest.mock import Mock

import numpy as np

import composition
from domain.models import CalibrationData, TargetedMarker


def test_tracking_builder_selects_faruco_adapter_for_faruco_targets(monkeypatch) -> None:
    target = TargetedMarker(
        id=0,
        length=0.896,
        marker_type="faruco",
        fractal_type="FRACTAL_3L_6",
    )
    calibration = CalibrationData(
        camera_matrix=np.array([[500.0, 0.0, 200.0], [0.0, 500.0, 200.0], [0.0, 0.0, 1.0]]),
        dist_coeffs=np.zeros(5),
        camera_width=400,
        camera_height=400,
    )
    detector = Mock(name="faruco-detector")
    faruco_factory = Mock(return_value=detector)
    monkeypatch.setattr(composition, "FArUcoDetector", faruco_factory)

    service = composition.build_tracking_service(
        camera=Mock(),
        target=target,
        detector_config=None,
        calibration_data=calibration,
    )

    assert service.detector is detector
    faruco_factory.assert_called_once_with(target, calibration)
