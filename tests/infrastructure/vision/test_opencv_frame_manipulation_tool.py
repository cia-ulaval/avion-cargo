import numpy as np
import pytest

from infrastructure.vision.opencv_frame_manipution_tool import AxisStyle, FrameManipulationTool


def draw_axes(frame, translation, *, rotation=(0.0, 0.0, 0.0), copy=False):
    return FrameManipulationTool.draw_axes_for_poses(
        frame,
        np.float64([[200, 0, 160], [0, 200, 120], [0, 0, 1]]),
        np.zeros(5),
        np.float64(rotation).reshape(1, 1, 3),
        np.float64(translation).reshape(1, 1, 3),
        axis_style=AxisStyle(axis_length_m=0.1),
        copy=copy,
    )


def test_axes_outside_the_image_are_clipped_without_opencv_warnings(capfd):
    frame = np.zeros((240, 320, 3), dtype=np.uint8)

    result = draw_axes(frame, (0.75, 0.0, 1.0))

    assert result is frame
    np.testing.assert_array_equal(result[120, 319], [0, 0, 255])
    np.testing.assert_array_equal(result[140, 310], [0, 255, 0])
    assert "drawFrameAxes" not in capfd.readouterr().err


def test_axis_with_origin_outside_the_image_keeps_its_visible_segment(capfd):
    frame = np.zeros((240, 320, 3), dtype=np.uint8)

    result = draw_axes(frame, (-0.85, 0.0, 1.0))

    np.testing.assert_array_equal(result[120, 10], [0, 0, 255])
    assert "drawFrameAxes" not in capfd.readouterr().err


def test_visible_axes_preserve_colours_and_copy_semantics():
    frame = np.zeros((240, 320, 3), dtype=np.uint8)

    result = draw_axes(frame, (0.0, 0.0, 1.0), copy=True)

    assert result is not frame and not np.any(frame)
    np.testing.assert_array_equal(result[120, 180], [0, 0, 255])
    np.testing.assert_array_equal(result[140, 160], [0, 255, 0])
    np.testing.assert_array_equal(result[120, 160], [255, 0, 0])


@pytest.mark.parametrize("translation", [(0.0, 0.0, -1.0), (np.nan, 0.0, 1.0), (0.0, 0.0, 0.0)])
def test_invalid_or_behind_camera_origin_is_not_drawn(translation):
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    assert not np.any(draw_axes(frame, translation))


def test_axis_endpoint_behind_the_camera_is_skipped():
    frame = np.zeros((240, 320, 3), dtype=np.uint8)

    result = draw_axes(frame, (0.01, 0.0, 0.05), rotation=(np.pi, 0.0, 0.0))

    assert np.any(result[:, :, 1:])
    assert not np.any(result[:, :, 0])
