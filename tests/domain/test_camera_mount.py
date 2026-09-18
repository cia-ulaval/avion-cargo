import pytest

from domain.camera_mount import CameraMount
from domain.models import Pose3D


def test_default_mount_preserves_historical_downward_axes():
    assert CameraMount().to_body(Pose3D(1, 2, 3)) == Pose3D(-2, 1, 3)


def test_tilted_camera_and_lever_arm_transform_target_into_vehicle_frame():
    mount = CameraMount(rotation=((0, 0, 1), (0, 1, 0), (-1, 0, 0)), translation_m=(.1, .2, .3))
    assert mount.to_body(Pose3D(1, 2, 3)) == Pose3D(3.1, 2.2, -.7)


@pytest.mark.parametrize('rotation', [((1, 0, 0), (0, 1, 0), (0, 0, -1)), ((2, 0, 0), (0, 1, 0), (0, 0, 1))])
def test_non_rigid_mount_is_rejected(rotation):
    with pytest.raises(ValueError, match='orthonormal'):
        CameraMount(rotation=rotation)
