from dataclasses import dataclass

import numpy as np

from vtol.domain.models import Pose3D
from vtol.domain.rigid_transform import RigidTransform


@dataclass(frozen=True, slots=True)
class CameraMount:
    # p_body = R_body_camera @ p_camera + camera_origin_in_body_m
    rotation: tuple = ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    translation_m: tuple = (0.0, 0.0, 0.0)

    def __post_init__(self):
        rotation = np.asarray(self.rotation, dtype=float)
        translation = np.asarray(self.translation_m, dtype=float)
        if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
            raise ValueError("camera mount rotation must be a finite 3x3 matrix")
        if translation.shape != (3,) or not np.isfinite(translation).all():
            raise ValueError("camera mount translation_m must contain three finite values (in meters)")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(rotation), 1.0):
            raise ValueError("camera mount rotation must be orthonormal with determinant +1")
        object.__setattr__(self, "rotation", tuple(tuple(row) for row in rotation))
        object.__setattr__(self, "translation_m", tuple(translation))

    def to_body(self, pose: Pose3D) -> Pose3D:
        vector = self.body_from_camera.as_matrix() @ [pose.x, pose.y, pose.z, 1.0]
        return Pose3D(*(float(value) for value in vector[:3]))

    @property
    def body_from_camera(self) -> RigidTransform:
        matrix = np.eye(4)
        matrix[:3, :3] = self.rotation
        matrix[:3, 3] = self.translation_m
        return RigidTransform(matrix)

    def transform_to_body(self, camera_from_marker: RigidTransform) -> RigidTransform:
        return self.body_from_camera.compose(camera_from_marker)
