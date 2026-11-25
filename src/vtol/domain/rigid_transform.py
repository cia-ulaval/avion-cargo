import math
from dataclasses import dataclass

import numpy as np

from vtol.domain.errors import InvalidPoseError
from vtol.domain.models import Pose3D


@dataclass(frozen=True, slots=True)
class RigidTransform:
    """Immutable 4x4 SE(3) matrix; translation is expressed in meters."""

    matrix: tuple

    def __post_init__(self) -> None:
        matrix = np.asarray(self.matrix, dtype=float)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise InvalidPoseError("Transform must be a finite 4x4 matrix")
        rotation = matrix[:3, :3]
        if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9, rtol=0):
            raise InvalidPoseError("Transform must have homogeneous last row [0, 0, 0, 1]")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0) or not np.isclose(
            np.linalg.det(rotation), 1, atol=1e-6, rtol=0
        ):
            raise InvalidPoseError("Transform rotation must be orthonormal with determinant +1")
        object.__setattr__(self, "matrix", tuple(tuple(float(value) for value in row) for row in matrix))

    @classmethod
    def from_vectors(cls, rotation_vector: np.ndarray, translation_vector: np.ndarray) -> "RigidTransform":
        """Convert an axis-angle rotation vector and metric translation to SE(3)."""
        r = np.asarray(rotation_vector, dtype=float).reshape(3)
        t = np.asarray(translation_vector, dtype=float).reshape(3)
        if not np.isfinite(r).all() or not np.isfinite(t).all():
            raise InvalidPoseError("Pose vectors must be finite")
        angle = float(np.linalg.norm(r))
        matrix = np.eye(4)
        if angle > 0:
            x, y, z = r / angle
            skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            matrix[:3, :3] = np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * skew @ skew
        matrix[:3, 3] = t
        return cls(matrix)

    def as_matrix(self) -> np.ndarray:
        """Return an independent numeric matrix for composition or point conversion."""
        return np.asarray(self.matrix, dtype=float)

    @property
    def position(self) -> Pose3D:
        return Pose3D(*(row[3] for row in self.matrix[:3]))

    def quaternion_wxyz(self) -> tuple[float, float, float, float]:
        r = self.as_matrix()[:3, :3]
        # Select the largest component to remain stable around 180-degree rotations.
        candidates = np.array(
            [
                1 + np.trace(r),
                1 + 2 * r[0, 0] - np.trace(r),
                1 + 2 * r[1, 1] - np.trace(r),
                1 + 2 * r[2, 2] - np.trace(r),
            ]
        )
        index = int(np.argmax(candidates))
        component = math.sqrt(max(0, candidates[index])) / 2
        q = np.zeros(4)
        q[index] = component
        scale = 4 * component
        if index == 0:
            q[1:] = [r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]
            q[1:] /= scale
        else:
            i = index - 1
            j, k = (i + 1) % 3, (i + 2) % 3
            q[0] = (r[k, j] - r[j, k]) / scale
            q[j + 1] = (r[j, i] + r[i, j]) / scale
            q[k + 1] = (r[k, i] + r[i, k]) / scale
        q /= np.linalg.norm(q)
        return tuple(float(value) for value in q)

    def compose(self, source_transform: "RigidTransform") -> "RigidTransform":
        return RigidTransform(self.as_matrix() @ source_transform.as_matrix())

    def to_list(self) -> list[list[float]]:
        return [list(row) for row in self.matrix]
