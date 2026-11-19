"""Point-cloud map registration used after LIO/LIVO produces a local scan.

This module is deliberately independent of ROS and PCL so the registration
logic can be tested on Windows before it is wrapped as ``map -> odom`` on the
Ubuntu robot.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class Pose3D:
    rotation: np.ndarray
    translation: np.ndarray

    @staticmethod
    def identity() -> "Pose3D":
        return Pose3D(np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64))

    @staticmethod
    def from_xyyaw(x: float, y: float, yaw_rad: float, z: float = 0.0) -> "Pose3D":
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        return Pose3D(
            np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)), dtype=np.float64),
            np.array((x, y, z), dtype=np.float64),
        )

    def transform(self, points: np.ndarray) -> np.ndarray:
        return points @ self.rotation.T + self.translation

    def compose_left(self, rotation: np.ndarray, translation: np.ndarray) -> "Pose3D":
        return Pose3D(rotation @ self.rotation, rotation @ self.translation + translation)

    def matrix(self) -> np.ndarray:
        result = np.eye(4, dtype=np.float64)
        result[:3, :3] = self.rotation
        result[:3, 3] = self.translation
        return result


@dataclass(frozen=True)
class RegistrationResult:
    pose_map_from_scan: Pose3D
    converged: bool
    iterations: int
    rmse_m: float
    overlap_ratio: float
    translation_step_m: float
    rotation_step_rad: float
    geometry_eigenvalues: tuple[float, float, float]
    accepted: bool
    seed_index: int = 0
    ambiguous: bool = False
    rejection_reason: str = ""


def voxel_downsample(points: np.ndarray, voxel_m: float) -> np.ndarray:
    points = _points(points)
    if voxel_m <= 0:
        return points.copy()
    keys = np.floor(points / voxel_m).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    sums = np.zeros((inverse.max() + 1, 3), dtype=np.float64)
    counts = np.bincount(inverse)
    np.add.at(sums, inverse, points)
    return sums / counts[:, None]


class IcpMapLocalizer:
    """Small-cloud reference ICP; not a real-time MID-360 registration backend.

    Ambiguity checks compare the supplied seeds only: accepting a candidate
    does not establish global uniqueness over unexplored parts of the map.
    """

    def __init__(
        self,
        *,
        voxel_m: float = 0.05,
        max_correspondence_m: float = 0.30,
        trim_fraction: float = 0.82,
        max_iterations: int = 35,
        translation_epsilon_m: float = 0.0005,
        rotation_epsilon_rad: float = 0.0005,
        accept_rmse_m: float = 0.045,
        accept_overlap: float = 0.55,
        min_correspondences: int = 30,
        min_geometry_ratio: float = 0.005,
        ambiguity_translation_m: float = 0.10,
        ambiguity_rotation_rad: float = math.radians(5.0),
        ambiguity_rmse_margin_m: float = 0.005,
        ambiguity_overlap_margin: float = 0.05,
    ) -> None:
        self.voxel_m = voxel_m
        self.max_correspondence_m = max_correspondence_m
        self.trim_fraction = trim_fraction
        self.max_iterations = max_iterations
        self.translation_epsilon_m = translation_epsilon_m
        self.rotation_epsilon_rad = rotation_epsilon_rad
        self.accept_rmse_m = accept_rmse_m
        self.accept_overlap = accept_overlap
        self.min_correspondences = min_correspondences
        self.min_geometry_ratio = min_geometry_ratio
        self.ambiguity_translation_m = ambiguity_translation_m
        self.ambiguity_rotation_rad = ambiguity_rotation_rad
        self.ambiguity_rmse_margin_m = ambiguity_rmse_margin_m
        self.ambiguity_overlap_margin = ambiguity_overlap_margin
        if not 0 < trim_fraction <= 1 or max_iterations < 1 or min_correspondences < 3:
            raise ValueError("invalid ICP iteration/correspondence configuration")

    def localize(
        self,
        scan_points: np.ndarray,
        map_points: np.ndarray,
        seeds: Iterable[Pose3D] | None = None,
    ) -> RegistrationResult:
        scan = voxel_downsample(scan_points, self.voxel_m)
        target = voxel_downsample(map_points, self.voxel_m)
        candidate_seeds = [Pose3D.identity()] if seeds is None else list(seeds)
        if not candidate_seeds:
            raise ValueError("at least one seed pose is required")
        results = [self._single(scan, target, seed, index) for index, seed in enumerate(candidate_seeds)]
        best = min(results, key=lambda x: (not x.accepted, x.rmse_m, -x.overlap_ratio))
        if best.accepted:
            for other in results:
                if not other.accepted or other.seed_index == best.seed_index:
                    continue
                translation = np.linalg.norm(other.pose_map_from_scan.translation - best.pose_map_from_scan.translation)
                delta = other.pose_map_from_scan.rotation @ best.pose_map_from_scan.rotation.T
                angle = math.acos(np.clip((np.trace(delta) - 1.0) / 2.0, -1.0, 1.0))
                distinct = translation > self.ambiguity_translation_m or angle > self.ambiguity_rotation_rad
                comparable = (
                    other.rmse_m <= best.rmse_m + self.ambiguity_rmse_margin_m
                    and other.overlap_ratio >= best.overlap_ratio - self.ambiguity_overlap_margin
                )
                if distinct and comparable:
                    return replace(best, accepted=False, ambiguous=True, rejection_reason="multiple_matching_poses")
        return best

    def _single(
        self, scan: np.ndarray, target: np.ndarray, seed: Pose3D, seed_index: int
    ) -> RegistrationResult:
        pose = seed
        converged = False
        translation_step = math.inf
        rotation_step = math.inf
        iterations = 0
        for iterations in range(1, self.max_iterations + 1):
            transformed = pose.transform(scan)
            indices, squared = _nearest_neighbors(transformed, target)
            distances = np.sqrt(squared)
            cutoff = min(self.max_correspondence_m, float(np.quantile(distances, self.trim_fraction)))
            keep = distances <= cutoff
            if int(keep.sum()) < self.min_correspondences:
                break
            rotation, translation = _rigid_fit(transformed[keep], target[indices[keep]])
            pose = pose.compose_left(rotation, translation)
            translation_step = float(np.linalg.norm(translation))
            rotation_step = float(math.acos(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)))
            if translation_step < self.translation_epsilon_m and rotation_step < self.rotation_epsilon_rad:
                converged = True
                break

        transformed = pose.transform(scan)
        indices, squared = _nearest_neighbors(transformed, target)
        distances = np.sqrt(squared)
        keep = distances <= self.max_correspondence_m
        overlap = float(keep.mean())
        rmse = float(np.sqrt(np.mean(squared[keep]))) if keep.any() else math.inf
        geometry = _geometry_eigenvalues(transformed[keep]) if keep.sum() >= 3 else (0.0, 0.0, 0.0)
        # A line leaves rotation about the line unobservable. A plane is not
        # automatically rank-deficient for point-to-point ICP with known pairs;
        # repeated surfaces still need the multi-seed ambiguity test above.
        geometry_ok = geometry[1] > 1e-6 and geometry[1] / max(geometry[2], 1e-12) >= self.min_geometry_ratio
        accepted = bool(
            converged
            and keep.sum() >= self.min_correspondences
            and overlap >= self.accept_overlap
            and rmse <= self.accept_rmse_m
            and geometry_ok
        )
        return RegistrationResult(
            pose_map_from_scan=pose,
            converged=converged,
            iterations=iterations,
            rmse_m=rmse,
            overlap_ratio=overlap,
            translation_step_m=translation_step,
            rotation_step_rad=rotation_step,
            geometry_eigenvalues=geometry,
            accepted=accepted,
            seed_index=seed_index,
            rejection_reason="" if accepted else ("degenerate_geometry" if not geometry_ok else "poor_alignment"),
        )


def _points(value: np.ndarray) -> np.ndarray:
    points = np.asarray(value, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
        raise ValueError("point cloud must have shape Nx3 and contain points")
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) == 0:
        raise ValueError("point cloud contains no finite points")
    return points


def _nearest_neighbors(source: np.ndarray, target: np.ndarray, chunk: int = 256) -> tuple[np.ndarray, np.ndarray]:
    # Bound the NxMx3 difference tensor and its reduction to roughly 64 MiB.
    # This is the small-cloud reference implementation, not the PCL runtime.
    chunk = max(1, min(chunk, 2_000_000 // len(target)))
    indices = np.empty(len(source), dtype=np.int64)
    squared = np.empty(len(source), dtype=np.float64)
    for start in range(0, len(source), chunk):
        query = source[start : start + chunk]
        distance_matrix = np.sum((query[:, None, :] - target[None, :, :]) ** 2, axis=2)
        local_indices = np.argmin(distance_matrix, axis=1)
        rows = np.arange(len(query))
        indices[start : start + len(query)] = local_indices
        squared[start : start + len(query)] = distance_matrix[rows, local_indices]
    return indices, squared


def _rigid_fit(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    return rotation, translation


def _geometry_eigenvalues(points: np.ndarray) -> tuple[float, float, float]:
    centered = points - points.mean(axis=0)
    values = np.linalg.eigvalsh(centered.T @ centered / max(1, len(points) - 1))
    return tuple(float(value) for value in values)
