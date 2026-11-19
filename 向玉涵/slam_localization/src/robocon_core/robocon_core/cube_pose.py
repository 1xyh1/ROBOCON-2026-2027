"""RGB-D cube detection and grasp-oriented 3D pose estimation.

The estimator intentionally owns no arm logic.  It returns an object pose and a
visible face which a later ROS 2 adapter can transform into ``arm_base``.
Images follow OpenCV's BGR convention and depth is expressed in metres.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Literal

import cv2
import numpy as np

TargetClass = Literal["auto", "earth", "sky"]


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    distortion_model: str = "plumb_bob"
    distortion: tuple[float, ...] = ()

    def validate(self) -> None:
        if not np.isfinite((self.fx, self.fy, self.cx, self.cy)).all() or self.fx <= 0 or self.fy <= 0:
            raise ValueError("fx and fy must be positive")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("image size must be positive")
        if self.distortion_model not in ("", "plumb_bob", "rational_polynomial", "equidistant"):
            raise ValueError(f"unsupported distortion model: {self.distortion_model}")
        expected = {"": (0,), "plumb_bob": (0, 4, 5), "rational_polynomial": (0, 8), "equidistant": (4,)}
        if len(self.distortion) not in expected[self.distortion_model] or not np.isfinite(self.distortion).all():
            raise ValueError("invalid distortion coefficients")

    def rays(self, pixels_uv: np.ndarray) -> np.ndarray:
        """Deproject raw-image pixels to unit optical-depth rays, including D."""
        pixels = np.asarray(pixels_uv, dtype=np.float64).reshape(-1, 1, 2)
        matrix = np.array(((self.fx, 0, self.cx), (0, self.fy, self.cy), (0, 0, 1)), dtype=np.float64)
        if self.distortion_model == "equidistant":
            xy = cv2.fisheye.undistortPoints(pixels, matrix, np.asarray(self.distortion))
        elif self.distortion and np.any(self.distortion):
            xy = cv2.undistortPoints(pixels, matrix, np.asarray(self.distortion))
        else:
            xy = (pixels - np.array((self.cx, self.cy))) / np.array((self.fx, self.fy))
        return np.column_stack((xy.reshape(-1, 2), np.ones(len(pixels))))


@dataclass(frozen=True)
class PoseEstimate:
    class_id: str
    color: str
    surface_center_m: tuple[float, float, float]
    object_center_m: tuple[float, float, float]
    rotation_camera_object: tuple[tuple[float, float, float], ...]
    dimensions_m: tuple[float, float, float]
    approach_direction: tuple[float, float, float]
    euclidean_range_m: float
    optical_depth_m: float
    depth_valid_ratio: float
    fit_residual_m: float
    confidence: float
    orientation_observable: bool
    pixel_bbox_xywh: tuple[int, int, int, int]

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class _Candidate:
    contour: np.ndarray
    mask: np.ndarray
    color: str


class CubePoseEstimator:
    """Find coloured Earth/Sky faces and estimate their pose from aligned depth."""

    NOMINAL_SIZE_M = {"earth": 0.350, "sky": 0.200}

    def __init__(
        self,
        *,
        min_area_px: int = 500,
        min_depth_m: float = 0.15,
        max_depth_m: float = 4.0,
        plane_inlier_threshold_m: float = 0.008,
        morphology_px: int = 5,
    ) -> None:
        self.min_area_px = min_area_px
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
        self.plane_inlier_threshold_m = plane_inlier_threshold_m
        self.morphology_px = max(1, morphology_px | 1)

    def estimate(
        self,
        image_bgr: np.ndarray,
        depth_m: np.ndarray,
        intrinsics: CameraIntrinsics,
        target_class: TargetClass = "auto",
    ) -> list[PoseEstimate]:
        self._validate_inputs(image_bgr, depth_m, intrinsics)
        if target_class not in ("auto", *self.NOMINAL_SIZE_M):
            raise ValueError(f"unknown target class: {target_class}")
        estimates: list[PoseEstimate] = []
        for candidate in self._candidates(image_bgr):
            estimate = self._estimate_candidate(candidate, depth_m, intrinsics, target_class)
            if estimate is not None:
                estimates.append(estimate)
        return sorted(estimates, key=lambda x: (-x.confidence, x.euclidean_range_m))

    def estimate_best(
        self,
        image_bgr: np.ndarray,
        depth_m: np.ndarray,
        intrinsics: CameraIntrinsics,
        target_class: TargetClass = "auto",
    ) -> PoseEstimate | None:
        estimates = self.estimate(image_bgr, depth_m, intrinsics, target_class)
        return estimates[0] if estimates else None

    def _validate_inputs(
        self, image_bgr: np.ndarray, depth_m: np.ndarray, intrinsics: CameraIntrinsics
    ) -> None:
        intrinsics.validate()
        if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError("image_bgr must have shape HxWx3")
        if image_bgr.dtype != np.uint8:
            raise ValueError("image_bgr must contain uint8 pixels")
        if depth_m.ndim != 2 or depth_m.shape != image_bgr.shape[:2]:
            raise ValueError("aligned depth must have the same HxW as the colour image")
        if image_bgr.shape[1] != intrinsics.width or image_bgr.shape[0] != intrinsics.height:
            raise ValueError("intrinsics image size does not match the input")

    def _candidates(self, image_bgr: np.ndarray) -> list[_Candidate]:
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        ranges = {
            "red": [((0, 85, 55), (12, 255, 255)), ((168, 85, 55), (179, 255, 255))],
            "blue": [((95, 80, 45), (140, 255, 255))],
        }
        kernel = np.ones((self.morphology_px, self.morphology_px), np.uint8)
        found: list[_Candidate] = []
        for color, limits in ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in limits:
                mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if cv2.contourArea(contour) < self.min_area_px:
                    continue
                component = np.zeros_like(mask)
                cv2.drawContours(component, [contour], -1, 255, cv2.FILLED)
                found.append(_Candidate(contour=contour, mask=component.astype(bool), color=color))
        return found

    def _estimate_candidate(
        self,
        candidate: _Candidate,
        depth_m: np.ndarray,
        intr: CameraIntrinsics,
        target_class: TargetClass,
    ) -> PoseEstimate | None:
        valid_depth = (
            candidate.mask
            & np.isfinite(depth_m)
            & (depth_m >= self.min_depth_m)
            & (depth_m <= self.max_depth_m)
        )
        mask_count = int(candidate.mask.sum())
        if mask_count == 0:
            return None
        depth_valid_ratio = float(valid_depth.sum() / mask_count)
        if valid_depth.sum() < max(80, self.min_area_px // 4):
            return None

        # This estimator needs one complete face. A clipped/occluded silhouette
        # cannot determine the centre from its visible-pixel centroid.
        x, y, w, h = cv2.boundingRect(candidate.contour)
        if x <= 0 or y <= 0 or x + w >= intr.width or y + h >= intr.height:
            return None
        polygon = cv2.approxPolyDP(candidate.contour, 0.02 * cv2.arcLength(candidate.contour, True), True)
        if len(polygon) != 4 or not cv2.isContourConvex(polygon):
            return None
        v, u = np.nonzero(valid_depth)
        z = depth_m[v, u].astype(np.float64)
        points = intr.rays(np.column_stack((u, v))) * z[:, None]
        fit = self._fit_plane(points)
        if fit is None:
            return None
        plane_center, normal, inliers = fit
        if np.dot(normal, plane_center) > 0:
            normal = -normal
        axis_a = np.array((1.0, 0.0, 0.0)) if abs(normal[0]) < 0.9 else np.array((0.0, 1.0, 0.0))
        axis_a -= normal * np.dot(axis_a, normal)
        axis_a /= np.linalg.norm(axis_a)
        axis_b = np.cross(normal, axis_a)
        basis = np.column_stack((axis_a, axis_b))
        rays = intr.rays(candidate.contour.reshape(-1, 2))
        denominators = rays @ normal
        if np.any(np.abs(denominators) < 1e-6):
            return None
        edge_depth = float(plane_center @ normal) / denominators
        if np.any(edge_depth <= 0):
            return None
        boundary = rays * edge_depth[:, None]
        projected = ((boundary - plane_center) @ basis).astype(np.float32)
        rectangle = cv2.minAreaRect(projected.reshape(-1, 1, 2))
        span_a, span_b = rectangle[1]
        if min(span_a, span_b) <= 0 or max(span_a, span_b) / min(span_a, span_b) > 1.20:
            return None
        surface_center = plane_center + basis @ np.asarray(rectangle[0])
        corners = cv2.boxPoints(rectangle)
        edge = (corners[1] - corners[0]).astype(np.float64)
        axis_a = basis @ (edge / np.linalg.norm(edge))
        axis_b = np.cross(normal, axis_a)
        measured_size = float((span_a + span_b) / 2.0)
        if target_class == "auto":
            class_id = min(self.NOMINAL_SIZE_M, key=lambda key: abs(measured_size - self.NOMINAL_SIZE_M[key]))
        else:
            class_id = target_class
        nominal = self.NOMINAL_SIZE_M[class_id]
        size_error = abs(measured_size - nominal) / nominal
        if size_error > 0.20:
            return None

        plane_distance = (points[inliers] - plane_center) @ normal
        fit_residual = float(np.sqrt(np.mean(plane_distance**2)))
        object_center = surface_center - normal * (nominal / 2.0)
        rotation = np.column_stack((axis_a, axis_b, normal))
        contour_area = float(cv2.contourArea(candidate.contour))
        fill_ratio = contour_area / max(1.0, float(w * h))
        confidence = float(
            np.clip(
                0.35 * depth_valid_ratio * float(inliers.mean())
                + 0.30 * np.exp(-4.0 * size_error)
                + 0.20 * np.exp(-fit_residual / 0.012)
                + 0.15 * min(1.0, fill_ratio / 0.75),
                0.0,
                1.0,
            )
        )
        return PoseEstimate(
            class_id=class_id,
            color=candidate.color,
            surface_center_m=tuple(float(x) for x in surface_center),
            object_center_m=tuple(float(x) for x in object_center),
            rotation_camera_object=tuple(tuple(float(x) for x in row) for row in rotation),
            dimensions_m=(nominal, nominal, nominal),
            approach_direction=tuple(float(x) for x in -normal),
            euclidean_range_m=float(np.linalg.norm(object_center)),
            optical_depth_m=float(object_center[2]),
            depth_valid_ratio=depth_valid_ratio,
            fit_residual_m=fit_residual,
            confidence=confidence,
            orientation_observable=False,
            pixel_bbox_xywh=(int(x), int(y), int(w), int(h)),
        )

    def _fit_plane(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        """RANSAC rejects depth outliers by plane distance, preserving slanted faces."""
        rng = np.random.default_rng(0)
        sample = points[rng.choice(len(points), min(3000, len(points)), replace=False)]
        best = np.zeros(len(sample), dtype=bool)
        for _ in range(64):
            triple = sample[rng.choice(len(sample), 3, replace=False)]
            normal = np.cross(triple[1] - triple[0], triple[2] - triple[0])
            length = np.linalg.norm(normal)
            if length < 1e-9:
                continue
            normal /= length
            keep = np.abs((sample - triple[0]) @ normal) <= self.plane_inlier_threshold_m
            if keep.sum() > best.sum():
                best = keep
                if keep.all():
                    break
        if best.sum() < max(80, 0.65 * len(sample)):
            return None
        selected = sample[best]
        for _ in range(2):
            center = selected.mean(axis=0)
            centered = selected - center
            values, vectors = np.linalg.eigh(centered.T @ centered / len(centered))
            if values[1] < 1e-6:
                return None
            normal = vectors[:, 0]
            inliers = np.abs((points - center) @ normal) <= self.plane_inlier_threshold_m
            if inliers.sum() < max(80, 0.65 * len(points)):
                return None
            selected = points[inliers]
        return center, normal, inliers
