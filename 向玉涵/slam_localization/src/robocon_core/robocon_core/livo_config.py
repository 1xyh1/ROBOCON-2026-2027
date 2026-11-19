"""Measured CameraInfo and explicit transforms -> isolated FAST-LIVO2 parameters.

T_A_B maps a point in B to A; L=LiDAR, I=MID IMU, C=RGB optical frame.
This configuration is not an approved runtime calibration/map bundle.
"""
from __future__ import annotations

import math
import re
import numpy as np


def replay_log_failures(log: str) -> dict:
    """ros2 launch can exit zero even after its required child segfaults."""
    return {"child_exit_errors": [int(code) for code in re.findall(
                r"process has died \[pid \d+, exit code (-?\d+)", log)],
            "debugger_stop_signals": re.findall(r"received signal (SIG\w+)", log)}


def rigid_transform(value, label: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{label}: expected finite 4x4 transform")
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5, rtol=0)):
        raise ValueError(f"{label}: not a proper rigid transform")
    return matrix


def camera_model(info: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if info["distortion_model"] not in ("plumb_bob", "rational_polynomial"):
        raise ValueError("only ROS Brown-Conrady CameraInfo is supported")
    if (not isinstance(info["width"], int) or not isinstance(info["height"], int)
            or info["width"] <= 0 or info["height"] <= 0
            or info.get("binning_x", 0) not in (0, 1)
            or info.get("binning_y", 0) not in (0, 1)
            or any(info.get("roi", {}).get(k, 0) for k in ("x_offset", "y_offset", "width", "height"))):
        raise ValueError("full-resolution CameraInfo without ROI/binning is required")
    K = np.asarray(info["k"], dtype=float).reshape(3, 3)
    D = np.asarray(info["d"], dtype=float)
    R = np.asarray(info["r"], dtype=float).reshape(3, 3)
    P = np.asarray(info["p"], dtype=float).reshape(3, 4)
    if (D.shape not in ((4,), (5,), (8,), (12,), (14,))
            or not all(np.isfinite(m).all() for m in (K, D, R, P))
            or min(K[0, 0], K[1, 1], P[0, 0], P[1, 1]) <= 0
            or not np.allclose(R.T @ R, np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(R), 1.0, atol=1e-6)
            or not np.allclose(P[:, 3], 0, atol=1e-9)
            or not np.allclose(K[2], [0, 0, 1], atol=1e-9)
            or not np.allclose(P[2, :3], [0, 0, 1], atol=1e-9)
            or not np.allclose([P[0, 1], P[1, 0]], 0, atol=1e-9)):
        raise ValueError("invalid monocular CameraInfo matrices")
    if not info.get("frame_id"):
        raise ValueError("CameraInfo optical frame is required")
    return K, D, R, P[:, :3]


def parameters(camera: dict, calibration: dict, *, allow_unverified: bool = False) -> tuple[dict, dict]:
    if camera["camera_serial"] != calibration["camera_serial"]:
        raise ValueError("camera serial mismatch")
    if not calibration.get("calibration_id"):
        raise ValueError("an explicit calibration_id is required")
    info = camera["camera_info"]
    _, _, R_rect, projection = camera_model(info)
    T_I_L = rigid_transform(calibration["T_I_L"], "T_I_L")
    T_C_L = rigid_transform(calibration["T_C_L"], "T_C_L")
    unverified = []
    if calibration.get("status") != "measured":
        unverified.append("mechanical_extrinsics_not_measured")
    if not calibration.get("residual_time_verified", False):
        unverified.append("camera_residual_time_not_verified")
    offset = calibration.get("camera_to_common_time_s")
    if offset is None:
        if not allow_unverified:
            raise ValueError("camera residual time offset is missing")
        offset = 0.0
        unverified.append("zero_residual_time_assumed_for_experiment")
    if not math.isfinite(float(offset)):
        raise ValueError("camera time offset must be finite")
    if unverified and not allow_unverified:
        raise ValueError("unverified calibration requires explicit isolated experiment opt-in")
    # Rectification changes the camera axes; rotate translation as well as rotation.
    T_rect_C = np.eye(4)
    T_rect_C[:3, :3] = R_rect
    T_rect_L = T_rect_C @ T_C_L
    params = {
        "common": {"img_topic": "/fusion/input/color_rect", "lid_topic": "/livox/lidar",
                   "imu_topic": "/livox/imu", "img_en": 1, "lidar_en": 1,
                   "ros_driver_bug_fix": False, "publish_tf": False,
                   "odom_frame": "livo_odom", "body_frame": "livo_mid360_imu"},
        "camera": {"width": int(info["width"]), "height": int(info["height"]),
                   "fx": float(projection[0, 0]), "fy": float(projection[1, 1]),
                   "cx": float(projection[0, 2]), "cy": float(projection[1, 2])},
        "extrin_calib": {"extrinsic_R": T_I_L[:3, :3].ravel().tolist(),
                         "extrinsic_T": T_I_L[:3, 3].tolist(),
                         "Rcl": T_rect_L[:3, :3].ravel().tolist(),
                         "Pcl": T_rect_L[:3, 3].tolist()},
        "time_offset": {"img_time_offset": float(offset), "imu_time_offset": 0.0,
                        "lidar_time_offset": 0.0, "exposure_time_init": 0.0},
        "preprocess": {"lidar_type": 1, "scan_line": 4, "blind": 0.5,
                       "point_filter_num": 3, "filter_size_surf": 0.15,
                       "feature_extract_enabled": False},
        "imu": {"imu_en": True, "imu_int_frame": 30, "acc_cov": 0.1,
                "gyr_cov": 0.1, "b_acc_cov": 0.0001, "b_gyr_cov": 0.0001},
        "vio": {"max_iterations": 5, "img_point_cov": 100, "outlier_threshold": 1000,
                "patch_size": 8, "patch_pyrimid_level": 4, "normal_en": True,
                "raycast_en": False, "inverse_composition_en": False,
                "exposure_estimate_en": True, "inv_expo_cov": 0.1},
        # The fork reads min_iterations, not the upstream example's max_iterations.
        "lio": {"min_iterations": 5, "dept_err": 0.02, "beam_err": 0.05,
                "min_eigen_value": 0.0025, "voxel_size": 0.5, "max_layer": 2,
                "max_points_num": 50, "layer_init_num": [5, 5, 5, 5, 5]},
        "local_map": {"map_sliding_en": True, "half_map_size": 30, "sliding_thresh": 8.0},
        "uav": {"imu_rate_odom": False, "gravity_align_en": False},
        "publish": {"dense_map_en": False, "pub_effect_point_en": False,
                    "pub_plane_en": False, "pub_scan_num": 1, "blind_rgb_points": 0.0},
        "evo": {"pose_output_en": False},
        "pcd_save": {"pcd_save_en": False, "colmap_output_en": False, "interval": -1},
        "image_save": {"img_save_en": False},
    }
    evidence = {"kind": "isolated_livo_replay_not_competition_localization",
                "camera_serial": camera["camera_serial"],
                "calibration_id": calibration["calibration_id"], "publish_tf": False,
                "unverified": unverified, "depth_in_filter": False,
                "T_rectified_camera_L": T_rect_L.tolist(),
                "T_rectified_camera_I": (T_rect_L @ np.linalg.inv(T_I_L)).tolist(),
                "camera_to_common_time_s_applied_once": float(offset)}
    return {"/**": {"ros__parameters": params}}, evidence
