#!/usr/bin/env python3
"""从固定 AprilGrid 的 PnP 姿态与 IMU 陀螺数据估计旋转和时标候选。"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

from calibration_common import (EXPECTED_WHO_AM_I, EXPECTED_READY_FLAGS, GYRO_DPS_PER_LSB,
                                UINT16_MODULUS, UINT32_MODULUS, sequence_quality)
try:
    import numpy as np
except ImportError as exc:  # pragma: no cover - 运行环境提示
    raise SystemExit("需要 numpy；请使用已安装 ROS 2、OpenCV 与 numpy 的工作区 Python。") from exc


SCHEMA = "bx72_joint_imu_rotation_v1"
CAMERA_SCHEMA = "bx72_camera_pnp_v1"
TIME_BASIS_LABELS = {
    "ros_header": "ROS header stamp (bridge publication/receive timeline)",
    "received_wall": "joint-recorder callback wall-clock receive timestamp",
    "received_monotonic": "joint-recorder callback monotonic receive timestamp",
}


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number}: JSONL 格式错误：{exc}") from exc
            if isinstance(record, dict):
                records.append(record)
    return records


def _nested_time(row: dict[str, Any], name: str) -> Any:
    timestamps = row.get("timestamps")
    return timestamps.get(name) if isinstance(timestamps, dict) else None


def extract_time_ns(row: dict[str, Any], basis: str, is_camera: bool) -> int | None:
    if basis == "ros_header":
        if is_camera:
            header = _nested_time(row, "ros_header")
        else:
            header = row.get("header")
        if not isinstance(header, dict):
            return None
        try:
            sec = int(header["sec"])
            nanosec = int(header["nanosec"])
        except (KeyError, TypeError, ValueError):
            return None
        if nanosec < 0 or nanosec >= 1_000_000_000:
            return None
        return sec * 1_000_000_000 + nanosec
    if basis == "received_wall":
        value = row.get("received_wall_unix_ns")
        if value is None:
            value = _nested_time(row, "received_wall_unix_ns")
    else:
        value = row.get("received_monotonic_ns")
        if value is None:
            value = _nested_time(row, "received_monotonic_ns")
    try:
        return int(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def _parse_imu_rows(rows: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    parsed: list[dict[str, Any]] = []
    reject_reasons: Counter[str] = Counter()
    raw_count = 0
    flags_seen: set[int] = set()
    who_seen: set[int] = set()
    ready_state_known = True
    for row in rows:
        if "gyro_raw" not in row:
            continue
        raw_count += 1
        try:
            gyro_raw = np.asarray(row["gyro_raw"], dtype=np.float64).reshape(3)
        except (TypeError, ValueError):
            reject_reasons["gyro_raw_not_xyz"] += 1
            continue
        if not np.all(np.isfinite(gyro_raw)):
            reject_reasons["gyro_raw_nonfinite"] += 1
            continue
        flags = row.get("flags")
        who_am_i = row.get("who_am_i")
        if flags is None or who_am_i is None:
            ready_state_known = False
        else:
            flags = int(flags)
            who_am_i = int(who_am_i)
            flags_seen.add(flags)
            who_seen.add(who_am_i)
            if flags != EXPECTED_READY_FLAGS:
                reject_reasons["sensor_flags_not_ready"] += 1
                continue
            if who_am_i != EXPECTED_WHO_AM_I:
                reject_reasons["unexpected_who_am_i"] += 1
                continue
        gyro_rad_s = gyro_raw * (GYRO_DPS_PER_LSB * math.pi / 180.0)
        timestamp_us = row.get("timestamp_us")
        parsed.append(
            {
                "row": row,
                "gyro_raw": gyro_raw,
                "gyro_rad_s": gyro_rad_s,
                "reader_sequence": _optional_int(row.get("reader_sequence")),
                "packet_sequence": _optional_int(row.get("sequence")),
                "timestamp_us": _optional_int(timestamp_us),
            }
        )
    state_ok = ready_state_known and bool(parsed) and all(
        int(item["row"].get("flags", -1)) == EXPECTED_READY_FLAGS
        and int(item["row"].get("who_am_i", -1)) == EXPECTED_WHO_AM_I
        for item in parsed
    )
    reader_values = [item["reader_sequence"] for item in parsed if item["reader_sequence"] is not None]
    packet_values = [item["packet_sequence"] for item in parsed if item["packet_sequence"] is not None]
    timestamp_values = [
        item["timestamp_us"] % UINT32_MODULUS
        for item in parsed
        if item["timestamp_us"] is not None
    ]
    quality = {
        "raw_rows": raw_count,
        "usable_gyro_rows": len(parsed),
        "rejected_rows": dict(reject_reasons),
        "flags_seen": sorted(flags_seen),
        "who_am_i_seen": sorted(who_seen),
        "ready_state_fields_complete": ready_state_known,
        "ready_state_confirmed": state_ok,
        "reader_sequence": sequence_quality(reader_values, UINT16_MODULUS, backward_key="backward_or_reset_steps"),
        "packet_sequence": sequence_quality(packet_values, UINT32_MODULUS, backward_key="backward_or_reset_steps"),
        "fpga_timestamp_us_mod32": {
            "first": timestamp_values[0] if timestamp_values else None,
            "last": timestamp_values[-1] if timestamp_values else None,
            "used_for_alignment": False,
            "reason": "camera PnP records have no proven common FPGA IMU timestamp domain",
        },
        "gyro_scale_rad_s_per_lsb": GYRO_DPS_PER_LSB * math.pi / 180.0,
        "accelerometer_used_for_rotation_fit": False,
    }
    return parsed, quality


def _optional_int(value: Any) -> int | None:
    try:
        return int(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def _camera_matrix(row: dict[str, Any]) -> np.ndarray:
    pnp = row.get("pnp")
    if not isinstance(pnp, dict):
        raise ValueError("missing_pnp")
    value = pnp.get("R_camera_board_row_major")
    if value is None:
        transform = pnp.get("T_camera_board_row_major_4x4")
        if transform is None:
            raise ValueError("missing_pose_matrix")
        matrix4 = np.asarray(transform, dtype=np.float64).reshape(4, 4)
        value = matrix4[:3, :3]
    matrix = np.asarray(value, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(matrix)):
        raise ValueError("nonfinite_pose_matrix")
    determinant = float(np.linalg.det(matrix))
    orthogonality_error = float(np.linalg.norm(matrix.T @ matrix - np.eye(3), ord="fro"))
    if determinant <= 0.0 or abs(determinant - 1.0) > 0.05 or orthogonality_error > 0.05:
        raise ValueError("pose_rotation_not_in_SO3")
    u, _, vt = np.linalg.svd(matrix)
    projection = np.eye(3)
    projection[2, 2] = np.linalg.det(u @ vt)
    corrected = u @ projection @ vt
    return corrected


def _camera_pose_rows(rows: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    valid: list[dict[str, Any]] = []
    failure_reasons: Counter[str] = Counter()
    pnp_statuses: Counter[str] = Counter()
    provisional_values: set[bool] = set()
    calibration_sources: set[str] = set()
    calibration_hashes: set[str] = set()
    used_camera_fit_rows = 0
    detections_counts: list[int] = []
    orthogonalized = 0
    raw_pose_rows = 0
    rows_without_pnp_record = 0
    for row in rows:
        pnp = row.get("pnp")
        quality = row.get("quality")
        if not isinstance(pnp, dict):
            rows_without_pnp_record += 1
            continue
        raw_pose_rows += 1
        status = str(pnp.get("status", "missing"))
        pnp_statuses[status] += 1
        fit = row.get("camera_fit")
        if isinstance(fit, dict):
            used_camera_fit_rows += 1
            if isinstance(fit.get("provisional"), bool):
                provisional_values.add(bool(fit["provisional"]))
            if fit.get("source_path") is not None:
                calibration_sources.add(str(fit["source_path"]))
            if fit.get("sha256") is not None:
                calibration_hashes.add(str(fit["sha256"]))
        if status != "ok":
            failure_reasons[str(pnp.get("failure_reason") or f"pnp_status_{status}")] += 1
            continue
        if not isinstance(quality, dict) or quality.get("valid") is not True:
            failure_reasons["quality_valid_false_or_missing"] += 1
            continue
        detections = row.get("detections")
        ids = detections.get("ids") if isinstance(detections, dict) else None
        if ids is None and isinstance(detections, dict):
            markers = detections.get("markers", [])
            ids = [marker.get("id") for marker in markers if isinstance(marker, dict)]
        try:
            unique_ids = {int(value) for value in ids}
        except (TypeError, ValueError):
            unique_ids = set()
        detections_counts.append(len(unique_ids))
        if len(unique_ids) < 6:
            failure_reasons["fewer_than_six_unique_detected_tags"] += 1
            continue
        try:
            rotation = _camera_matrix(row)
        except (ValueError, TypeError, np.linalg.LinAlgError) as exc:
            failure_reasons[str(exc) or "invalid_pnp_rotation"] += 1
            continue
        raw_rotation = pnp.get("R_camera_board_row_major")
        if raw_rotation is not None:
            raw_rotation_matrix = np.asarray(raw_rotation, dtype=np.float64).reshape(3, 3)
            if np.linalg.norm(rotation - raw_rotation_matrix) > 1e-8:
                orthogonalized += 1
        translation = pnp.get("t_camera_board_m")
        if translation is not None:
            try:
                translation = np.asarray(translation, dtype=np.float64).reshape(3)
            except (TypeError, ValueError):
                failure_reasons["invalid_pnp_translation"] += 1
                continue
            if not np.all(np.isfinite(translation)) or translation[2] <= 0.0:
                failure_reasons["nonpositive_or_nonfinite_board_depth"] += 1
                continue
        times = {basis: extract_time_ns(row, basis, is_camera=True) for basis in TIME_BASIS_LABELS}
        valid.append({"rotation_camera_board": rotation, "times_ns": times, "row": row})
    quality_report = {
        "camera_input_rows": len(rows),
        "rows_with_pnp_record": raw_pose_rows,
        "rows_without_pnp_record": rows_without_pnp_record,
        "valid_pnp_pose_rows": len(valid),
        "invalid_or_rejected_pose_rows": max(0, raw_pose_rows - len(valid)),
        "pnp_status_counts": dict(pnp_statuses),
        "rejection_reasons": dict(failure_reasons),
        "unique_detected_tag_count_min": min(detections_counts) if detections_counts else None,
        "unique_detected_tag_count_median": float(np.median(detections_counts)) if detections_counts else None,
        "camera_fit_rows_with_provenance": used_camera_fit_rows,
        "camera_fit_provisional_values": sorted(provisional_values),
        "camera_fit_sources": sorted(calibration_sources),
        "camera_fit_sha256_values": sorted(calibration_hashes),
        "rotation_matrices_reorthogonalized": orthogonalized,
    }
    return valid, quality_report


def build_imu_series(
    imu_rows: list[dict[str, Any]], basis: str, origin_ns: int
) -> tuple[np.ndarray, np.ndarray, int, int, list[tuple[float, float]]]:
    timed: list[tuple[int, np.ndarray, int | None, int | None]] = []
    missing_time = 0
    for item in imu_rows:
        time_ns = extract_time_ns(item["row"], basis, is_camera=False)
        if time_ns is None:
            missing_time += 1
            continue
        timed.append((time_ns, item["gyro_rad_s"], item["reader_sequence"], item["packet_sequence"]))
    timed.sort(key=lambda pair: pair[0])
    gaps = []
    for previous, current in zip(timed, timed[1:]):
        reader_ok = previous[2] is not None and current[2] is not None and (current[2] - previous[2]) % UINT16_MODULUS == 1
        packet_ok = previous[3] is not None and current[3] is not None and (current[3] - previous[3]) % UINT32_MODULUS == 1
        if current[0] > previous[0] and not (reader_ok and packet_ok):
            gaps.append(((previous[0] - origin_ns) / 1e9, (current[0] - origin_ns) / 1e9))
    grouped: dict[int, list[np.ndarray]] = defaultdict(list)
    for time_ns, gyro, _, _ in timed:
        grouped[time_ns].append(gyro)
    duplicate_count = sum(len(values) - 1 for values in grouped.values())
    time_ns_values = sorted(grouped)
    times = np.asarray([(value - origin_ns) / 1e9 for value in time_ns_values], dtype=np.float64)
    rates = np.vstack([np.mean(grouped[value], axis=0) for value in time_ns_values]) if time_ns_values else np.empty((0, 3))
    return times, rates, missing_time, duplicate_count, gaps


def build_camera_series(
    pose_rows: list[dict[str, Any]], basis: str, origin_ns: int
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    timed: list[tuple[int, np.ndarray, dict[str, Any]]] = []
    missing_time = 0
    for pose in pose_rows:
        time_ns = pose["times_ns"].get(basis)
        if time_ns is None:
            missing_time += 1
            continue
        timed.append((time_ns, pose["rotation_camera_board"], pose["row"]))
    timed.sort(key=lambda item: item[0])
    poses: list[dict[str, Any]] = []
    duplicate_timestamps = 0
    previous_ns: int | None = None
    for time_ns, rotation, row in timed:
        if time_ns == previous_ns:
            duplicate_timestamps += 1
            continue
        previous_ns = time_ns
        poses.append(
            {
                "time_s": (time_ns - origin_ns) / 1e9,
                "time_ns": time_ns,
                "rotation_camera_board": rotation,
                "row": row,
            }
        )
    return poses, {"poses_missing_timestamp": missing_time, "duplicate_timestamps_dropped": duplicate_timestamps}


def rotation_matrix_to_quaternion_xyzw(matrix: np.ndarray) -> list[float]:
    r = matrix
    trace = float(np.trace(r))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (r[2, 1] - r[1, 2]) / scale
        qy = (r[0, 2] - r[2, 0]) / scale
        qz = (r[1, 0] - r[0, 1]) / scale
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        scale = math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) * 2.0
        qw = (r[2, 1] - r[1, 2]) / max(scale, 1e-15)
        qx = 0.25 * scale
        qy = (r[0, 1] + r[1, 0]) / max(scale, 1e-15)
        qz = (r[0, 2] + r[2, 0]) / max(scale, 1e-15)
    elif r[1, 1] > r[2, 2]:
        scale = math.sqrt(max(0.0, 1.0 + r[1, 1] - r[0, 0] - r[2, 2])) * 2.0
        qw = (r[0, 2] - r[2, 0]) / max(scale, 1e-15)
        qx = (r[0, 1] + r[1, 0]) / max(scale, 1e-15)
        qy = 0.25 * scale
        qz = (r[1, 2] + r[2, 1]) / max(scale, 1e-15)
    else:
        scale = math.sqrt(max(0.0, 1.0 + r[2, 2] - r[0, 0] - r[1, 1])) * 2.0
        qw = (r[1, 0] - r[0, 1]) / max(scale, 1e-15)
        qx = (r[0, 2] + r[2, 0]) / max(scale, 1e-15)
        qy = (r[1, 2] + r[2, 1]) / max(scale, 1e-15)
        qz = 0.25 * scale
    quat = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    quat /= np.linalg.norm(quat)
    if quat[3] < 0.0:
        quat *= -1.0
    return quat.tolist()


def rotation_log(matrix: np.ndarray) -> np.ndarray:
    quat = np.asarray(rotation_matrix_to_quaternion_xyzw(matrix), dtype=np.float64)
    vector = quat[:3]
    magnitude = float(np.linalg.norm(vector))
    if magnitude < 1e-12:
        return 2.0 * vector
    angle = 2.0 * math.atan2(magnitude, max(0.0, float(quat[3])))
    return vector * (angle / magnitude)


def camera_angular_segments(
    poses: list[dict[str, Any]], max_gap_s: float, min_rate_rad_s: float,
    min_separation_s: float = 0.0,
) -> tuple[list[dict[str, Any]], dict[str, int]]:
    segments: list[dict[str, Any]] = []
    counters = Counter()
    pairs = []
    index = 0
    while index + 1 < len(poses):
        next_index = index + 1
        if min_separation_s > 0:
            while next_index < len(poses) and poses[next_index]["time_s"] - poses[index]["time_s"] < min_separation_s:
                next_index += 1
        if next_index >= len(poses):
            break
        pairs.append((index, poses[index], poses[next_index]))
        index = next_index if min_separation_s > 0 else index + 1
    for index, first, second in pairs:
        dt = second["time_s"] - first["time_s"]
        if dt <= 0.0:
            counters["nonpositive_frame_interval"] += 1
            continue
        if dt > max_gap_s:
            counters["camera_time_gap_over_limit"] += 1
            continue
        # R_camera_board maps board vectors into the moving optical frame.
        # R_i @ R_j.T is the body-frame camera increment for a static board.
        relative = first["rotation_camera_board"] @ second["rotation_camera_board"].T
        try:
            omega_camera = rotation_log(relative) / dt
        except (FloatingPointError, ValueError, np.linalg.LinAlgError):
            counters["camera_rotation_log_failed"] += 1
            continue
        rate = float(np.linalg.norm(omega_camera))
        if not math.isfinite(rate):
            counters["camera_rate_nonfinite"] += 1
            continue
        if rate < min_rate_rad_s:
            counters["low_camera_rotation_rate"] += 1
            continue
        segments.append(
            {
                "index": index,
                "camera_t0_s": first["time_s"],
                "camera_t1_s": second["time_s"],
                "dt_s": dt,
                "omega_camera_rad_s": omega_camera,
                "angle_rad": float(np.linalg.norm(rotation_log(relative))),
            }
        )
    return segments, dict(counters)


def _interp(times: np.ndarray, values: np.ndarray, at: float) -> np.ndarray | None:
    if not len(times) or at < times[0] or at > times[-1]:
        return None
    right = int(np.searchsorted(times, at, side="left"))
    if right < len(times) and times[right] == at:
        return values[right]
    if right == 0 or right == len(times):
        return None
    left = right - 1
    span = times[right] - times[left]
    if span <= 0.0:
        return None
    fraction = (at - times[left]) / span
    return values[left] * (1.0 - fraction) + values[right] * fraction


def integrate_average_gyro(
    times: np.ndarray,
    rates: np.ndarray,
    start: float,
    end: float,
    max_imu_gap_s: float,
    sequence_gaps: list[tuple[float, float]],
) -> np.ndarray | None:
    if end <= start or start < times[0] or end > times[-1]:
        return None
    if any(start < gap_end and end > gap_start for gap_start, gap_end in sequence_gaps):
        return None
    first = _interp(times, rates, start)
    last = _interp(times, rates, end)
    if first is None or last is None:
        return None
    inside_start = int(np.searchsorted(times, start, side="right"))
    inside_end = int(np.searchsorted(times, end, side="left"))
    inside_times = times[inside_start:inside_end]
    inside_rates = rates[inside_start:inside_end]
    local_times = np.concatenate(([start], inside_times, [end]))
    local_rates = np.vstack((first, inside_rates, last))
    intervals = np.diff(local_times)
    if not len(intervals) or np.any(intervals <= 0.0) or np.max(intervals) > max_imu_gap_s:
        return None
    trapezoids = (local_rates[:-1] + local_rates[1:]) * (0.5 * intervals[:, None])
    average = np.sum(trapezoids, axis=0) / (end - start)
    return average if np.all(np.isfinite(average)) else None


def fit_rotation_and_bias(
    imu_vectors: np.ndarray, camera_vectors: np.ndarray
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    x_mean = np.mean(imu_vectors, axis=0)
    y_mean = np.mean(camera_vectors, axis=0)
    x_centered = imu_vectors - x_mean
    y_centered = camera_vectors - y_mean
    covariance = y_centered.T @ x_centered
    u, singular_values, vt = np.linalg.svd(covariance)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(u @ vt)
    rotation_camera_imu = u @ correction @ vt
    intercept_camera_rad_s = y_mean - rotation_camera_imu @ x_mean
    gyro_bias_imu_rad_s = -rotation_camera_imu.T @ intercept_camera_rad_s
    predicted = imu_vectors @ rotation_camera_imu.T + intercept_camera_rad_s
    residuals = camera_vectors - predicted
    residual_norms = np.linalg.norm(residuals, axis=1)
    camera_centered_energy = float(np.sum(y_centered * y_centered))
    error_energy = float(np.sum(residuals * residuals))
    rank_ratio = (
        float(singular_values[1] / singular_values[0])
        if len(singular_values) >= 2 and singular_values[0] > 1e-15
        else 0.0
    )
    condition_number = float(singular_values[0] / singular_values[1]) if singular_values[1] > 1e-15 else math.inf
    rank3_ratio = (
        float(singular_values[2] / singular_values[0])
        if len(singular_values) >= 3 and singular_values[0] > 1e-15
        else 0.0
    )
    observable = rank_ratio >= 0.05 and condition_number <= 20.0
    metrics = {
        "pairs": int(len(imu_vectors)),
        "rmse_rad_s": float(np.sqrt(np.mean(residual_norms**2))) if len(residual_norms) else None,
        "median_error_rad_s": float(np.median(residual_norms)) if len(residual_norms) else None,
        "p95_error_rad_s": float(np.percentile(residual_norms, 95)) if len(residual_norms) else None,
        "camera_rate_rms_rad_s": (
            float(np.sqrt(np.mean(np.sum(camera_vectors * camera_vectors, axis=1))))
            if len(camera_vectors)
            else None
        ),
        "r_squared": 1.0 - error_energy / camera_centered_energy if camera_centered_energy > 1e-15 else None,
        "cross_covariance_singular_values": singular_values.tolist(),
        "second_to_first_singular_ratio": rank_ratio,
        "third_to_first_singular_ratio": rank3_ratio,
        "condition_number_s1_over_s2": condition_number if math.isfinite(condition_number) else None,
        "rotation_observable_from_excitation": observable,
        "observability_rule": "至少两个非共线角速度方向：s2/s1 >= 0.05 且 s1/s2 <= 20；s3/s1 另行报告。",
    }
    return rotation_camera_imu, gyro_bias_imu_rad_s, metrics


def _error_metrics(
    rotation: np.ndarray,
    bias_imu: np.ndarray,
    imu_vectors: np.ndarray,
    camera_vectors: np.ndarray,
) -> dict[str, Any]:
    prediction = (imu_vectors - bias_imu) @ rotation.T
    residuals = camera_vectors - prediction
    norms = np.linalg.norm(residuals, axis=1)
    denominator = float(np.sum((camera_vectors - np.mean(camera_vectors, axis=0)) ** 2))
    numerator = float(np.sum(residuals**2))
    _, _, excitation = fit_rotation_and_bias(imu_vectors, camera_vectors)
    excitation.update(
        {
            "rmse_rad_s": float(np.sqrt(np.mean(norms**2))) if len(norms) else None,
            "median_error_rad_s": float(np.median(norms)) if len(norms) else None,
            "p95_error_rad_s": float(np.percentile(norms, 95)) if len(norms) else None,
            "r_squared": 1.0 - numerator / denominator if denominator > 1e-15 else None,
        }
    )
    return excitation


def _solve_time_basis(
    basis: str,
    imu_rows: list[dict[str, Any]],
    camera_poses: list[dict[str, Any]],
    args: argparse.Namespace,
) -> dict[str, Any]:
    camera_ns = [pose["times_ns"][basis] for pose in camera_poses if pose["times_ns"].get(basis) is not None]
    imu_ns = [extract_time_ns(item["row"], basis, is_camera=False) for item in imu_rows]
    imu_ns = [value for value in imu_ns if value is not None]
    if len(camera_ns) < 3 or len(imu_ns) < 3:
        return {
            "status": "unavailable",
            "reason": "此时间基准的 camera 或 IMU 有效时标不足。",
            "time_basis": basis,
            "time_semantics": TIME_BASIS_LABELS[basis],
        }
    origin_ns = min(min(camera_ns), min(imu_ns))
    imu_times, imu_rates, missing_imu_time, duplicate_imu_times, sequence_gaps = build_imu_series(imu_rows, basis, origin_ns)
    camera_series, camera_time_quality = build_camera_series(camera_poses, basis, origin_ns)
    base = {
        "time_basis": basis,
        "time_semantics": TIME_BASIS_LABELS[basis],
        "offset_definition": "t_imu_query = t_camera_pose_interval + delta_imu_minus_camera",
        "imu_samples_timed": int(len(imu_times)),
        "imu_samples_missing_timestamp": missing_imu_time,
        "imu_duplicate_timestamps_collapsed": duplicate_imu_times,
        "imu_sequence_gap_intervals_rejected": len(sequence_gaps),
        "camera_pose_count_on_basis": int(len(camera_series)),
        "camera_time_quality": camera_time_quality,
        "imu_time_span_s": float(imu_times[-1] - imu_times[0]) if len(imu_times) > 1 else 0.0,
        "imu_interval_median_ms": (
            float(np.median(np.diff(imu_times)) * 1000.0) if len(imu_times) > 1 else None
        ),
        "imu_effective_rate_hz": (
            float((len(imu_times) - 1) / (imu_times[-1] - imu_times[0]))
            if len(imu_times) > 1 and imu_times[-1] > imu_times[0]
            else None
        ),
        "camera_time_span_s": (
            float(camera_series[-1]["time_s"] - camera_series[0]["time_s"])
            if len(camera_series) > 1
            else 0.0
        ),
    }
    if len(imu_times) < args.min_pairs or len(camera_series) < 3:
        return {**base, "status": "insufficient_data", "reason": "采样数不足。"}
    segments, segment_quality = camera_angular_segments(
        camera_series, args.max_camera_gap_ms / 1000.0, args.min_camera_rate_rad_s,
        args.camera_pair_separation_ms / 1000.0,
    )
    base["camera_angular_segment_filter"] = segment_quality
    base["camera_angular_segments_before_imu_coverage"] = len(segments)
    if not segments:
        return {
            **base,
            "status": "not_observable",
            "reason": "没有超过角速度门限且通过 PnP 时间间隔检查的相机姿态增量。静止姿态不能校准该旋转。",
        }

    max_offset_s = args.max_offset_ms / 1000.0
    max_imu_gap_s = args.max_imu_gap_ms / 1000.0
    segments = [
        segment
        for segment in segments
        if segment["camera_t0_s"] - max_offset_s >= imu_times[0]
        and segment["camera_t1_s"] + max_offset_s <= imu_times[-1]
    ]
    base["camera_segments_inside_all_offset_coverage"] = len(segments)
    if len(segments) < args.min_pairs:
        return {
            **base,
            "status": "insufficient_time_overlap",
            "reason": "在完整时偏搜索区间内共同覆盖的相机运动区间不足。",
        }

    grid = np.arange(
        -args.max_offset_ms,
        args.max_offset_ms + args.offset_step_ms * 0.5,
        args.offset_step_ms,
        dtype=np.float64,
    ) / 1000.0
    aligned_by_offset: list[dict[int, np.ndarray]] = []
    for delta in grid:
        aligned: dict[int, np.ndarray] = {}
        for index, segment in enumerate(segments):
            average = integrate_average_gyro(
                imu_times,
                imu_rates,
                segment["camera_t0_s"] + float(delta),
                segment["camera_t1_s"] + float(delta),
                max_imu_gap_s,
                sequence_gaps,
            )
            if average is not None:
                aligned[index] = average
        aligned_by_offset.append(aligned)
    common_indices = set(aligned_by_offset[0])
    for aligned in aligned_by_offset[1:]:
        common_indices.intersection_update(aligned)
    common_indices = sorted(common_indices)
    base["segments_integrable_at_every_offset"] = len(common_indices)
    base["segments_lost_to_imu_gaps_at_some_offsets"] = len(segments) - len(common_indices)
    if len(common_indices) < args.min_pairs:
        return {
            **base,
            "status": "insufficient_imu_continuity",
            "reason": "IMU 丢样或间隔使共同时间偏移范围内的可积分运动区间不足。",
        }

    split_at = int(math.floor(len(common_indices) * (1.0 - args.holdout_fraction)))
    train_indices = common_indices[:split_at]
    holdout_indices = common_indices[split_at:]
    required_train = max(10, int(math.ceil(args.min_pairs * 0.60)))
    required_holdout = max(5, int(math.ceil(args.min_pairs * 0.15)))
    base["chronological_split"] = {
        "train_pairs": len(train_indices),
        "holdout_pairs": len(holdout_indices),
        "holdout_fraction_requested": args.holdout_fraction,
        "required_train_pairs": required_train,
        "required_holdout_pairs": required_holdout,
        "method": "先按有效相机运动区间时间排序，最后一段留作独立时间留出集。",
    }
    if len(train_indices) < required_train or len(holdout_indices) < required_holdout:
        return {
            **base,
            "status": "insufficient_holdout",
            "reason": "训练或按时间分离的留出样本不足。",
        }

    camera_vectors = np.vstack([segments[index]["omega_camera_rad_s"] for index in common_indices])
    train_count = len(train_indices)
    train_positions = list(range(train_count))
    holdout_positions = list(range(train_count, len(common_indices)))
    grid_scores: list[float] = []
    grid_fits: list[tuple[np.ndarray, np.ndarray, dict[str, Any]] | None] = []
    for aligned in aligned_by_offset:
        imu_vectors = np.vstack([aligned[index] for index in common_indices])
        train_fit = fit_rotation_and_bias(
            imu_vectors[train_positions], camera_vectors[train_positions]
        )
        grid_fits.append(train_fit)
        score = train_fit[2]["rmse_rad_s"]
        grid_scores.append(float(score) if score is not None else math.inf)
    best_grid_index = int(np.argmin(grid_scores))
    best_delta_s = float(grid[best_grid_index])
    best_train_rotation, best_train_bias, best_train_excitation = grid_fits[best_grid_index]
    best_train_rmse = grid_scores[best_grid_index]
    if best_grid_index in (0, len(grid) - 1):
        offset_at_boundary = True
    else:
        offset_at_boundary = False
    near_tolerance = max(0.005, best_train_rmse * 0.02)
    near_indices = [i for i, score in enumerate(grid_scores) if score <= best_train_rmse + near_tolerance]
    near_span_ms = (
        (float(grid[max(near_indices)]) - float(grid[min(near_indices)])) * 1000.0
        if near_indices
        else None
    )
    max_acceptable_span_ms = max(20.0, 3.0 * args.offset_step_ms)
    offset_unique = (
        not offset_at_boundary
        and near_span_ms is not None
        and near_span_ms <= max_acceptable_span_ms
    )

    all_imu_vectors = np.vstack([aligned_by_offset[best_grid_index][index] for index in common_indices])
    all_camera_vectors = camera_vectors
    final_rotation, final_bias, full_fit_metrics = fit_rotation_and_bias(all_imu_vectors, all_camera_vectors)
    validation_metrics = _error_metrics(
        best_train_rotation,
        best_train_bias,
        all_imu_vectors[holdout_positions],
        all_camera_vectors[holdout_positions],
    )
    train_metrics = _error_metrics(
        best_train_rotation,
        best_train_bias,
        all_imu_vectors[train_positions],
        all_camera_vectors[train_positions],
    )
    rotation_valid = bool(
        best_train_excitation["rotation_observable_from_excitation"]
        and validation_metrics["rotation_observable_from_excitation"]
        and best_train_rmse <= args.max_train_rmse_rad_s
        and validation_metrics["rmse_rad_s"] is not None
        and validation_metrics["rmse_rad_s"] <= args.max_holdout_rmse_rad_s
        and train_metrics["r_squared"] is not None and train_metrics["r_squared"] >= 0.5
        and validation_metrics["r_squared"] is not None and validation_metrics["r_squared"] >= 0.5
    )
    if rotation_valid:
        rotation_status = "observable_candidate_with_holdout_support"
        rotation_reason = "训练和留出数据均包含非共线角速度方向，残差低于配置阈值且 R² 不低于 0.5。"
    elif not best_train_excitation["rotation_observable_from_excitation"]:
        rotation_status = "not_observable"
        rotation_reason = "训练激励方向秩不足或条件数过差；单轴/静止数据不能唯一确定三维旋转。"
    elif not validation_metrics["rotation_observable_from_excitation"]:
        rotation_status = "holdout_not_observable"
        rotation_reason = "留出集未覆盖足够的非共线旋转方向，不能声称独立验证通过。"
    else:
        rotation_status = "failed_residual_gate"
        rotation_reason = "训练或独立留出残差、相对解释度未达到门槛。"
    timing_status = "candidate_unique_on_selected_timeline" if offset_unique else (
        "ambiguous_or_search_boundary"
    )
    if offset_at_boundary:
        timing_reason = "最优候选落在搜索边界；扩大时偏范围前不得将其视为确定偏移。"
    elif not offset_unique:
        timing_reason = "近似等价的候选覆盖较宽时间区间，数据未能唯一约束该时标偏移。"
    else:
        timing_reason = "该值仅是在所选 ROS/接收时标上的有效对齐候选，不是曝光时间或硬件同步精度。"

    return {
        **base,
        "status": "candidate" if rotation_valid else rotation_status,
        "rotation_status": rotation_status,
        "rotation_reason": rotation_reason,
        "time_offset_status": timing_status,
        "time_offset_reason": timing_reason,
        "estimated_delta_imu_minus_camera_ms": best_delta_s * 1000.0,
        "time_offset_grid_resolution_ms": args.offset_step_ms,
        "time_offset_search_bounds_ms": [-args.max_offset_ms, args.max_offset_ms],
        "offset_near_best_error_tolerance_rad_s": near_tolerance,
        "offset_near_best_span_ms": near_span_ms,
        "offset_max_acceptable_near_best_span_ms": max_acceptable_span_ms,
        "offset_at_search_boundary": offset_at_boundary,
        "offset_unique_by_grid_score": offset_unique,
        "train_rmse_vs_offset_rad_s": [
            {"delta_imu_minus_camera_ms": float(grid[index] * 1000.0), "rmse_rad_s": grid_scores[index]}
            for index in range(len(grid))
        ],
        "rotation_camera_imu": {
            "maps": "angular velocity vectors in IMU660RC native sensor axes into camera optical axes",
            "matrix_row_major": final_rotation.tolist(),
            "quaternion_xyzw": rotation_matrix_to_quaternion_xyzw(final_rotation),
            "gyro_bias_candidate_sensor_axes_rad_s": final_bias.tolist(),
            "train_only_matrix_row_major": best_train_rotation.tolist(),
            "train_only_gyro_bias_sensor_axes_rad_s": best_train_bias.tolist(),
            "full_data_fit_metrics": full_fit_metrics,
            "train_metrics": train_metrics,
            "heldout_metrics": validation_metrics,
        },
    }


def make_report(args: argparse.Namespace) -> tuple[dict[str, Any], bool]:
    imu_input_rows = read_jsonl(Path(args.imu).expanduser())
    camera_input_rows = read_jsonl(Path(args.camera).expanduser())
    imu_rows, imu_quality = _parse_imu_rows(imu_input_rows)
    camera_poses, camera_quality = _camera_pose_rows(camera_input_rows)
    requested_bases = args.time_bases
    basis_results = {
        basis: _solve_time_basis(basis, imu_rows, camera_poses, args)
        for basis in requested_bases
    }
    candidate_bases = [
        basis
        for basis, result in basis_results.items()
        if result.get("rotation_status") == "observable_candidate_with_holdout_support"
    ]
    time_unique_bases = [
        basis
        for basis in candidate_bases
        if basis_results[basis].get("offset_unique_by_grid_score") is True
    ]
    provisional_values = camera_quality["camera_fit_provisional_values"]
    provisional = True if not provisional_values else any(provisional_values)
    report = {
        "schema": SCHEMA,
        "status": (
            "exploratory_candidates_only"
            if candidate_bases
            else "no_observable_joint_rotation_candidate"
        ),
        "inputs": {"imu_jsonl": str(Path(args.imu)), "camera_pnp_jsonl": str(Path(args.camera))},
        "assumption": {
            "target_board_static_in_environment": True,
            "camera_pose_convention": "T_camera_board maps board points into camera optical frame: p_camera = R_camera_board * p_board + t_camera_board.",
            "camera_optical_axes": "x right, y down, z forward; board axes are supplied by the AprilGrid PnP extractor.",
            "camera_angular_velocity_convention": "For consecutive PnP poses, log(R_camera_board_i * R_camera_board_j.T) / dt estimates camera body angular velocity in camera axes for a static board.",
            "translation_solved": False,
            "exposure_time_proven": False,
            "hardware_time_sync_proven": False,
            "tf_applied_or_modified": False,
        },
        "camera_pnp_quality": camera_quality,
        "imu_data_quality": imu_quality,
        "time_alignment_candidates": basis_results,
        "rotation_estimable_time_bases": candidate_bases,
        "time_offset_unique_candidate_bases": time_unique_bases,
        "camera_intrinsics_provisional": provisional,
        "interpretation_limit": (
            "该脚本只给出探索性 IMU→camera 旋转和发布/接收时间轴上的有效对齐候选。"
            "ROS header 是 bridge 时间；接收时标是 recorder callback 时间；两者都不是已证明的曝光时间。"
            "即使留出误差较小，也不表示硬件同步、平移、完整六自由度外参或最终 TF 已标定。"
            "camera_fit.provisional 为 true 时，所有结果仍依赖临时相机内参。"
        ),
        "excitation_guidance": (
            "目标保持固定；应在每个主要转动轴上做平滑的正反向转动，至少覆盖两个非共线轴，最好三轴均有多次启停和反向；"
            "避免仅绕竖直轴转、保持静止或一次性单向匀速。留出验证应包含训练未用的后续多轴运动。"
            "这些角速度数据不能独立确定平移。"
        ),
        "thresholds": {
            "min_camera_angular_rate_rad_s": args.min_camera_rate_rad_s,
            "max_camera_pose_gap_ms": args.max_camera_gap_ms,
            "camera_pair_separation_ms": args.camera_pair_separation_ms,
            "max_imu_interpolation_gap_ms": args.max_imu_gap_ms,
            "max_train_rmse_rad_s": args.max_train_rmse_rad_s,
            "max_holdout_rmse_rad_s": args.max_holdout_rmse_rad_s,
            "min_pairs": args.min_pairs,
            "holdout_fraction": args.holdout_fraction,
            "minimum_s2_over_s1": 0.05,
            "maximum_s1_over_s2": 20.0,
            "minimum_train_and_holdout_r_squared": 0.5,
        },
    }
    successful = bool(candidate_bases and time_unique_bases)
    return report, successful


def positive_float(value: str) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("需要有限数字") from exc
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError("数值必须为正的有限数")
    return result


def nonnegative_float(value: str) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("需要有限数字") from exc
    if not math.isfinite(result) or result < 0.0:
        raise argparse.ArgumentTypeError("数值必须为非负的有限数")
    return result


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="只离线读取联合采集 JSONL；用固定标板 PnP 姿态与原始 IMU 陀螺估计旋转/时偏候选。"
    )
    parser.add_argument("--imu", required=True, help="record_joint_calibration.py 生成的 imu.jsonl")
    parser.add_argument("--camera", required=True, help="extract_joint_camera.py 生成的 bx72_camera_pnp_v1 JSONL")
    parser.add_argument(
        "--time-bases",
        nargs="+",
        choices=tuple(TIME_BASIS_LABELS),
        default=["ros_header", "received_wall", "received_monotonic"],
        help="分别估计 ROS header、wall receive、monotonic receive 时间轴上的候选偏移",
    )
    parser.add_argument("--max-offset-ms", type=positive_float, default=300.0)
    parser.add_argument("--offset-step-ms", type=positive_float, default=5.0)
    parser.add_argument("--max-camera-gap-ms", type=positive_float, default=500.0)
    parser.add_argument("--camera-pair-separation-ms", type=nonnegative_float, default=0.0,
                        help="optional non-overlapping pose interval to reduce frame-to-frame PnP noise")
    parser.add_argument("--max-imu-gap-ms", type=positive_float, default=40.0)
    parser.add_argument("--min-camera-rate-rad-s", type=positive_float, default=0.03)
    parser.add_argument("--min-pairs", type=int, default=30)
    parser.add_argument("--holdout-fraction", type=float, default=0.25)
    parser.add_argument("--max-train-rmse-rad-s", type=positive_float, default=0.30)
    parser.add_argument("--max-holdout-rmse-rad-s", type=positive_float, default=0.35)
    return parser


def main() -> int:
    parser = make_parser()
    args = parser.parse_args()
    if args.camera_pair_separation_ms >= args.max_camera_gap_ms:
        parser.error("--camera-pair-separation-ms must be below --max-camera-gap-ms")
    if args.max_offset_ms / args.offset_step_ms < 2:
        parser.error("--max-offset-ms 必须至少覆盖两个 --offset-step-ms")
    if not math.isclose(
        args.max_offset_ms / args.offset_step_ms,
        round(args.max_offset_ms / args.offset_step_ms),
        rel_tol=0.0,
        abs_tol=1e-8,
    ):
        parser.error("为保证对称网格，--max-offset-ms 必须是 --offset-step-ms 的整数倍")
    if args.min_pairs < 12:
        parser.error("--min-pairs 至少为 12")
    if not 0.1 <= args.holdout_fraction <= 0.5:
        parser.error("--holdout-fraction 范围为 0.1..0.5")
    try:
        report, candidate = make_report(args)
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
        return 0 if candidate else 2
    except Exception as exc:
        print(f"错误：{type(exc).__name__}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
