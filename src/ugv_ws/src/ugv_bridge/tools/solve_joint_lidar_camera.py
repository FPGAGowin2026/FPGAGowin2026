#!/usr/bin/env python3
"""Estimate camera<-N10 rigid transform from manually associated board-plane returns.

This is a diagnostic solver. It never detects or labels a target automatically and
always reports the calibration as unverified. See --help for the annotation schema.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

try:
    import numpy as np
except ImportError as exc:  # pragma: no cover - exercised on the target runtime
    raise SystemExit("This tool requires NumPy.") from exc


ANNOTATION_SCHEMA = "bx72_joint_lidar_camera_annotations_v1"
RESULT_SCHEMA = "bx72_joint_lidar_camera_solution_v1"
INVALID_RANGES_MM = {0, 65535}


class InputError(Exception):
    """Raised for malformed or incomplete input records."""


def read_json(path: Path) -> Any:
    try:
        with path.open("r", encoding="utf-8") as stream:
            return json.load(stream)
    except OSError as exc:
        raise InputError(f"cannot read {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise InputError(f"invalid JSON in {path}: {exc}") from exc


def iter_jsonl(path: Path) -> Iterable[dict[str, Any]]:
    """Yield only complete JSONL records; an unterminated final line is ignored."""
    try:
        with path.open("rb") as stream:
            for raw in stream:
                if not raw.endswith(b"\n"):
                    break
                try:
                    value = json.loads(raw)
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue
                if isinstance(value, dict):
                    yield value
    except OSError as exc:
        raise InputError(f"cannot read {path}: {exc}") from exc


def file_fingerprint(path: Path) -> dict[str, Any]:
    stat = path.stat()
    return {
        "path": str(path.resolve()),
        "size_bytes": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
    }


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite_vector(value: Any, length: int, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if arr.size != length:
        raise ValueError(f"{name} must contain {length} numbers")
    arr = arr.reshape(length)
    if not np.isfinite(arr).all():
        raise ValueError(f"{name} contains non-finite numbers")
    return arr


def project_to_so3(matrix: np.ndarray) -> np.ndarray:
    u, _, vh = np.linalg.svd(matrix)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(u @ vh)
    return u @ correction @ vh


def parse_rotation(value: Any) -> tuple[np.ndarray, float]:
    arr = np.asarray(value, dtype=float)
    if arr.size != 9 or not np.isfinite(arr).all():
        raise ValueError("rotation must be a finite 3x3 matrix or 9-element row-major array")
    arr = arr.reshape((3, 3))
    orthogonal_error = float(np.linalg.norm(arr.T @ arr - np.eye(3), ord="fro"))
    determinant = float(np.linalg.det(arr))
    if determinant <= 0.0 or orthogonal_error > 0.1:
        raise ValueError("rotation is not a proper near-orthogonal matrix")
    projected = project_to_so3(arr)
    return projected, float(np.linalg.norm(projected - arr, ord="fro"))


def camera_receive_time_ns(row: dict[str, Any]) -> int | None:
    timestamps = row.get("timestamps")
    if isinstance(timestamps, dict):
        value = timestamps.get("received_monotonic_ns")
        if isinstance(value, int):
            return value
    value = row.get("received_monotonic_ns")
    return value if isinstance(value, int) else None


def camera_pose_from_row(row: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, float] | None:
    quality = row.get("quality")
    pnp = row.get("pnp")
    if not isinstance(quality, dict) or quality.get("valid") is not True:
        return None
    if not isinstance(pnp, dict) or pnp.get("status") != "ok":
        return None
    rotation = pnp.get("R_camera_board_row_major")
    translation = pnp.get("t_camera_board_m")
    if rotation is None or translation is None:
        transform = pnp.get("T_camera_board_row_major_4x4")
        if transform is None:
            return None
        transform = np.asarray(transform, dtype=float)
        if transform.size != 16 or not np.isfinite(transform).all():
            return None
        transform = transform.reshape((4, 4))
        rotation = transform[:3, :3]
        translation = transform[:3, 3]
    try:
        r, correction = parse_rotation(rotation)
        t = finite_vector(translation, 3, "t_camera_board_m")
    except (TypeError, ValueError):
        return None
    return r, t, correction


def operator_events(path: Path) -> dict[str, int]:
    found: dict[str, int] = {}
    for row in iter_jsonl(path):
        label = row.get("label")
        if not isinstance(label, str):
            continue
        stamp = row.get("received_monotonic_ns")
        if not isinstance(stamp, int):
            continue
        if label in found:
            raise InputError(f"event label {label!r} occurs more than once")
        found[label] = stamp
    return found


def resolve_time_window(spec: dict[str, Any], events: dict[str, int]) -> tuple[int, int]:
    start = spec.get("start_monotonic_ns")
    end = spec.get("end_monotonic_ns")
    if isinstance(start, int) and isinstance(end, int):
        lo, hi = start, end
    elif isinstance(spec.get("start_event"), str) and isinstance(spec.get("end_event"), str):
        start_name, end_name = spec["start_event"], spec["end_event"]
        if start_name not in events or end_name not in events:
            raise InputError(f"window references a missing event: {start_name!r}, {end_name!r}")
        lo, hi = events[start_name], events[end_name]
    elif isinstance(spec.get("event_center"), str):
        label = spec["event_center"]
        if label not in events:
            raise InputError(f"window references missing event {label!r}")
        before = spec.get("before_ms")
        after = spec.get("after_ms")
        if not isinstance(before, (int, float)) or not isinstance(after, (int, float)):
            raise InputError("event_center windows require numeric before_ms and after_ms")
        if before < 0 or after < 0:
            raise InputError("before_ms and after_ms must be non-negative")
        lo = events[label] - int(before * 1_000_000)
        hi = events[label] + int(after * 1_000_000)
    else:
        raise InputError(
            "each window needs start/end_monotonic_ns, start_event/end_event, "
            "or event_center with before_ms/after_ms"
        )
    if lo < 0 or hi <= lo:
        raise InputError("window end must be later than its non-negative start")
    return lo, hi


def normalize_angle_deg(angle: float) -> float:
    return angle % 360.0


def angle_in_sector(angle: float, sector: tuple[float, float]) -> bool:
    value = normalize_angle_deg(angle)
    lo, hi = (normalize_angle_deg(x) for x in sector)
    if lo <= hi:
        return lo <= value <= hi
    return value >= lo or value <= hi


def percentile(values: np.ndarray, q: float) -> float | None:
    if values.size == 0:
        return None
    return float(np.percentile(values, q))


def rotation_angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    relative = a.T @ b
    cosine = max(-1.0, min(1.0, (float(np.trace(relative)) - 1.0) / 2.0))
    return math.degrees(math.acos(cosine))


def mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    return project_to_so3(np.sum(rotations, axis=0))


def vector_stats(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"min": None, "median": None, "p95": None, "max": None}
    arr = np.asarray(values, dtype=float)
    return {
        "min": float(np.min(arr)),
        "median": float(np.median(arr)),
        "p95": percentile(arr, 95),
        "max": float(np.max(arr)),
    }


def robust_residual_summary(residuals: np.ndarray) -> dict[str, float | None]:
    if residuals.size == 0:
        return {
            "count": 0,
            "signed_mean_m": None,
            "rms_m": None,
            "median_abs_m": None,
            "p95_abs_m": None,
            "max_abs_m": None,
        }
    absolute = np.abs(residuals)
    return {
        "count": int(residuals.size),
        "signed_mean_m": float(np.mean(residuals)),
        "rms_m": float(np.sqrt(np.mean(residuals * residuals))),
        "median_abs_m": float(np.median(absolute)),
        "p95_abs_m": percentile(absolute, 95),
        "max_abs_m": float(np.max(absolute)),
    }


def make_window(spec: dict[str, Any], events: dict[str, int], number: int) -> dict[str, Any]:
    identifier = spec.get("id")
    if not isinstance(identifier, str) or not identifier.strip():
        raise InputError(f"window {number} needs a non-empty id")
    role = spec.get("role")
    if role not in {"train", "holdout"}:
        raise InputError(f"window {identifier!r} role must be 'train' or 'holdout'")
    if spec.get("stationary") is not True:
        raise InputError(f"window {identifier!r} must explicitly assert stationary: true")
    if spec.get("association_status") not in {"candidate", "human_confirmed"}:
        raise InputError(
            f"window {identifier!r} association_status must be 'candidate' or 'human_confirmed'"
        )
    sector = spec.get("angle_sector_deg")
    distance = spec.get("range_mm")
    if not isinstance(sector, list) or len(sector) != 2:
        raise InputError(f"window {identifier!r} needs angle_sector_deg: [start, end]")
    if not isinstance(distance, list) or len(distance) != 2:
        raise InputError(f"window {identifier!r} needs range_mm: [min, max]")
    try:
        angle_sector = tuple(float(v) for v in sector)
        range_mm = tuple(float(v) for v in distance)
    except (TypeError, ValueError) as exc:
        raise InputError(f"window {identifier!r} angle/range bounds must be numeric") from exc
    if not all(math.isfinite(v) for v in (*angle_sector, *range_mm)):
        raise InputError(f"window {identifier!r} angle/range bounds must be finite")
    if range_mm[0] < 0 or range_mm[1] <= range_mm[0]:
        raise InputError(f"window {identifier!r} range_mm must increase and be non-negative")
    if range_mm[1] >= 65535:
        raise InputError(f"window {identifier!r} range max must be below the N10 no-echo code 65535")
    lo, hi = resolve_time_window(spec, events)
    return {
        "id": identifier,
        "role": role,
        "stationary_asserted": True,
        "association_status": spec["association_status"],
        "target_id": str(spec.get("target_id", "")),
        "time_start_monotonic_ns": lo,
        "time_end_monotonic_ns": hi,
        "angle_sector_deg_raw": [angle_sector[0], angle_sector[1]],
        "range_mm": [range_mm[0], range_mm[1]],
        "min_camera_frames": int(spec.get("min_camera_frames", 3)),
        "min_lidar_points": int(spec.get("min_lidar_points", 20)),
        "spec": spec,
        "camera_frames_seen": 0,
        "camera_frames_quality_valid": 0,
        "camera_frames_used": 0,
        "camera_provisional_flags": set(),
        "camera_rotations": [],
        "camera_translations": [],
        "camera_rotation_corrections": [],
        "lidar_rows": 0,
        "lidar_points": [],
        "lidar_missing_returns": 0,
        "lidar_array_length_mismatches": 0,
        "lidar_sequence": [],
        "lidar_counters": {"dropped": [], "crc_errors": [], "length_errors": [], "framing_errors": []},
        "lidar_timestamp_offsets_us": [],
        "lidar_header_receive_offsets_ms": [],
        "failure_reasons": [],
    }


def load_camera_rows(path: Path, windows: list[dict[str, Any]]) -> dict[str, Any]:
    summary = {"rows": 0, "rows_without_monotonic_time": 0, "invalid_quality": 0, "pnp_not_ok": 0}
    for row in iter_jsonl(path):
        summary["rows"] += 1
        stamp = camera_receive_time_ns(row)
        if stamp is None:
            summary["rows_without_monotonic_time"] += 1
            continue
        touched = [w for w in windows if w["time_start_monotonic_ns"] <= stamp <= w["time_end_monotonic_ns"]]
        if not touched:
            continue
        quality = row.get("quality")
        pnp = row.get("pnp")
        if not isinstance(quality, dict) or quality.get("valid") is not True:
            summary["invalid_quality"] += 1
            for window in touched:
                window["camera_frames_seen"] += 1
            continue
        for window in touched:
            window["camera_frames_seen"] += 1
            window["camera_frames_quality_valid"] += 1
        if not isinstance(pnp, dict) or pnp.get("status") != "ok":
            summary["pnp_not_ok"] += 1
            continue
        pose = camera_pose_from_row(row)
        if pose is None:
            continue
        rotation, translation, correction = pose
        camera_fit = row.get("camera_fit")
        provisional = camera_fit.get("provisional") if isinstance(camera_fit, dict) else None
        for window in touched:
            window["camera_frames_used"] += 1
            window["camera_rotations"].append(rotation)
            window["camera_translations"].append(translation)
            window["camera_rotation_corrections"].append(correction)
            window["camera_provisional_flags"].add(provisional)
    return summary


def load_lidar_rows(path: Path, windows: list[dict[str, Any]], angle_sign: float) -> dict[str, Any]:
    summary = {"rows": 0, "rows_without_monotonic_time": 0, "unterminated_final_line_ignored": True}
    for row in iter_jsonl(path):
        summary["rows"] += 1
        stamp = row.get("received_monotonic_ns")
        if not isinstance(stamp, int):
            summary["rows_without_monotonic_time"] += 1
            continue
        touched = [w for w in windows if w["time_start_monotonic_ns"] <= stamp <= w["time_end_monotonic_ns"]]
        if not touched:
            continue
        angles = row.get("angle_cdeg")
        distances = row.get("distance_mm")
        strengths = row.get("strength")
        if not isinstance(angles, list) or not isinstance(distances, list):
            for window in touched:
                window["failure_reasons"].append("N10 angle_cdeg/distance_mm arrays missing")
            continue
        if len(angles) != len(distances):
            for window in touched:
                window["lidar_array_length_mismatches"] += 1
        for window in touched:
            window["lidar_rows"] += 1
            sequence = row.get("sequence")
            if isinstance(sequence, int):
                window["lidar_sequence"].append(sequence)
            for key in window["lidar_counters"]:
                value = row.get(key)
                if isinstance(value, (int, float)):
                    window["lidar_counters"][key].append(float(value))
            timestamp_us = row.get("timestamp_us")
            if isinstance(timestamp_us, int):
                # Arithmetic offset only; the FPGA and host clock epochs are not asserted equal.
                window["lidar_timestamp_offsets_us"].append(stamp / 1000.0 - timestamp_us)
            header = row.get("header")
            wall = row.get("received_wall_unix_ns")
            if isinstance(header, dict) and isinstance(wall, int):
                sec, nsec = header.get("sec"), header.get("nanosec")
                if isinstance(sec, int) and isinstance(nsec, int):
                    window["lidar_header_receive_offsets_ms"].append(
                        ((sec * 1_000_000_000 + nsec) - wall) / 1_000_000.0
                    )
            paired_count = min(len(angles), len(distances))
            for index in range(paired_count):
                try:
                    angle_deg = (int(angles[index]) / 100.0) % 360.0
                    distance_mm = int(distances[index])
                except (TypeError, ValueError, OverflowError):
                    continue
                if distance_mm in INVALID_RANGES_MM or distance_mm < 0:
                    window["lidar_missing_returns"] += 1
                    continue
                if not angle_in_sector(angle_deg, tuple(window["angle_sector_deg_raw"])):
                    continue
                if not (window["range_mm"][0] <= distance_mm <= window["range_mm"][1]):
                    continue
                strength = None
                if isinstance(strengths, list) and index < len(strengths):
                    try:
                        strength = int(strengths[index])
                    except (TypeError, ValueError, OverflowError):
                        pass
                theta = math.radians(angle_deg)
                x = distance_mm / 1000.0 * math.cos(theta)
                y = angle_sign * distance_mm / 1000.0 * math.sin(theta)
                window["lidar_points"].append(
                    {
                        "p_lidar_m": (x, y, 0.0),
                        "angle_deg_raw": angle_deg,
                        "range_mm": distance_mm,
                        "strength": strength,
                    }
                )
    return summary


def finalize_windows(
    windows: list[dict[str, Any]],
    max_translation_spread_m: float,
    max_rotation_spread_deg: float,
) -> None:
    for window in windows:
        reasons = window["failure_reasons"]
        if window["camera_frames_used"] < window["min_camera_frames"]:
            reasons.append(
                f"only {window['camera_frames_used']} valid camera poses; "
                f"need {window['min_camera_frames']}"
            )
        if window["lidar_rows"] == 0:
            reasons.append("no N10 packets in the time window")
        if len(window["lidar_points"]) < window["min_lidar_points"]:
            reasons.append(
                f"only {len(window['lidar_points'])} manually selected points; "
                f"need {window['min_lidar_points']}"
            )
        if window["camera_rotations"]:
            mean_r = mean_rotation(window["camera_rotations"])
            mean_t = np.mean(window["camera_translations"], axis=0)
            rotation_spread = np.asarray(
                [rotation_angle_deg(mean_r, r) for r in window["camera_rotations"]], dtype=float
            )
            translation_spread = np.linalg.norm(
                np.asarray(window["camera_translations"]) - mean_t.reshape((1, 3)), axis=1
            )
            window["plane_pose"] = {"R_camera_board": mean_r, "t_camera_board_m": mean_t}
            window["camera_pose_spread"] = {
                "translation_m": {
                    "median": float(np.median(translation_spread)),
                    "p95": percentile(translation_spread, 95),
                    "max": float(np.max(translation_spread)),
                },
                "rotation_deg": {
                    "median": float(np.median(rotation_spread)),
                    "p95": percentile(rotation_spread, 95),
                    "max": float(np.max(rotation_spread)),
                },
                "rotation_projection_correction_frobenius_max": max(
                    window["camera_rotation_corrections"], default=0.0
                ),
            }
            if percentile(translation_spread, 95) > max_translation_spread_m:
                reasons.append("camera translation varies too much for a stationary window")
            if percentile(rotation_spread, 95) > max_rotation_spread_deg:
                reasons.append("camera rotation varies too much for a stationary window")
        else:
            window["plane_pose"] = None
            window["camera_pose_spread"] = None
        seq = window["lidar_sequence"]
        sequence_gaps = 0
        sequence_duplicates = 0
        for previous, current in zip(seq, seq[1:]):
            delta = (current - previous) & 0xFFFFFFFF
            if delta == 0:
                sequence_duplicates += 1
            elif delta > 1 and delta < 0x80000000:
                sequence_gaps += delta - 1
        counters = {}
        for key, values in window["lidar_counters"].items():
            counters[key] = {
                "min": min(values) if values else None,
                "max": max(values) if values else None,
                "first": values[0] if values else None,
                "last": values[-1] if values else None,
                "distinct_values": sorted(set(values)),
            }
        ranges = [point["range_mm"] for point in window["lidar_points"]]
        angles = [point["angle_deg_raw"] for point in window["lidar_points"]]
        strengths = [point["strength"] for point in window["lidar_points"] if point["strength"] is not None]
        window["lidar_summary"] = {
            "packet_count": window["lidar_rows"],
            "selected_point_count": len(window["lidar_points"]),
            "missing_return_count_in_packet_arrays": window["lidar_missing_returns"],
            "array_length_mismatch_packets": window["lidar_array_length_mismatches"],
            "sequence_first": seq[0] if seq else None,
            "sequence_last": seq[-1] if seq else None,
            "sequence_gap_count_observed": sequence_gaps,
            "sequence_duplicate_count_observed": sequence_duplicates,
            "packet_counters": counters,
            "selected_range_mm": vector_stats([float(v) for v in ranges]),
            "selected_angle_raw_deg": {
                "min": min(angles) if angles else None,
                "max": max(angles) if angles else None,
                "sector_deg": window["angle_sector_deg_raw"],
            },
            "selected_strength": vector_stats([float(v) for v in strengths]),
            "timestamp_us_minus_receive_monotonic_us_arithmetic": vector_stats(
                window["lidar_timestamp_offsets_us"]
            ),
            "header_minus_received_wall_ms": vector_stats(window["lidar_header_receive_offsets_ms"]),
            "time_semantics_note": (
                "Offsets are arithmetic comparisons only. FPGA timestamp_us is not asserted to "
                "share the host clock epoch or to represent per-beam acquisition time."
            ),
        }
        window["usable"] = len(reasons) == 0
        window["failure_reasons"] = list(dict.fromkeys(reasons))


def points_for_windows(windows: list[dict[str, Any]], role: str) -> dict[str, np.ndarray]:
    p_values: list[tuple[float, float, float]] = []
    n_values: list[tuple[float, float, float]] = []
    q_values: list[tuple[float, float, float]] = []
    groups: list[str] = []
    counts = {w["id"]: len(w["lidar_points"]) for w in windows if w["role"] == role and w["usable"]}
    for window in windows:
        if window["role"] != role or not window["usable"]:
            continue
        mean_r = window["plane_pose"]["R_camera_board"]
        mean_t = window["plane_pose"]["t_camera_board_m"]
        normal = mean_r[:, 2]
        q = mean_t
        for point in window["lidar_points"]:
            p_values.append(point["p_lidar_m"])
            n_values.append(tuple(normal))
            q_values.append(tuple(q))
            groups.append(window["id"])
    return {
        "p": np.asarray(p_values, dtype=float).reshape((-1, 3)),
        "n": np.asarray(n_values, dtype=float).reshape((-1, 3)),
        "q": np.asarray(q_values, dtype=float).reshape((-1, 3)),
        "group": np.asarray(groups, dtype=object),
        "counts": counts,
    }


def weighted_rows(observations: dict[str, np.ndarray]) -> np.ndarray:
    counts = observations["counts"]
    groups = observations["group"]
    return np.asarray([1.0 / math.sqrt(counts[str(group)]) for group in groups], dtype=float)


def affine_design(observations: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    p, n, q = observations["p"], observations["n"], observations["q"]
    if p.shape[0] == 0:
        return np.empty((0, 9)), np.empty((0,)), np.empty((0,))
    rows = np.column_stack((p[:, 0:1] * n, p[:, 1:2] * n, n))
    rhs = np.einsum("ij,ij->i", n, q)
    return rows, rhs, weighted_rows(observations)


def matrix_rank_and_singular_values(matrix: np.ndarray) -> tuple[int, list[float], float | None]:
    if matrix.size == 0:
        return 0, [], None
    singular = np.linalg.svd(matrix, compute_uv=False)
    if singular.size == 0 or singular[0] == 0:
        return 0, [float(v) for v in singular], None
    rank = int(np.sum(singular > singular[0] * 1e-10))
    condition = float(singular[0] / singular[-1]) if singular[-1] > 0 else None
    return rank, [float(v) for v in singular], condition


def initialize_from_planes(observations: dict[str, np.ndarray]) -> dict[str, Any]:
    design, rhs, weights = affine_design(observations)
    if design.shape[0] == 0:
        return {"status": "no_points", "rank": 0, "singular_values": [], "condition_number": None}
    weighted_design = design * weights.reshape((-1, 1))
    weighted_rhs = rhs * weights
    rank, singular, condition = matrix_rank_and_singular_values(weighted_design)
    result: dict[str, Any] = {
        "status": "rank_deficient",
        "rank": rank,
        "singular_values": singular,
        "condition_number": condition,
    }
    if rank < 9 or condition is None or condition > 1e10:
        return result
    coefficients, _, _, _ = np.linalg.lstsq(weighted_design, weighted_rhs, rcond=None)
    first = coefficients[:3]
    second = coefficients[3:6]
    basis = np.column_stack((first, second))
    u, _, vh = np.linalg.svd(basis, full_matrices=False)
    orthonormal = u @ vh
    rotation = np.column_stack((orthonormal[:, 0], orthonormal[:, 1], np.cross(orthonormal[:, 0], orthonormal[:, 1])))
    if np.linalg.det(rotation) < 0.0:
        rotation[:, 2] *= -1.0
    translation = coefficients[6:9]
    # Refit translation for the projected proper rotation before nonlinear refinement.
    residual_without_t = np.einsum("ij,ij->i", observations["n"], observations["p"] @ rotation.T - observations["q"])
    t_rows = observations["n"] * weights.reshape((-1, 1))
    t_rhs = -residual_without_t * weights
    translation, _, _, _ = np.linalg.lstsq(t_rows, t_rhs, rcond=None)
    result.update({"status": "ok", "R": rotation, "t": translation})
    return result


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float)


def exp_so3(vector: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(vector))
    matrix = skew(vector)
    if theta < 1e-9:
        return np.eye(3) + matrix + 0.5 * matrix @ matrix
    return np.eye(3) + (math.sin(theta) / theta) * matrix + ((1.0 - math.cos(theta)) / (theta * theta)) * (matrix @ matrix)


def residual_and_jacobian(
    rotation: np.ndarray, translation: np.ndarray, observations: dict[str, np.ndarray]
) -> tuple[np.ndarray, np.ndarray]:
    p_camera_rotated = observations["p"] @ rotation.T
    predicted = p_camera_rotated + translation.reshape((1, 3))
    difference = predicted - observations["q"]
    residual = np.einsum("ij,ij->i", observations["n"], difference)
    rotation_jacobian = np.cross(p_camera_rotated, observations["n"])
    jacobian = np.column_stack((rotation_jacobian, observations["n"]))
    return residual, jacobian


def huber_weights(residual: np.ndarray, delta_m: float) -> np.ndarray:
    absolute = np.abs(residual)
    weights = np.ones_like(absolute)
    outside = absolute > delta_m
    weights[outside] = delta_m / absolute[outside]
    return weights


def huber_cost(residual: np.ndarray, observations: dict[str, np.ndarray], delta_m: float) -> float:
    groups = observations["group"]
    counts = observations["counts"]
    absolute = np.abs(residual)
    losses = np.where(
        absolute <= delta_m,
        0.5 * residual * residual,
        delta_m * (absolute - 0.5 * delta_m),
    )
    total = 0.0
    for group in counts:
        mask = groups == group
        if np.any(mask):
            total += float(np.mean(losses[mask]))
    return total


def optimize_transform(
    observations: dict[str, np.ndarray],
    rotation: np.ndarray,
    translation: np.ndarray,
    huber_delta_m: float,
    max_iterations: int,
) -> dict[str, Any]:
    converged = False
    current_cost = None
    completed = 0
    for iteration in range(max_iterations):
        residual, jacobian = residual_and_jacobian(rotation, translation, observations)
        robust = huber_weights(residual, huber_delta_m)
        equal_window = weighted_rows(observations)
        row_scale = np.sqrt(robust) * equal_window
        weighted_j = jacobian * row_scale.reshape((-1, 1))
        weighted_r = residual * row_scale
        delta, _, _, _ = np.linalg.lstsq(weighted_j, -weighted_r, rcond=None)
        old_cost = huber_cost(residual, observations, huber_delta_m)
        accepted = False
        step_scale = 1.0
        for _ in range(12):
            proposed_r = exp_so3(delta[:3] * step_scale) @ rotation
            proposed_t = translation + delta[3:] * step_scale
            proposed_residual, _ = residual_and_jacobian(proposed_r, proposed_t, observations)
            proposed_cost = huber_cost(proposed_residual, observations, huber_delta_m)
            if proposed_cost <= old_cost:
                rotation, translation = proposed_r, proposed_t
                current_cost = proposed_cost
                accepted = True
                break
            step_scale *= 0.5
        completed = iteration + 1
        if not accepted:
            if float(np.linalg.norm(delta)) < 1e-8:
                converged = True
            break
        if float(np.linalg.norm(delta * step_scale)) < 1e-8 or abs(old_cost - current_cost) < 1e-14:
            converged = True
            break
    residual, jacobian = residual_and_jacobian(rotation, translation, observations)
    equal_window = weighted_rows(observations)
    weighted_j = jacobian * equal_window.reshape((-1, 1))
    rank, singular, condition = matrix_rank_and_singular_values(weighted_j)
    return {
        "R": rotation,
        "t": translation,
        "residuals": residual,
        "jacobian": jacobian,
        "iterations": completed,
        "converged": converged,
        "huber_cost_per_window_sum_m2": float(current_cost if current_cost is not None else huber_cost(residual, observations, huber_delta_m)),
        "jacobian_rank": rank,
        "jacobian_singular_values": singular,
        "jacobian_condition_number": condition,
    }


def per_window_residuals(
    residuals: np.ndarray, groups: np.ndarray, window_ids: list[str]
) -> dict[str, dict[str, float | None]]:
    result: dict[str, dict[str, float | None]] = {}
    for identifier in window_ids:
        result[identifier] = robust_residual_summary(residuals[groups == identifier])
    return result


def normal_diversity(windows: list[dict[str, Any]]) -> dict[str, Any]:
    valid = [w for w in windows if w["role"] == "train" and w["usable"]]
    normals = [w["plane_pose"]["R_camera_board"][:, 2] for w in valid]
    angles = []
    for index, first in enumerate(normals):
        for second in normals[index + 1 :]:
            cosine = max(-1.0, min(1.0, float(np.dot(first, second))))
            angles.append(math.degrees(math.acos(cosine)))
    return {
        "training_window_count": len(valid),
        "max_pairwise_board_normal_angle_deg": max(angles) if angles else None,
        "min_pairwise_board_normal_angle_deg": min(angles) if angles else None,
        "pairwise_angles_deg": angles,
    }


def serializable_window(window: dict[str, Any]) -> dict[str, Any]:
    mean_pose = window["plane_pose"]
    return {
        "id": window["id"],
        "role": window["role"],
        "target_id": window["target_id"],
        "association_status": window["association_status"],
        "association_interpretation": "operator-declared only; this tool does not verify target identity",
        "stationary_asserted": window["stationary_asserted"],
        "time_monotonic_ns_inclusive": [window["time_start_monotonic_ns"], window["time_end_monotonic_ns"]],
        "angle_sector_deg_raw": window["angle_sector_deg_raw"],
        "range_mm": window["range_mm"],
        "camera_frame_counts": {
            "seen": window["camera_frames_seen"],
            "quality_valid": window["camera_frames_quality_valid"],
            "pnp_poses_used": window["camera_frames_used"],
            "provisional_flags": sorted(str(value) for value in window["camera_provisional_flags"]),
        },
        "camera_mean_board_pose": (
            {
                "R_camera_board_row_major": mean_pose["R_camera_board"].reshape(-1).tolist(),
                "t_camera_board_m": mean_pose["t_camera_board_m"].tolist(),
                "plane_normal_camera": mean_pose["R_camera_board"][:, 2].tolist(),
            }
            if mean_pose is not None
            else None
        ),
        "camera_pose_spread": window["camera_pose_spread"],
        "lidar": window["lidar_summary"],
        "usable_for_fit": window["usable"],
        "reasons_not_usable": window["failure_reasons"],
    }


def transform_matrix(rotation: np.ndarray, translation: np.ndarray) -> list[list[float]]:
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix.tolist()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Fit T_camera_lidar from board-plane constraints using manually selected N10 "
            "time windows, raw angle sectors, and ranges. Output is always unverified."
        ),
        epilog=(
            "Annotation JSON example:\n"
            "{\n"
            f'  "schema": "{ANNOTATION_SCHEMA}", "target_id": "board",\n'
            '  "angle_increasing": "ccw",\n'
            '  "windows": [{\n'
            '    "id": "pose_1_train", "role": "train", "association_status": "candidate",\n'
            '    "stationary": true, "event_center": "pose_1_done",\n'
            '    "before_ms": 500, "after_ms": 500,\n'
            '    "angle_sector_deg": [330, 25], "range_mm": [300, 1500]\n'
            '  }, {\n'
            '    "id": "pose_holdout", "role": "holdout", "association_status": "human_confirmed",\n'
            '    "stationary": true, "start_monotonic_ns": 123, "end_monotonic_ns": 456,\n'
            '    "angle_sector_deg": [340, 20], "range_mm": [300, 1500]\n'
            "  }]\n"
            "}\n\n"
            "A sector whose start is greater than its end crosses raw 0 degrees. Event windows "
            "resolve operator labels using events.jsonl. Camera header time is bridge publish "
            "time, not exposure time; N10 timestamp_us is not treated as per-beam time."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--capture-dir", type=Path, help="directory containing n10.jsonl, camera.jsonl, events.jsonl")
    parser.add_argument("--n10", type=Path, help="override N10 JSONL path")
    parser.add_argument("--camera", type=Path, help="override camera PnP JSONL path")
    parser.add_argument("--events", type=Path, help="override events JSONL path")
    parser.add_argument("--annotations", type=Path, required=True, help="operator-authored association annotations JSON")
    parser.add_argument("--output", type=Path, required=True, help="new JSON result path; existing files are never overwritten")
    parser.add_argument("--min-train-windows", type=int, default=3)
    parser.add_argument("--min-holdout-windows", type=int, default=1)
    parser.add_argument("--min-camera-frames", type=int, default=3)
    parser.add_argument("--min-lidar-points", type=int, default=20)
    parser.add_argument("--max-camera-translation-spread-m", type=float, default=0.02)
    parser.add_argument("--max-camera-rotation-spread-deg", type=float, default=2.0)
    parser.add_argument("--huber-delta-m", type=float, default=0.015)
    parser.add_argument("--max-iterations", type=int, default=50)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.capture_dir is None and (args.n10 is None or args.camera is None or args.events is None):
            raise InputError("provide --capture-dir or all of --n10, --camera, and --events")
        capture = args.capture_dir
        n10_path = args.n10 or (capture / "n10.jsonl")
        camera_path = args.camera or (capture / "camera.jsonl")
        events_path = args.events or (capture / "events.jsonl")
        for path in (n10_path, camera_path, events_path, args.annotations):
            if not path.is_file():
                raise InputError(f"input file does not exist: {path}")
        input_paths = [n10_path.resolve(), camera_path.resolve(), events_path.resolve(), args.annotations.resolve()]
        if args.output.resolve() in input_paths:
            raise InputError("output path must not replace an input file")
        if args.min_train_windows < 1 or args.min_holdout_windows < 0:
            raise InputError("minimum window counts are invalid")
        if args.min_camera_frames < 1 or args.min_lidar_points < 1:
            raise InputError("minimum frame/point counts must be positive")
        if args.max_camera_translation_spread_m <= 0 or args.max_camera_rotation_spread_deg <= 0:
            raise InputError("pose spread limits must be positive")
        if args.huber_delta_m <= 0 or args.max_iterations < 1:
            raise InputError("Huber delta and iteration limit must be positive")

        annotation = read_json(args.annotations)
        if not isinstance(annotation, dict) or annotation.get("schema") != ANNOTATION_SCHEMA:
            raise InputError(f"annotations must use schema {ANNOTATION_SCHEMA!r}")
        angle_direction = annotation.get("angle_increasing")
        if angle_direction not in {"ccw", "cw"}:
            raise InputError("annotations must explicitly set angle_increasing to 'ccw' or 'cw'")
        angle_sign = 1.0 if angle_direction == "ccw" else -1.0
        target_id = annotation.get("target_id")
        if not isinstance(target_id, str) or not target_id.strip():
            raise InputError("annotations need a non-empty target_id")
        specs = annotation.get("windows")
        if not isinstance(specs, list) or not specs:
            raise InputError("annotations need a non-empty windows array")
        events = operator_events(events_path)
        windows = [make_window(spec, events, i + 1) for i, spec in enumerate(specs) if isinstance(spec, dict)]
        if len(windows) != len(specs):
            raise InputError("every windows entry must be an object")
        ids = [w["id"] for w in windows]
        if len(ids) != len(set(ids)):
            raise InputError("window ids must be unique")
        for window in windows:
            if not window["target_id"]:
                window["target_id"] = target_id
            if window["min_camera_frames"] <= 0 or window["min_lidar_points"] <= 0:
                raise InputError(f"window {window['id']!r} minimum counts must be positive")

        camera_input_summary = load_camera_rows(camera_path, windows)
        lidar_input_summary = load_lidar_rows(n10_path, windows, angle_sign)
        for window in windows:
            if window["min_camera_frames"] == 3:
                window["min_camera_frames"] = args.min_camera_frames
            if window["min_lidar_points"] == 20:
                window["min_lidar_points"] = args.min_lidar_points
        finalize_windows(
            windows,
            args.max_camera_translation_spread_m,
            args.max_camera_rotation_spread_deg,
        )
        train_windows = [w for w in windows if w["role"] == "train"]
        holdout_windows = [w for w in windows if w["role"] == "holdout"]
        usable_train = [w for w in train_windows if w["usable"]]
        usable_holdout = [w for w in holdout_windows if w["usable"]]
        train_obs = points_for_windows(windows, "train")
        holdout_obs = points_for_windows(windows, "holdout")
        init = initialize_from_planes(train_obs)
        fit = None
        transform = None
        estimate_status = "not_estimable"
        fit_failure_reasons = []
        enough_windows = len(usable_train) >= args.min_train_windows and len(usable_holdout) >= args.min_holdout_windows
        if not enough_windows:
            fit_failure_reasons.append(
                f"need at least {args.min_train_windows} usable train and "
                f"{args.min_holdout_windows} usable holdout windows"
            )
        if init.get("status") != "ok":
            fit_failure_reasons.append(
                f"affine initialization unavailable (design rank {init.get('rank')} of 9)"
            )
        if enough_windows and init.get("status") == "ok":
            fit = optimize_transform(
                train_obs,
                init["R"],
                init["t"],
                args.huber_delta_m,
                args.max_iterations,
            )
            if fit["jacobian_rank"] < 6 or (fit["jacobian_condition_number"] or math.inf) > 1e8:
                estimate_status = "unobservable_candidate"
                fit_failure_reasons.append(
                    f"training Jacobian rank/conditioning insufficient ({fit['jacobian_rank']} of 6)"
                )
            else:
                estimate_status = "candidate_estimate"
            transform = {
                "rotation": fit["R"],
                "translation": fit["t"],
            }
        elif init.get("status") == "ok":
            # Retain a diagnostic initializer only when there is enough training evidence.
            transform = {"rotation": init["R"], "translation": init["t"]}
            estimate_status = "candidate_initializer_only"

        provisional_seen = any(True in window["camera_provisional_flags"] for window in windows)
        provisional_unknown = any(None in window["camera_provisional_flags"] for window in windows if window["camera_frames_used"])
        overall_reason = [
            "camera intrinsics/PnP are provisional or their provenance is unknown",
            "camera header and N10 timestamps are not exposure/per-beam acquisition timestamps",
            "target identity depends on operator annotations and is not verified by this tool",
        ]
        if estimate_status != "candidate_estimate":
            overall_reason.extend(fit_failure_reasons)

        train_metrics: dict[str, Any] = {}
        holdout_metrics: dict[str, Any] = {}
        if fit is not None:
            train_metrics = {
                "aggregate": robust_residual_summary(fit["residuals"]),
                "by_window": per_window_residuals(
                    fit["residuals"], train_obs["group"], [w["id"] for w in usable_train]
                ),
            }
            if transform is not None and holdout_obs["p"].shape[0]:
                holdout_residual, _ = residual_and_jacobian(transform["rotation"], transform["translation"], holdout_obs)
                holdout_metrics = {
                    "aggregate": robust_residual_summary(holdout_residual),
                    "by_window": per_window_residuals(
                        holdout_residual, holdout_obs["group"], [w["id"] for w in usable_holdout]
                    ),
                }

        rotation_out = transform["rotation"] if transform is not None else None
        translation_out = transform["translation"] if transform is not None else None
        output = {
            "schema": RESULT_SCHEMA,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "calibration_status": "unverified",
            "estimate_status": estimate_status,
            "unverified_reasons": overall_reason,
            "coordinate_conventions": {
                "transform_name": "T_camera_lidar",
                "mapping": "p_camera = R_camera_lidar * p_lidar + t_camera_lidar",
                "camera_frame": "camera optical: x right, y down, z forward",
                "lidar_frame": "n10_raw: x along raw angle 0, y follows configured positive angle, z = x cross y",
                "lidar_point_equation": "(range_m*cos(theta), sign*range_m*sin(theta), 0), theta from raw angle_cdeg",
                "raw_angle_increasing_assumption": angle_direction,
                "angle_increasing_assumption_source": "operator annotation; not inferred from samples",
                "board_frame": "board x along ID+6, y along ID+1, z=x cross y; metres",
                "camera_board_mapping": "p_camera = R_camera_board * p_board + t_camera_board",
                "board_plane_equation_in_camera": "n_camera dot (p_camera - t_camera_board) = 0, n_camera = R_camera_board[:,2]",
                "handedness": "right-handed frames; lidar z is defined by x cross y from the selected angle sign",
                "association_method": "only manually annotated time windows, raw angle sectors, and range intervals are used",
                "missing_echo_policy": "N10 distance 0 and 65535 are excluded and reported as missing; never interpreted as free space or target absence",
                "time_alignment_limits": "Windows use subscriber receive_monotonic_ns. Camera ROS header is bridge publish time, not exposure; N10 timestamp_us is not treated as per-beam time.",
            },
            "inputs": {
                "n10": file_fingerprint(n10_path),
                "camera": file_fingerprint(camera_path),
                "events": file_fingerprint(events_path),
                "annotations": {**file_fingerprint(args.annotations), "sha256": sha256_file(args.annotations)},
                "camera_fit_provisional_seen": provisional_seen,
                "camera_fit_provenance_unknown_seen": provisional_unknown,
                "camera_input_summary": camera_input_summary,
                "n10_input_summary": lidar_input_summary,
            },
            "annotation_summary": {
                "target_id": target_id,
                "window_count": len(windows),
                "association_statuses": sorted({w["association_status"] for w in windows}),
                "operator_event_count": len(events),
            },
            "windows": [serializable_window(window) for window in windows],
            "observability": {
                "training_pose_diversity": normal_diversity(windows),
                "affine_initialization": {
                    "status": init.get("status"),
                    "design_rank": init.get("rank"),
                    "required_rank": 9,
                    "singular_values": init.get("singular_values", []),
                    "condition_number": init.get("condition_number"),
                },
                "training_jacobian": (
                    {
                        "rank": fit["jacobian_rank"],
                        "required_rank": 6,
                        "singular_values": fit["jacobian_singular_values"],
                        "condition_number": fit["jacobian_condition_number"],
                    }
                    if fit is not None
                    else None
                ),
            },
            "T_camera_lidar": (
                {
                    "R_row_major": rotation_out.reshape(-1).tolist(),
                    "t_m": translation_out.tolist(),
                    "matrix_row_major_4x4": transform_matrix(rotation_out, translation_out),
                }
                if transform is not None
                else None
            ),
            "fit": (
                {
                    "iterations": fit["iterations"],
                    "converged": fit["converged"],
                    "huber_delta_m": args.huber_delta_m,
                    "training_residuals": train_metrics,
                    "holdout_residuals": holdout_metrics,
                    "fit_failure_reasons": fit_failure_reasons,
                }
                if fit is not None
                else {
                    "training_residuals": None,
                    "holdout_residuals": None,
                    "fit_failure_reasons": fit_failure_reasons,
                }
            ),
            "limits": [
                "No N10 target detection or wall classification is performed.",
                "A fit is never declared a passed calibration; camera intrinsics are provisional or unknown, and sensor acquisition timing is not synchronized here.",
                "Quality thresholds for final acceptance must be chosen and reviewed outside this diagnostic output.",
            ],
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        try:
            with args.output.open("x", encoding="utf-8") as stream:
                json.dump(output, stream, indent=2, ensure_ascii=False, allow_nan=False)
                stream.write("\n")
        except FileExistsError as exc:
            raise InputError(f"output already exists; refusing to overwrite: {args.output}") from exc
        except OSError as exc:
            raise InputError(f"cannot write output {args.output}: {exc}") from exc
        print(f"Wrote {args.output}")
        print(f"estimate_status={estimate_status}; calibration_status=unverified")
        if fit is not None:
            print(
                "training residual RMS/p95 abs (m): "
                f"{train_metrics['aggregate']['rms_m']} / {train_metrics['aggregate']['p95_abs_m']}"
            )
            if holdout_metrics:
                print(
                    "holdout residual RMS/p95 abs (m): "
                    f"{holdout_metrics['aggregate']['rms_m']} / {holdout_metrics['aggregate']['p95_abs_m']}"
                )
        return 0
    except InputError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
