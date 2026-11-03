#!/usr/bin/env python3
"""Extract per-frame AprilGrid PnP poses from an existing joint capture.

Input is a record_joint_calibration.py session directory. No ROS subscription,
UDP socket, or vehicle command is opened. One output JSON object is written for
every selected camera.jsonl line, including malformed or rejected frames.
Optional motion selection preserves the original recording and reports how
many frames were skipped; it does not change the per-frame quality gates.

Output schema bx72_camera_pnp_v1:
  timestamps.ros_header        ROS bridge publication stamp (not exposure time)
  timestamps.received_*        host subscriber callback receive times
  detections.markers           all detected IDs and original-image pixel corners
  quality.valid                hard per-frame detection/PnP/reprojection gate
  pnp.T_camera_board_*          board-to-camera transform from solvePnP

T_camera_board maps AprilGrid board coordinates into the ROS camera optical
frame: p_camera = R_camera_board @ p_board + t_camera_board. Camera axes are
x right, y down, z forward. Board axes are x along ID+6, y along ID+1, z=x cross
y, with the origin at the top-left outer corner of tag 0. Translation is metres.
"""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from calibrate_camera_aprilgrid import detect_aprilgrid, object_corners


SCHEMA = "bx72_camera_pnp_v1"
TOOL_VERSION = "1.0"
WIDTH = 640
HEIGHT = 360
EXPECTED_LAYOUT = "bx72_column_major_bl_tl_tr_br_v1"
EXPECTED_CORNERS = ["bottom_left", "top_left", "top_right", "bottom_right"]
FRAME_RE = re.compile(r"frame_(\d+)\.bgr8$")


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def parse_camera_fit(path: Path) -> tuple[dict[str, Any], np.ndarray, np.ndarray, dict[str, Any]]:
    raw = path.read_bytes()
    fit = json.loads(raw.decode("utf-8"))
    target = fit.get("target", {})
    params = fit.get("camera_info_parameters", {})
    if target.get("layout_id") != EXPECTED_LAYOUT:
        raise ValueError(f"camera fit layout mismatch: {target.get('layout_id')!r}")
    if target.get("corner_order") != EXPECTED_CORNERS:
        raise ValueError(f"camera fit corner order mismatch: {target.get('corner_order')!r}")
    if target.get("tag_family") != "tag36h11" or target.get("black_border_bits") != 2:
        raise ValueError("camera fit target must be tag36h11 with two black border bits")
    if params.get("image_width") != WIDTH or params.get("image_height") != HEIGHT:
        raise ValueError("camera fit must match 640x360 image mode")
    if params.get("distortion_model") != "plumb_bob":
        raise ValueError("only OpenCV plumb_bob camera fits are supported")

    matrix = np.asarray(params.get("camera_matrix"), dtype=np.float64)
    distortion = np.asarray(params.get("distortion_coefficients"), dtype=np.float64).reshape(-1)
    if matrix.size != 9 or not np.isfinite(matrix).all():
        raise ValueError("camera_matrix must contain 9 finite values")
    matrix = matrix.reshape(3, 3)
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise ValueError("camera focal lengths must be positive")
    if distortion.size not in (4, 5, 8, 12, 14) or not np.isfinite(distortion).all():
        raise ValueError("distortion_coefficients must contain 4, 5, 8, 12, or 14 finite values")

    train_views = int(fit.get("train", {}).get("view_count", 0))
    holdout_views = int(fit.get("holdout", {}).get("view_count", 0))
    # The supplied fit is explicitly provisional. Preserve that state even if
    # a later fit file omits a status field or contains a low pixel RMS.
    provisional = bool(fit.get("provisional", True))
    fit_info = {
        "source_path": str(path.resolve()),
        "sha256": sha256(raw),
        "created_utc": fit.get("created_utc"),
        "status": "provisional" if provisional else "provided_final",
        "provisional": provisional,
        "applied_to_bridge": False,
        "train_views": train_views,
        "holdout_views": holdout_views,
        "quality_warnings": fit.get("quality_warnings", []),
        "K": matrix.reshape(-1).astype(float).tolist(),
        "D": distortion.astype(float).tolist(),
        "distortion_model": "plumb_bob",
    }
    return fit, matrix, distortion, fit_info


def target_geometry(fit: dict[str, Any]) -> dict[str, Any]:
    spec = fit["target"]
    tag_size = float(spec.get("tag_size_m_used_for_fit", spec.get("tag_size_m")))
    spacing = float(spec.get("tag_spacing_ratio_used_for_fit", spec.get("tag_spacing_ratio")))
    if not math.isfinite(tag_size) or tag_size <= 0.0:
        raise ValueError("camera fit tagSize must be positive")
    if not math.isfinite(spacing) or not 0.0 < spacing <= 1.0:
        raise ValueError("camera fit tagSpacing must be in (0, 1]")
    return {
        "tag_family": "tag36h11",
        "rows": 6,
        "cols": 6,
        "tag_size_m": tag_size,
        "tag_spacing_ratio": spacing,
        "tag_gap_m_nominal": tag_size * spacing,
        "tag_pitch_m_nominal": tag_size * (1.0 + spacing),
        "black_border_bits": 2,
        "layout_id": EXPECTED_LAYOUT,
        "id_to_grid": "column=id//6, row=id%6",
        "corner_order": EXPECTED_CORNERS,
        "board_frame": {
            "origin": "top-left outer corner of tag 0",
            "x_axis": "toward ID+6 (increasing column)",
            "y_axis": "toward ID+1 (increasing row)",
            "z_axis": "x cross y; faces the visible target",
            "unit": "metre",
        },
        "print_scale_independently_measured": bool(
            spec.get("print_scale_independently_measured", False)),
    }


def blank_pose(status: str = "not_attempted", reason: str | None = None) -> dict[str, Any]:
    return {
        "status": status,
        "failure_reason": reason,
        "convention": "p_camera = R_camera_board @ p_board + t_camera_board",
        "camera_frame": "ROS header.frame_id; optical x-right, y-down, z-forward",
        "board_frame": "AprilGrid axes and origin are defined in target.board_frame",
        "rvec_axis_angle_rad": None,
        "R_camera_board_row_major": None,
        "t_camera_board_m": None,
        "T_camera_board_row_major_4x4": None,
        "translation_unit": "metre",
        "pose_uncertainty": {
            "estimated": False,
            "reason": "solvePnP pixel residual is not a pose covariance estimate",
        },
    }


def timestamps_for(record: dict[str, Any] | None) -> dict[str, Any]:
    record = record or {}
    header = record.get("header")
    header_fields = None
    if isinstance(header, dict):
        header_fields = {
            "sec": header.get("sec"),
            "nanosec": header.get("nanosec"),
            "frame_id": header.get("frame_id"),
        }
    wall = record.get("received_wall_unix_ns")
    mono = record.get("received_monotonic_ns")
    header_valid = (
        isinstance(header_fields, dict)
        and isinstance(header_fields.get("sec"), int)
        and isinstance(header_fields.get("nanosec"), int)
        and header_fields["sec"] >= 0
        and 0 <= header_fields["nanosec"] < 1_000_000_000
    )
    receive_valid = (
        isinstance(wall, int) and wall > 0 and isinstance(mono, int) and mono > 0
    )
    return {
        "ros_header": header_fields,
        "received_wall_unix_ns": wall,
        "received_monotonic_ns": mono,
        "header_valid": bool(header_valid),
        "receive_valid": bool(receive_valid),
        "semantics": {
            "ros_header": "bridge publication time; not measured sensor exposure time",
            "received_wall_unix_ns": "host subscriber callback wall-clock receive time",
            "received_monotonic_ns": "host subscriber callback monotonic receive time",
            "synchronization": "no hardware time synchronization is assumed",
        },
    }


def frame_failure(index: int, record: dict[str, Any] | None, reason: str,
                  fit_info: dict[str, Any], target: dict[str, Any]) -> dict[str, Any]:
    record = record or {}
    file_name = record.get("file")
    match = FRAME_RE.search(str(file_name or ""))
    return {
        "schema": SCHEMA,
        "tool_version": TOOL_VERSION,
        "record_index": index,
        "frame_number": int(match.group(1)) if match else None,
        "source": {"file": file_name, "sha256": record.get("sha256")},
        "image": {k: record.get(k) for k in ("width", "height", "encoding", "step", "is_bigendian")},
        "timestamps": timestamps_for(record),
        "detections": {"ids": [], "markers": [], "detected_tag_count": 0, "corner_count": 0},
        "quality": {
            "valid": False,
            "reasons": [reason],
            "detector_geometry_ok": False,
            "tag_count": 0,
            "corner_count": 0,
            "grid_center_rms_px": None,
            "grid_center_inlier_ratio": None,
            "corner_order_rms_px": None,
            "reprojection": None,
        },
        "pnp": blank_pose("not_attempted", reason),
        "camera_fit": fit_info,
        "target": target,
    }


def safe_image_path(session_dir: Path, relative_name: str) -> Path:
    root = session_dir.resolve()
    path = (root / relative_name).resolve()
    if path != root and root not in path.parents:
        raise ValueError("image path escapes session directory")
    return path


def extract_frame(
    index: int,
    record: dict[str, Any],
    session_dir: Path,
    fit_info: dict[str, Any],
    target: dict[str, Any],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    min_tags: int,
    rms_limit: float,
    p95_limit: float,
    max_limit: float,
) -> dict[str, Any]:
    relative_name = str(record.get("file") or "")
    match = FRAME_RE.search(relative_name)
    frame_number = int(match.group(1)) if match else None
    row = {
        "schema": SCHEMA,
        "tool_version": TOOL_VERSION,
        "record_index": index,
        "frame_number": frame_number,
        "source": {"file": relative_name, "sha256": record.get("sha256")},
        "image": {k: record.get(k) for k in ("width", "height", "encoding", "step", "is_bigendian")},
        "timestamps": timestamps_for(record),
        "detections": {"ids": [], "markers": [], "detected_tag_count": 0, "corner_count": 0},
        "quality": {
            "valid": False,
            "reasons": [],
            "detector_geometry_ok": False,
            "tag_count": 0,
            "corner_count": 0,
            "grid_center_rms_px": None,
            "grid_center_inlier_ratio": None,
            "corner_order_rms_px": None,
            "reprojection": None,
        },
        "pnp": blank_pose(),
        "camera_fit": fit_info,
        "target": target,
    }
    reasons: list[str] = []
    if not row["timestamps"]["header_valid"]:
        reasons.append("invalid_or_missing_ros_header")
    if not row["timestamps"]["receive_valid"]:
        reasons.append("missing_receive_timestamps")
    image_format_ok = (record.get("width"), record.get("height"), record.get("encoding")) == (WIDTH, HEIGHT, "bgr8")
    if not image_format_ok:
        reasons.append("unexpected_image_format")
    step = record.get("step")
    if not isinstance(step, int) or step < WIDTH * 3:
        reasons.append("invalid_image_step")
    if not image_format_ok or not isinstance(step, int) or step < WIDTH * 3:
        row["quality"]["reasons"] = reasons
        row["pnp"] = blank_pose("not_attempted", ";".join(reasons))
        return row

    try:
        image_path = safe_image_path(session_dir, relative_name)
    except (ValueError, OSError):
        row["quality"]["reasons"] = ["invalid_image_path"]
        row["pnp"] = blank_pose("not_attempted", "invalid_image_path")
        return row
    try:
        raw = image_path.read_bytes()
    except OSError:
        row["quality"]["reasons"] = ["missing_image_file"]
        row["pnp"] = blank_pose("not_attempted", "missing_image_file")
        return row
    if len(raw) != int(step) * HEIGHT:
        row["quality"]["reasons"] = ["raw_frame_size_mismatch"]
        row["pnp"] = blank_pose("not_attempted", "raw_frame_size_mismatch")
        return row
    actual_hash = sha256(raw)
    if record.get("sha256") != actual_hash:
        row["quality"]["reasons"] = ["raw_frame_sha256_mismatch"]
        row["pnp"] = blank_pose("not_attempted", "raw_frame_sha256_mismatch")
        return row

    try:
        raw_rows = np.frombuffer(raw, dtype=np.uint8).reshape(HEIGHT, step)
        bgr = raw_rows[:, :WIDTH * 3].reshape(HEIGHT, WIDTH, 3).copy()
        detection = detect_aprilgrid(
            bgr, target["tag_size_m"], target["tag_spacing_ratio"], scale=2)
    except cv2.error as exc:
        row["quality"]["reasons"] = [f"detection_error:{type(exc).__name__}:{exc}"]
        row["pnp"] = blank_pose("not_attempted", "detection_error")
        return row

    detections = detection["records"]
    row["detections"] = {
        "ids": detection["detected_ids"],
        "markers": [
            {
                "id": int(marker["id"]),
                "grid_column": marker.get("grid_column"),
                "grid_row": marker.get("grid_row"),
                "corners_px": marker["corners_used_original_px"],
                "corner_order": EXPECTED_CORNERS,
            }
            for marker in detections
        ],
        "detected_tag_count": int(detection["detected_tags"]),
        "corner_count": int(detection["detected_corners"]),
        "unique_valid_ids": detection["unique_valid_ids"],
        "duplicate_ids": detection["duplicate_ids"],
        "out_of_range_ids": detection["out_of_range_ids"],
        "detector": detection["detector"],
    }
    geometry = detection["geometry_check"]
    center_check = geometry["grid_center_homography"]
    corner_check = geometry["corner_order_homography"]
    ids = detection["unique_valid_ids"]
    row["quality"].update({
        "detector_geometry_ok": bool(geometry["geometry_ok"]),
        "tag_count": len(ids),
        "corner_count": int(detection["detected_corners"]),
        "grid_center_rms_px": center_check.get("rms_px"),
        "grid_center_inlier_ratio": center_check.get("inlier_ratio"),
        "corner_order_rms_px": corner_check.get("rms_px"),
        "corner_order_max_px": corner_check.get("max_px"),
    })
    pnp_gate_reasons = []
    if not geometry["geometry_ok"]:
        pnp_gate_reasons.append("grid_or_corner_order_check_failed")
    if len(ids) < min_tags:
        pnp_gate_reasons.append("too_few_unique_tags")
    if len(detection["duplicate_ids"]):
        pnp_gate_reasons.append("duplicate_tag_ids")
    if len(detection["out_of_range_ids"]):
        pnp_gate_reasons.append("tag_id_out_of_range")
    reasons.extend(pnp_gate_reasons)
    if pnp_gate_reasons:
        row["quality"]["reasons"] = reasons
        row["pnp"] = blank_pose("not_attempted", ";".join(pnp_gate_reasons))
        return row

    marker_by_id = {
        int(marker["id"]): marker
        for marker in detections
        if int(marker["id"]) in ids
    }
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    used_ids = []
    for tag_id in ids:
        marker = marker_by_id.get(int(tag_id))
        if marker is None:
            continue
        object_points.extend(object_corners(
            int(tag_id), target["tag_size_m"], target["tag_spacing_ratio"]))
        image_points.extend(np.asarray(marker["corners_used_original_px"], dtype=np.float32))
        used_ids.append(int(tag_id))
    object_array = np.asarray(object_points, dtype=np.float32).reshape(-1, 3)
    image_array = np.asarray(image_points, dtype=np.float32).reshape(-1, 2)
    row["detections"]["used_ids"] = used_ids
    row["detections"]["used_corner_count"] = len(image_array)
    if len(used_ids) < min_tags or len(image_array) < 4 * min_tags:
        row["quality"]["reasons"] = ["insufficient_unique_points"]
        row["pnp"] = blank_pose("not_attempted", "insufficient_unique_points")
        return row

    try:
        ok, rvec, tvec = cv2.solvePnP(
            object_array, image_array, camera_matrix, distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
    except cv2.error as exc:
        row["quality"]["reasons"] = [f"solvepnp_error:{exc}"]
        row["pnp"] = blank_pose("failed", "solvepnp_error")
        return row
    if not ok:
        row["quality"]["reasons"] = ["solvepnp_returned_false"]
        row["pnp"] = blank_pose("failed", "solvepnp_returned_false")
        return row

    rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
    tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1)
    rotation, _ = cv2.Rodrigues(rvec)
    camera_points = (rotation @ object_array.astype(np.float64).T + tvec).T
    depths = camera_points[:, 2]
    projected, _ = cv2.projectPoints(object_array, rvec, tvec, camera_matrix, distortion)
    pixel_errors = np.linalg.norm(projected.reshape(-1, 2) - image_array, axis=1)
    reprojection = {
        "rms_px": float(np.sqrt(np.mean(pixel_errors ** 2))),
        "median_px": float(np.median(pixel_errors)),
        "p95_px": float(np.percentile(pixel_errors, 95)),
        "max_px": float(np.max(pixel_errors)),
        "corner_count": int(len(pixel_errors)),
    }
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = tvec.reshape(3)
    if not np.isfinite(transform).all():
        row["quality"]["reasons"] = ["nonfinite_pose"]
        row["pnp"] = blank_pose("failed", "nonfinite_pose")
        return row
    if not np.all(depths > 0.0):
        reasons.append("nonpositive_camera_depth")
    if reprojection["rms_px"] > rms_limit:
        reasons.append("reprojection_rms_exceeded")
    if reprojection["p95_px"] > p95_limit:
        reasons.append("reprojection_p95_exceeded")
    if reprojection["max_px"] > max_limit:
        reasons.append("reprojection_max_exceeded")
    row["quality"].update({
        "valid": not reasons,
        "reasons": reasons,
        "reprojection": reprojection,
        "positive_depth_all_corners": bool(np.all(depths > 0.0)),
    })
    row["pnp"] = {
        "status": "ok",
        "failure_reason": None,
        "convention": "p_camera = R_camera_board @ p_board + t_camera_board",
        "camera_frame": "ROS header.frame_id; optical x-right, y-down, z-forward",
        "board_frame": "AprilGrid origin at tag 0 upper-left outer corner; x=ID+6, y=ID+1, z=x cross y",
        "rvec_axis_angle_rad": rvec.reshape(3).astype(float).tolist(),
        "R_camera_board_row_major": rotation.reshape(-1).astype(float).tolist(),
        "R_camera_board": rotation.astype(float).tolist(),
        "t_camera_board_m": tvec.reshape(3).astype(float).tolist(),
        "T_camera_board_row_major_4x4": transform.reshape(-1).astype(float).tolist(),
        "T_camera_board": transform.astype(float).tolist(),
        "translation_unit": "metre",
        "camera_depth_min_m": float(np.min(depths)),
        "camera_depth_max_m": float(np.max(depths)),
        "pose_uncertainty": {
            "estimated": False,
            "reason": "solvePnP pixel residual is not a pose covariance estimate",
        },
    }
    return row


def count_lines(path: Path) -> int:
    with path.open("rb") as stream:
        return sum(1 for line in stream if line.strip())


def create_output_dir(session_dir: Path, output_dir: str | None, fit_hash: str) -> Path:
    if output_dir:
        path = Path(output_dir).expanduser().resolve()
        path.mkdir(parents=True, exist_ok=False)
        return path
    stem = "camera_pnp_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ") + "_" + fit_hash[:8]
    path = session_dir / stem
    suffix = 1
    while path.exists():
        path = session_dir / f"{stem}_{suffix:02d}"
        suffix += 1
    path.mkdir(parents=False, exist_ok=False)
    return path


def motion_windows(session_dir: Path, threshold: float, padding: float) -> dict[str, Any]:
    """Find candidate motion intervals from recorded IMU, retaining event neighborhoods."""
    imu_path = session_dir / "imu.jsonl"
    samples = []
    with imu_path.open("r", encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            samples.append((int(row["received_monotonic_ns"]), [int(v) for v in row["gyro_raw"]]))
    if not samples:
        raise ValueError("IMU log is empty; motion selection unavailable")
    raw = np.asarray([item[1] for item in samples], dtype=np.float64)
    bias = np.median(raw, axis=0)
    rates = np.linalg.norm((raw - bias) * (0.0175 * np.pi / 180.0), axis=1)
    pad_ns = int(padding * 1e9)
    intervals = [(samples[i][0] - pad_ns, samples[i][0] + pad_ns)
                 for i in np.flatnonzero(rates > threshold)]
    events = session_dir / "events.jsonl"
    event_count = 0
    if events.exists():
        with events.open("r", encoding="utf-8") as stream:
            for line in stream:
                event = json.loads(line)
                if event.get("event") == "operator_mark":
                    t = int(event["received_monotonic_ns"])
                    intervals.append((t - pad_ns, t + pad_ns))
                    event_count += 1
    merged: list[list[int]] = []
    for a, b in sorted(intervals):
        if merged and a <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], b)
        else:
            merged.append([a, b])
    return {"method": "IMU gyro norm above threshold plus operator marks, with temporal padding and periodic idle frames",
            "gyro_median_raw_counts": bias.tolist(), "gyro_threshold_rad_s": threshold,
            "padding_s": padding, "operator_marks": event_count, "intervals_monotonic_ns": merged,
            "active_imu_samples": int(np.count_nonzero(rates > threshold))}


def run(args: argparse.Namespace) -> int:
    session_dir = Path(args.session).expanduser().resolve()
    camera_jsonl = session_dir / "camera.jsonl"
    summary_path = session_dir / "summary.json"
    session_path = session_dir / "session.json"
    if not camera_jsonl.is_file():
        raise FileNotFoundError(f"camera.jsonl not found: {camera_jsonl}")
    fit_path = Path(args.camera_fit).expanduser().resolve()
    fit, camera_matrix, distortion, fit_info = parse_camera_fit(fit_path)
    target = target_geometry(fit)
    fit_sha = fit_info["sha256"]
    total = count_lines(camera_jsonl)
    selection = motion_windows(session_dir, args.motion_threshold_rad_s, args.motion_padding_s) if args.motion_only else None
    interval_starts = [item[0] for item in selection["intervals_monotonic_ns"]] if selection else []
    last_idle_ns = None
    output_dir = create_output_dir(session_dir, args.output_dir, fit_sha)
    output_frames = output_dir / "frames.jsonl"
    output_summary = output_dir / "summary.json"
    started = time.time_ns()
    counts = {"processed": 0, "valid": 0, "failed": 0, "skipped_by_selection": 0}
    failure_counts: dict[str, int] = {}
    record_errors = 0
    last_progress = time.monotonic()
    print(f"Processing {total} camera.jsonl lines from {session_dir}", flush=True)
    print(f"Camera fit: {fit_path} (provisional={fit_info['provisional']})", flush=True)
    print(f"Output: {output_dir}", flush=True)

    with camera_jsonl.open("r", encoding="utf-8") as source, output_frames.open("x", encoding="utf-8", buffering=65536) as output:
        for index, line in enumerate(source):
            if not line.strip():
                continue
            record = None
            try:
                record = json.loads(line)
                if not isinstance(record, dict):
                    raise ValueError("camera record must be a JSON object")
                if selection is not None:
                    t = int(record["received_monotonic_ns"])
                    i = bisect.bisect_right(interval_starts, t) - 1
                    in_window = i >= 0 and t <= selection["intervals_monotonic_ns"][i][1]
                    idle_due = last_idle_ns is None or t - last_idle_ns >= int(args.idle_every_s * 1e9)
                    if not in_window and not idle_due:
                        counts["skipped_by_selection"] += 1
                        continue
                    if not in_window:
                        last_idle_ns = t
            except json.JSONDecodeError as exc:
                record_errors += 1
                result = frame_failure(index, None, f"malformed_json:{exc.msg}", fit_info, target)
            except (ValueError, TypeError, KeyError) as exc:
                record_errors += 1
                result = frame_failure(index, record if isinstance(record, dict) else None,
                                       f"record_error:{type(exc).__name__}:{exc}", fit_info, target)
            else:
                result = extract_frame(
                    index, record, session_dir, fit_info, target, camera_matrix, distortion,
                    args.min_tags, args.max_rms_px, args.max_p95_px, args.max_px,
                )
            output.write(json.dumps(result, separators=(",", ":"), ensure_ascii=False) + "\n")
            counts["processed"] += 1
            if result["quality"]["valid"]:
                counts["valid"] += 1
            else:
                counts["failed"] += 1
                for reason in result["quality"]["reasons"] or ["unspecified"]:
                    failure_counts[reason] = failure_counts.get(reason, 0) + 1
            now = time.monotonic()
            if counts["processed"] % args.progress_every == 0 or now - last_progress >= 5.0:
                print(
                    f"progress {counts['processed']}/{total}: valid={counts['valid']} failed={counts['failed']}",
                    flush=True,
                )
                last_progress = now

    input_summary = None
    if summary_path.is_file():
        try:
            input_summary = json.loads(summary_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            input_summary = {"read_error": "invalid summary.json"}
    input_session = None
    if session_path.is_file():
        try:
            session_bytes = session_path.read_bytes()
            input_session = json.loads(session_bytes.decode("utf-8"))
            session_sha = sha256(session_bytes)
        except (OSError, json.JSONDecodeError, UnicodeDecodeError):
            input_session, session_sha = {"read_error": "invalid session.json"}, None
    else:
        session_sha = None
    summary = {
        "schema": SCHEMA + "_summary",
        "tool_version": TOOL_VERSION,
        "created_utc": utc_now(),
        "source_session": str(session_dir),
        "source_camera_jsonl": str(camera_jsonl),
        "source_camera_jsonl_sha256": sha256(camera_jsonl.read_bytes()),
        "source_session_json_sha256": session_sha,
        "source_session_metadata": input_session,
        "source_capture_summary": input_summary,
        "camera_fit": fit_info,
        "target": target,
        "output_frames": str(output_frames),
        "counts": {**counts, "input_nonempty_lines": total},
        "frame_selection": selection,
        "failure_counts": failure_counts,
        "quality_gates": {
            "minimum_unique_tags": args.min_tags,
            "detector_grid_geometry_ok_required": True,
            "maximum_reprojection_rms_px": args.max_rms_px,
            "maximum_reprojection_p95_px": args.max_p95_px,
            "maximum_reprojection_error_px": args.max_px,
            "positive_depth_required": True,
        },
        "time_semantics": {
            "ros_header": "bridge publish stamp; not exposure timestamp",
            "received_wall_unix_ns": "host subscriber callback receive time",
            "received_monotonic_ns": "host subscriber callback receive time suitable for same-host relative timing",
            "hardware_synchronization": "not established",
        },
        "pose_scope": (
            "Per-frame provisional target-to-camera poses from the supplied intrinsics; "
            "not camera-to-robot extrinsics and not a hardware time-synchronization result."
        ),
        "elapsed_wall_s": (time.time_ns() - started) / 1e9,
    }
    with output_summary.open("x", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print(
        f"complete {counts['processed']}/{total}: valid={counts['valid']} failed={counts['failed']}",
        flush=True,
    )
    print(f"frames: {output_frames}")
    print(f"summary: {output_summary}")
    return 1 if record_errors else 0


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Offline AprilGrid PnP extraction for a record_joint_calibration session")
    parser.add_argument("--session", required=True, help="joint capture directory containing camera.jsonl and camera/")
    parser.add_argument("--camera-fit", required=True, help="camera fit JSON with camera_info_parameters and target")
    parser.add_argument("--output-dir", help="new output directory; must not already exist")
    parser.add_argument("--min-tags", type=int, default=6,
                        help="minimum unique IDs per accepted frame (default 6)")
    parser.add_argument("--max-rms-px", type=float, default=3.0)
    parser.add_argument("--max-p95-px", type=float, default=5.0)
    parser.add_argument("--max-px", type=float, default=10.0)
    parser.add_argument("--progress-every", type=int, default=10)
    parser.add_argument("--motion-only", action="store_true",
                        help="process IMU-motion and event windows plus periodic idle frames; preserve full source")
    parser.add_argument("--motion-threshold-rad-s", type=float, default=0.025)
    parser.add_argument("--motion-padding-s", type=float, default=1.5)
    parser.add_argument("--idle-every-s", type=float, default=15.0)
    return parser


def main() -> int:
    args = make_parser().parse_args()
    if args.min_tags < 4 or args.progress_every < 1:
        print("ERROR: min-tags must be >=4 and progress-every must be >=1", file=sys.stderr)
        return 2
    if min(args.max_rms_px, args.max_p95_px, args.max_px,
           args.motion_threshold_rad_s, args.motion_padding_s, args.idle_every_s) <= 0:
        print("ERROR: reprojection limits must be positive", file=sys.stderr)
        return 2
    try:
        return run(args)
    except OSError as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
