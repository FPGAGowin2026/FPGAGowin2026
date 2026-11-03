#!/usr/bin/env python3
"""Collect and calibrate the BX72 640x360 AprilGrid camera from ROS images.

The board coordinate system is chosen with ID increasing by 6 along board X
and by 1 along board Y. The labels are fixed to the print, not the camera
image. This tool checks the detected centers and all four corners against one
planar homography before a sample can enter a fit.

In the chosen board coordinates, the empirically verified detector corner order
is bottom-left, top-left, top-right, bottom-right. The board mapping is validated
against IDs 0, 3, 8, and 35 and then inherited by partial views in the same
dataset. Intrinsic calibration does not identify a camera-to-robot transform.

The detector upsamples grayscale 2x with INTER_CUBIC, as verified against the
saved 16:21 BX72 ROS frame. Pixel centers are mapped back to the original
640x360 image before cautious subpixel refinement.
"""

from __future__ import annotations

import argparse
from collections import deque
import hashlib
import json
import math
import queue
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np


TOOL_VERSION = "1.0"
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 360
DEFAULT_TOPIC = "/camera/image_raw"
DEFAULT_TAG_SIZE_M = 0.025
DEFAULT_TAG_SPACING = 0.3
DEFAULT_SCALE = 2
LAYOUT_ID = "bx72_column_major_bl_tl_tr_br_v1"
CORNER_ORDER = ["bottom_left", "top_left", "top_right", "bottom_right"]
ANCHOR_GRID = {0: (0, 0), 3: (0, 3), 8: (1, 2), 35: (5, 5)}


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def now_utc() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def target_spec(tag_size_m: float, tag_spacing: float) -> dict[str, Any]:
    return {
        "target_type": "aprilgrid",
        "tag_family": "tag36h11",
        "tag_rows": 6,
        "tag_cols": 6,
        "tag_size_m": float(tag_size_m),
        "tag_spacing_ratio": float(tag_spacing),
        "tag_gap_m_nominal": float(tag_size_m * tag_spacing),
        "tag_pitch_m_nominal": float(tag_size_m * (1.0 + tag_spacing)),
        "black_border_bits": 2,
        "layout_id": LAYOUT_ID,
        "id_to_grid": "column=id//6, row=id%6",
        "corner_order": CORNER_ORDER,
        "layout_anchors": {
            str(tag_id): {"column": col, "row": row}
            for tag_id, (col, row) in ANCHOR_GRID.items()
        },
        "print_scale_independently_measured": False,
        "spacing_convention": "gap_between_tags / black_square_tag_size",
        "kalibr_reference": "https://github.com/ethz-asl/kalibr/wiki/calibration-targets",
        "layout_note": (
            "Board X is chosen along the ID+6 direction and Y along ID+1. "
            "These axes are fixed to the print and may differ from another "
            "target generator's origin or axis convention."
        ),
    }


def predefined_dictionary() -> Any:
    aruco = cv2.aruco
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    return aruco.Dictionary_get(aruco.DICT_APRILTAG_36h11)


def create_detector() -> tuple[Any, Any]:
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV aruco 模块不可用；请在 ROS 环境中使用 Jetson 系统 OpenCV")
    params = (cv2.aruco.DetectorParameters_create()
              if hasattr(cv2.aruco, "DetectorParameters_create")
              else cv2.aruco.DetectorParameters())
    params.markerBorderBits = 2
    if hasattr(params, "cornerRefinementMethod"):
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    return predefined_dictionary(), params


def grid_coordinates(tag_id: int) -> tuple[int, int]:
    """Return coordinates in this dataset's board-fixed convention."""
    return tag_id // 6, tag_id % 6


def object_corners(tag_id: int, tag_size_m: float, tag_spacing: float) -> np.ndarray:
    col, row = grid_coordinates(tag_id)
    pitch = tag_size_m * (1.0 + tag_spacing)
    x0 = col * pitch
    y0 = row * pitch
    # Detector index order verified on the BX72 print: BL, TL, TR, BR.
    return np.asarray([
        [x0, y0 + tag_size_m, 0.0],
        [x0, y0, 0.0],
        [x0 + tag_size_m, y0, 0.0],
        [x0 + tag_size_m, y0 + tag_size_m, 0.0],
    ], dtype=np.float32)


def _project_homography(homography: np.ndarray, points: np.ndarray) -> np.ndarray:
    projected = cv2.perspectiveTransform(
        np.asarray(points, dtype=np.float32).reshape(-1, 1, 2), homography)
    return projected.reshape(-1, 2)


def detect_aprilgrid(
    bgr: np.ndarray,
    tag_size_m: float = DEFAULT_TAG_SIZE_M,
    tag_spacing: float = DEFAULT_TAG_SPACING,
    scale: int = DEFAULT_SCALE,
) -> dict[str, Any]:
    if bgr is None or bgr.shape != (IMAGE_HEIGHT, IMAGE_WIDTH, 3) or bgr.dtype != np.uint8:
        raise ValueError("输入必须为 640x360 uint8 BGR8")
    if scale < 1:
        raise ValueError("检测放大倍数必须至少为 1")

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    work = gray if scale == 1 else cv2.resize(
        gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    dictionary, params = create_detector()
    detector_corners, detector_ids, rejected = cv2.aruco.detectMarkers(
        work, dictionary, parameters=params)

    ids = [] if detector_ids is None else [int(v) for v in detector_ids.flatten()]
    records: list[dict[str, Any]] = []
    flat_raw: list[np.ndarray] = []
    flat_used: list[np.ndarray] = []
    max_refine_shift = 0.0
    refined_count = 0
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.01)

    for tag_id, scaled in zip(ids, detector_corners):
        # OpenCV resize maps pixel centers as (x + .5) * scale - .5.
        raw = ((np.asarray(scaled, dtype=np.float32).reshape(4, 2) + 0.5) / scale) - 0.5
        refined = raw.copy()
        refine_ok = False
        try:
            candidate = raw.reshape(-1, 1, 2).copy()
            cv2.cornerSubPix(gray, candidate, (2, 2), (-1, -1), criteria)
            candidate = candidate.reshape(4, 2)
            shift = np.linalg.norm(candidate - raw, axis=1)
            local_max = float(np.max(shift))
            max_refine_shift = max(max_refine_shift, local_max)
            # Keep raw detector corners if refinement makes a large move.
            if np.isfinite(candidate).all() and local_max <= 0.75:
                refined = candidate
                refine_ok = True
                refined_count += 1
        except cv2.error:
            pass

        record = {
            "id": tag_id,
            "grid_column": grid_coordinates(tag_id)[0] if 0 <= tag_id < 36 else None,
            "grid_row": grid_coordinates(tag_id)[1] if 0 <= tag_id < 36 else None,
            "corners_raw_original_px": raw.astype(float).tolist(),
            "corners_subpix_original_px": refined.astype(float).tolist(),
            "subpix_accepted": refine_ok,
            "corners_used_original_px": (refined if refine_ok else raw).astype(float).tolist(),
        }
        records.append(record)
        flat_raw.append(raw)
        flat_used.append(refined if refine_ok else raw)

    counts: dict[int, int] = {}
    for tag_id in ids:
        counts[tag_id] = counts.get(tag_id, 0) + 1
    duplicates = sorted(tag_id for tag_id, count in counts.items() if count > 1)
    unique_records = [
        rec for rec in records
        if 0 <= rec["id"] < 36 and counts.get(rec["id"]) == 1
    ]
    unique_ids = sorted(int(rec["id"]) for rec in unique_records)

    center_check: dict[str, Any] = {
        "available": False,
        "inliers": 0,
        "total": len(unique_records),
        "rms_px": None,
        "max_px": None,
        "inlier_ratio": 0.0,
    }
    corner_check: dict[str, Any] = {
        "available": False,
        "rms_px": None,
        "max_px": None,
        "median_tag_edge_px": None,
        "rms_limit_px": None,
        "max_limit_px": None,
    }

    if len(unique_records) >= 4:
        grid_centers = np.asarray([
            grid_coordinates(int(rec["id"])) for rec in unique_records
        ], dtype=np.float32)
        image_centers = np.asarray([
            np.mean(np.asarray(rec["corners_raw_original_px"], dtype=np.float32), axis=0)
            for rec in unique_records
        ], dtype=np.float32)
        homography, inlier_mask = cv2.findHomography(
            grid_centers, image_centers, cv2.RANSAC, 2.5)
        if homography is not None and inlier_mask is not None:
            inliers = inlier_mask.reshape(-1).astype(bool)
            predicted_centers = _project_homography(homography, grid_centers)
            center_errors = np.linalg.norm(predicted_centers - image_centers, axis=1)
            inlier_errors = center_errors[inliers]
            center_check = {
                "available": True,
                "inliers": int(np.count_nonzero(inliers)),
                "total": int(len(unique_records)),
                "rms_px": float(np.sqrt(np.mean(inlier_errors ** 2))) if len(inlier_errors) else None,
                "max_px": float(np.max(inlier_errors)) if len(inlier_errors) else None,
                "inlier_ratio": float(np.mean(inliers)),
                "anchor_ids_visible": all(anchor_id in unique_ids for anchor_id in ANCHOR_GRID),
            }

            half_tag_in_pitch = 0.5 / (1.0 + tag_spacing)
            model_corners: list[list[float]] = []
            observed_corners: list[np.ndarray] = []
            edge_lengths: list[float] = []
            for rec in unique_records:
                col, row = grid_coordinates(int(rec["id"]))
                x0, x1 = col - half_tag_in_pitch, col + half_tag_in_pitch
                y0, y1 = row - half_tag_in_pitch, row + half_tag_in_pitch
                # Board coordinates use image-like +x right, +y down; the
                # target's fixed decoded corner order is BL, TL, TR, BR.
                model_corners.extend([
                    [x0, y1], [x0, y0], [x1, y0], [x1, y1],
                ])
                obs = np.asarray(rec["corners_raw_original_px"], dtype=np.float32)
                observed_corners.extend(list(obs))
                edge_lengths.extend([
                    float(np.linalg.norm(obs[i] - obs[(i + 1) % 4])) for i in range(4)
                ])

            predicted_corners = _project_homography(
                homography, np.asarray(model_corners, dtype=np.float32))
            observed_array = np.asarray(observed_corners, dtype=np.float32)
            corner_errors = np.linalg.norm(predicted_corners - observed_array, axis=1)
            median_edge = float(np.median(edge_lengths)) if edge_lengths else 0.0
            rms_error = float(np.sqrt(np.mean(corner_errors ** 2)))
            max_error = float(np.max(corner_errors))
            rms_limit = max(3.0, 0.20 * median_edge)
            max_limit = max(5.0, 0.35 * median_edge)
            corner_check = {
                "available": True,
                "rms_px": rms_error,
                "max_px": max_error,
                "median_tag_edge_px": median_edge,
                "rms_limit_px": rms_limit,
                "max_limit_px": max_limit,
            }

    enough_tags = len(unique_ids) >= 4
    center_ok = (
        center_check["available"]
        and center_check["inlier_ratio"] >= 0.90
        and center_check["rms_px"] is not None
        and center_check["rms_px"] <= 3.0
    )
    corner_order_ok = (
        corner_check["available"]
        and corner_check["rms_px"] is not None
        and corner_check["rms_px"] <= corner_check["rms_limit_px"]
        and corner_check["max_px"] <= corner_check["max_limit_px"]
    )
    out_of_range = any(not 0 <= tag_id < 36 for tag_id in ids)
    geometry_ok = enough_tags and not duplicates and not out_of_range and center_ok and corner_order_ok
    full_grid = len(unique_ids) == 36 and not duplicates and unique_ids == list(range(36))

    return {
        "detector": {
            "opencv_version": cv2.__version__,
            "dictionary": "DICT_APRILTAG_36h11",
            "marker_border_bits": 2,
            "preprocess": f"grayscale; {scale}x INTER_CUBIC",
            "pixel_center_mapping": "(scaled_px + 0.5) / scale - 0.5",
            "subpixel": {
                "method": "cornerSubPix on original grayscale, win=(2,2)",
                "max_accepted_shift_px": 0.75,
                "accepted_corners": refined_count * 4,
                "max_observed_shift_px": max_refine_shift,
            },
            "rejected_candidates": len(rejected),
        },
        "detected_ids": ids,
        "unique_valid_ids": unique_ids,
        "out_of_range_ids": sorted(tag_id for tag_id in set(ids) if not 0 <= tag_id < 36),
        "duplicate_ids": duplicates,
        "detected_tags": len(records),
        "detected_corners": 4 * len(unique_records),
        "records": records,
        "geometry_check": {
            "grid_center_homography": center_check,
            "corner_order_homography": corner_check,
            "geometry_ok": bool(geometry_ok),
            "full_grid": bool(full_grid),
            "layout_id": LAYOUT_ID,
            "id_to_grid": "column=id//6, row=id%6",
            "corner_order": CORNER_ORDER,
            "physical_layout_note": (
                "Homography checks the ID grid and a single shared corner order. "
                "The square 6x6 layout is rotationally symmetric; a one-frame image "
                "does not establish camera-to-robot axes or print scale."
            ),
        },
        "eligible_for_fit": False,
    }


def ros_header_from_metadata(metadata_path: Path) -> dict[str, Any]:
    raw = json.loads(metadata_path.read_text(encoding="utf-8"))
    if isinstance(raw, dict) and isinstance(raw.get("image"), dict):
        image = raw["image"]
        header = image.get("metadata", {})
        if header:
            return header
    if isinstance(raw, dict) and all(k in raw for k in ("width", "height", "encoding")):
        return raw
    raise ValueError(
        "metadata 需为 observer summary.json（含 image.metadata）或单帧 ROS header JSON"
    )


def validate_ros_header(header: dict[str, Any], bgr: np.ndarray) -> None:
    expected = (IMAGE_WIDTH, IMAGE_HEIGHT, "bgr8")
    actual = (header.get("width"), header.get("height"), header.get("encoding"))
    if actual != expected:
        raise ValueError(f"PNG metadata 必须是 {expected}，实际 {actual}")
    if bgr.shape != (IMAGE_HEIGHT, IMAGE_WIDTH, 3):
        raise ValueError(f"PNG 像素尺寸必须为 640x360，实际 {bgr.shape[1]}x{bgr.shape[0]}")


def load_dataset_spec(root: Path) -> dict[str, Any] | None:
    path = root / "dataset.json"
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def ensure_dataset(root: Path, spec: dict[str, Any]) -> None:
    root.mkdir(parents=True, exist_ok=True)
    existing = load_dataset_spec(root)
    if existing is None:
        write_json(root / "dataset.json", {
            "tool": "calibrate_camera_aprilgrid.py",
            "tool_version": TOOL_VERSION,
            "created_utc": now_utc(),
            "image_width": IMAGE_WIDTH,
            "image_height": IMAGE_HEIGHT,
            "image_encoding": "bgr8",
            "target": spec,
        })
        return
    target = existing.get("target", {})
    keys = ("target_type", "tag_family", "tag_rows", "tag_cols", "tag_size_m",
            "black_border_bits", "layout_id")
    if any(target.get(key) != spec.get(key) for key in keys):
        raise ValueError("dataset.json 的靶标族、尺寸、ID 布局或黑边与本次设置不一致")
    if not math.isclose(float(target.get("tag_spacing_ratio", -1)), spec["tag_spacing_ratio"], abs_tol=1e-9):
        raise ValueError("tagSpacing 与现有数据集不一致；请统一实测比例后再采集")


def layout_confirmation_path(root: Path) -> Path:
    return root / "layout_verification.json"


def has_valid_layout_confirmation(root: Path, spec: dict[str, Any]) -> bool:
    path = layout_confirmation_path(root)
    if not path.exists():
        return False
    try:
        evidence = json.loads(path.read_text(encoding="utf-8"))
        return (
            evidence.get("layout_id") == LAYOUT_ID
            and len(evidence.get("full_grid_ids", [])) == 36
            and evidence.get("id_to_grid") == "column=id//6, row=id%6"
            and evidence.get("corner_order") == CORNER_ORDER
            and math.isclose(
                float(evidence.get("target", {}).get("tag_spacing_ratio", -1)),
                float(spec["tag_spacing_ratio"]), abs_tol=1e-9)
        )
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return False


def stability_check(frames: list[dict[str, Any]], spec: dict[str, Any], scale: int) -> dict[str, Any]:
    """Compare matching decoded corners in recent distinct frames, in original pixels."""
    result: dict[str, Any] = {"passed": False, "frame_count": len(frames), "frames": [],
                              "minimum_span_s": 0.7, "maximum_median_displacement_px": 1.5}
    if len(frames) < 3:
        result["frames"] = [{"sequence": frame["sequence"], "ros_image_header": frame["metadata"],
                             "common_tags": None, "median_displacement_px": None} for frame in frames]
        result["reason"] = "fewer than three recent frames"
        return result
    result["span_s"] = frames[-1]["received_monotonic"] - frames[0]["received_monotonic"]
    detected = [detect_aprilgrid(frame["image"], spec["tag_size_m"], spec["tag_spacing_ratio"], scale)
                for frame in frames]
    def corners_by_id(data: dict[str, Any]) -> dict[int, np.ndarray]:
        return {int(item["id"]): np.asarray(item["corners_used_original_px"], dtype=float)
                for item in data["records"] if item["id"] in data["unique_valid_ids"]}
    reference = corners_by_id(detected[-1])
    shifts = []
    for frame, data in zip(frames, detected):
        values = corners_by_id(data)
        common = sorted(set(reference) & set(values))
        delta = np.concatenate([np.linalg.norm(values[i] - reference[i], axis=1) for i in common]) if common else []
        median = float(np.median(delta)) if len(delta) else None
        if median is not None:
            shifts.append(median)
        result["frames"].append({"sequence": frame["sequence"], "ros_image_header": frame["metadata"],
                                 "common_tags": len(common), "median_displacement_px": median})
    result["max_frame_median_displacement_px"] = max(shifts) if shifts else None
    result["passed"] = bool(result["span_s"] >= result["minimum_span_s"]
                            and all(entry["common_tags"] >= 4 for entry in result["frames"])
                            and len(shifts) == len(frames)
                            and max(shifts) <= result["maximum_median_displacement_px"])
    result["reason"] = "observed stable" if result["passed"] else "insufficient coverage, duration, or stability"
    return result


def save_sample(
    root: Path,
    role: str,
    image_bgr: np.ndarray,
    png_bytes: bytes,
    source_metadata: dict[str, Any],
    spec: dict[str, Any],
    trigger: dict[str, Any],
    scale: int,
    stability_frames: list[dict[str, Any]] | None = None,
) -> Path:
    ensure_dataset(root, spec)
    validate_ros_header(source_metadata, image_bgr)
    analysis = detect_aprilgrid(
        image_bgr, spec["tag_size_m"], spec["tag_spacing_ratio"], scale)
    geometry = analysis["geometry_check"]
    layout_path = layout_confirmation_path(root)
    confirmed_before = has_valid_layout_confirmation(root, spec)
    full_frame_confirmation = bool(geometry["full_grid"] and geometry["geometry_ok"])
    layout_confirmed = confirmed_before or full_frame_confirmation
    analysis["geometry_check"]["layout_confirmed"] = layout_confirmed
    analysis["geometry_check"]["layout_confirmation_source"] = (
        "prior full-grid sample in this dataset" if confirmed_before else
        "this frame: all 36 IDs and a single consistent 6x6 homography" if full_frame_confirmation else
        "not confirmed; import/capture one full-grid frame first"
    )
    stability_frames = stability_frames or []
    temporal_stability = stability_check(stability_frames, spec, scale)
    analysis["eligible_for_fit"] = bool(geometry["geometry_ok"] and layout_confirmed
                                        and temporal_stability["passed"] and role != "candidate")

    image_hash = sha256(png_bytes)
    sample_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ") + "_" + image_hash[:8]
    sample_dir = root / "samples" / f"{role}_{sample_id}"
    if sample_dir.exists():
        raise FileExistsError(f"sample folder already exists: {sample_dir}")
    sample_dir.mkdir(parents=True, exist_ok=False)
    image_path = sample_dir / "camera.png"
    image_path.write_bytes(png_bytes)
    for index, frame in enumerate(stability_frames):
        ok, encoded = cv2.imencode(".png", frame["image"], [cv2.IMWRITE_PNG_COMPRESSION, 3])
        if not ok:
            raise RuntimeError("OpenCV stability PNG encode failed")
        data = encoded.tobytes()
        name = f"stability_{index:02d}.png"
        (sample_dir / name).write_bytes(data)
        temporal_stability["frames"][index].update({"png_file": name, "png_sha256": sha256(data)})
    record = {
        "sample_id": sample_dir.name,
        "tool_version": TOOL_VERSION,
        "role": role,
        "created_utc": now_utc(),
        "source_topic": source_metadata.get("topic", DEFAULT_TOPIC),
        "source": "FPGA bridge ROS /camera/image_raw; raw bgr8 image",
        "ros_image_header": source_metadata,
        "trigger": trigger,
        "target": spec,
        "png_file": "camera.png",
        "png_sha256": image_hash,
        "analysis": analysis,
        "temporal_stability": temporal_stability,
        "timestamp_note": (
            "ROS header stamp is generated by the bridge while publishing and is not a validated sensor exposure timestamp."
        ),
        "stability_note": "Recent frames are compared in image space; no independent motion or exposure timing is implied.",
    }
    write_json(sample_dir / "metadata.json", record)

    if full_frame_confirmation and not confirmed_before:
        write_json(layout_path, {
            "layout_id": LAYOUT_ID,
            "confirmed_by_sample": sample_dir.name,
            "confirmed_utc": now_utc(),
            "full_grid_ids": analysis["unique_valid_ids"],
            "anchor_ids": {str(k): list(v) for k, v in ANCHOR_GRID.items()},
            "id_to_grid": "column=id//6, row=id%6",
            "corner_order": CORNER_ORDER,
            "center_rms_px": geometry["grid_center_homography"]["rms_px"],
            "corner_rms_px": geometry["corner_order_homography"]["rms_px"],
            "target": spec,
            "scope": "image-grid consistency only; print dimensions and camera extrinsics remain separate",
        })
    return sample_dir


def import_frame(args: argparse.Namespace) -> None:
    image_path = Path(args.image).expanduser().resolve()
    metadata_path = Path(args.metadata).expanduser().resolve()
    root = Path(args.dataset).expanduser().resolve()
    png_bytes = image_path.read_bytes()
    original_summary = json.loads(metadata_path.read_text(encoding="utf-8"))
    expected_hash = original_summary.get("image", {}).get("sha256")
    if expected_hash and expected_hash != sha256(png_bytes):
        raise ValueError("PNG hash does not match observer metadata")
    image = cv2.imdecode(np.frombuffer(png_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"cannot decode PNG: {image_path}")
    header = ros_header_from_metadata(metadata_path)
    header = dict(header)
    header["source_png_path"] = str(image_path)
    header["source_metadata_path"] = str(metadata_path)
    header["source_png_sha256"] = sha256(png_bytes)
    header["topic"] = args.topic
    spec = target_spec(args.tag_size_m, args.tag_spacing_ratio)
    sample_dir = save_sample(
        root, args.role, image, png_bytes, header, spec,
        {"method": "import saved ROS frame", "triggered_by": "operator", "at_utc": now_utc()},
        args.scale,
    )
    report_sample(sample_dir)


def report_sample(sample_dir: Path) -> None:
    record = json.loads((sample_dir / "metadata.json").read_text(encoding="utf-8"))
    analysis = record["analysis"]
    geometry = analysis["geometry_check"]
    centers = geometry["grid_center_homography"]
    corners = geometry["corner_order_homography"]
    print(f"sample: {sample_dir}")
    print(f"role={record['role']} detected={analysis['detected_tags']} ids={analysis['unique_valid_ids']}")
    print(
        "grid-center RMS={:.2f}px, corner-order RMS={:.2f}px, "
        "layout-confirmed={}, eligible-for-fit={}".format(
            centers["rms_px"] if centers["rms_px"] is not None else float("nan"),
            corners["rms_px"] if corners["rms_px"] is not None else float("nan"),
            geometry["layout_confirmed"], analysis["eligible_for_fit"],
        )
    )
    print(f"PNG SHA-256: {record['png_sha256']}")
    print(f"temporal stability: {record['temporal_stability']['reason']}")
    counts = {role: 0 for role in ('candidate', 'train', 'holdout')}
    for path in sample_dir.parent.glob('*/metadata.json'):
        item = json.loads(path.read_text(encoding='utf-8'))
        if item.get('role') in counts and (item['role'] == 'candidate' or item.get('analysis', {}).get('eligible_for_fit')):
            counts[item['role']] += 1
    print(f"dataset samples: {counts}")


def _ros_capture(args: argparse.Namespace) -> None:
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
    except ImportError as exc:
        raise RuntimeError(
            "未找到 ROS Python 包；先 source /opt/ros/humble/setup.bash 和 ugv_ws/install/local_setup.bash"
        ) from exc

    root = Path(args.dataset).expanduser().resolve()
    spec = target_spec(args.tag_size_m, args.tag_spacing_ratio)
    rclpy.init()
    node = rclpy.create_node("bx72_aprilgrid_camera_calibration")
    latest_lock = threading.Lock()
    latest: dict[str, Any] = {}
    history: deque[dict[str, Any]] = deque(maxlen=8)
    sequence = 0

    def on_image(message: Image) -> None:
        nonlocal sequence
        sequence += 1
        if (message.width, message.height, message.encoding) != (IMAGE_WIDTH, IMAGE_HEIGHT, "bgr8"):
            return
        row_bytes = message.width * 3
        data = bytes(message.data)
        if message.step < row_bytes or len(data) < message.step * message.height:
            return
        rows = np.frombuffer(data, dtype=np.uint8).reshape(message.height, message.step)
        image = rows[:, :row_bytes].reshape(message.height, message.width, 3).copy()
        received_unix_ns = time.time_ns()
        received_monotonic = time.monotonic()
        header = {
            "topic": args.topic,
            "header_sec": int(message.header.stamp.sec),
            "header_nanosec": int(message.header.stamp.nanosec),
            "frame_id": message.header.frame_id,
            "width": int(message.width),
            "height": int(message.height),
            "encoding": message.encoding,
            "step": int(message.step),
            "is_bigendian": int(message.is_bigendian),
            "received_unix_ns": received_unix_ns,
            "ros_data_sha256": sha256(data),
        }
        with latest_lock:
            latest.clear()
            latest.update({
                "image": image,
                "png_source_bytes": data,
                "metadata": header,
                "received_monotonic": received_monotonic,
                "sequence": sequence,
            })
            if not history or received_monotonic - history[-1]["received_monotonic"] >= 0.3:
                history.append(dict(latest))

    node.create_subscription(Image, args.topic, on_image, qos_profile_sensor_data)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, name="bx72-ros-image", daemon=True)
    ros_thread.start()
    command_queue: queue.Queue[str | None] = queue.Queue()

    def read_commands() -> None:
        try:
            while True:
                line = input()
                command_queue.put(line.strip().lower())
                if line.strip().lower() in ("q", "quit", "exit"):
                    return
        except EOFError:
            command_queue.put(None)

    command_thread = threading.Thread(target=read_commands, name="bx72-trigger-input", daemon=True)
    command_thread.start()
    print(f"订阅 {args.topic}（只读 ROS；不占用 UDP 端口）。等待有效 bgr8 图像。", flush=True)
    print("输入 s + Enter 采集当前单帧，输入 q + Enter 退出不保存。", flush=True)

    last_preview_at = 0.0
    last_preview_sequence = -1
    try:
        while rclpy.ok():
            now = time.monotonic()
            with latest_lock:
                snapshot = dict(latest)
            if snapshot and now - last_preview_at >= 1.0:
                last_preview_at = now
                if now - snapshot["received_monotonic"] > args.max_frame_age_s:
                    print(f"[preview] STALE age={now - snapshot['received_monotonic']:.3f}s", flush=True)
                    continue
                if snapshot["sequence"] == last_preview_sequence:
                    continue
                last_preview_sequence = snapshot["sequence"]
                preview = detect_aprilgrid(
                    snapshot["image"], args.tag_size_m, args.tag_spacing_ratio, args.scale)
                g = preview["geometry_check"]
                points = [point for marker in preview["records"] if marker["id"] in preview["unique_valid_ids"]
                          for point in marker["corners_used_original_px"]]
                box = ([round(min(p[0] for p in points)), round(min(p[1] for p in points)),
                        round(max(p[0] for p in points)), round(max(p[1] for p in points))] if points else None)
                print(
                    f"[preview] seq={snapshot['sequence']} age={now - snapshot['received_monotonic']:.3f}s "
                    f"tags={preview['detected_tags']} ids={preview['unique_valid_ids']} "
                    f"corners={preview['detected_corners']} bbox={box} geometry_ok={g['geometry_ok']} "
                    f"center_rms={g['grid_center_homography']['rms_px']} "
                    f"corner_rms={g['corner_order_homography']['rms_px']}",
                    flush=True,
                )

            try:
                command = command_queue.get(timeout=0.05)
            except queue.Empty:
                continue
            if command in (None, "q", "quit", "exit"):
                print("退出；没有保存样本。")
                break
            if command != "s":
                if command:
                    print("命令只接受 s（采集一帧）或 q（退出）。", flush=True)
                continue

            with latest_lock:
                snapshot = dict(latest)
                stability_frames = list(history)[-4:]
            if not snapshot:
                print("尚无有效 bgr8 帧；没有保存。", flush=True)
                continue
            if stability_frames and stability_frames[-1]["sequence"] != snapshot["sequence"]:
                stability_frames.append(snapshot)
            age = time.monotonic() - snapshot["received_monotonic"]
            if age > args.max_frame_age_s:
                print(f"最新帧已过期 {age:.3f}s；没有保存，请重新触发。", flush=True)
                continue
            header = snapshot["metadata"]
            header_age = (time.time_ns() - (header["header_sec"] * 1_000_000_000 + header["header_nanosec"])) / 1e9
            if header_age > args.max_frame_age_s or header_age < -0.2:
                print(f"ROS 头时标过期或与本机时钟不一致 {header_age:.3f}s；没有保存。", flush=True)
                continue
            ok, encoded = cv2.imencode(".png", snapshot["image"], [cv2.IMWRITE_PNG_COMPRESSION, 3])
            if not ok:
                raise RuntimeError("OpenCV PNG 编码失败")
            metadata = dict(snapshot["metadata"])
            metadata["topic"] = args.topic
            sample_dir = save_sample(
                root, args.role, snapshot["image"], encoded.tobytes(), metadata, spec,
                {
                    "method": "stdin s + Enter",
                    "requested_unix_ns": time.time_ns(),
                    "frame_sequence": snapshot["sequence"],
                    "frame_age_at_trigger_s": age,
                    "header_age_at_trigger_s": header_age,
                },
                args.scale, stability_frames,
            )
            report_sample(sample_dir)
            break
    finally:
        executor.shutdown(timeout_sec=3.0)
        ros_thread.join(timeout=3.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def points_for_sample(record: dict[str, Any], tag_size_m: float, tag_spacing: float) -> tuple[np.ndarray, np.ndarray]:
    obj: list[np.ndarray] = []
    img: list[np.ndarray] = []
    by_id = {int(item["id"]): item for item in record["analysis"]["records"]}
    for tag_id in sorted(record["analysis"]["unique_valid_ids"]):
        if tag_id not in by_id:
            continue
        obj.extend(object_corners(tag_id, tag_size_m, tag_spacing))
        img.extend(np.asarray(by_id[tag_id]["corners_used_original_px"], dtype=np.float32))
    return np.asarray(obj, dtype=np.float32), np.asarray(img, dtype=np.float32)


def reprojection_stats(object_points: np.ndarray, image_points: np.ndarray, rvec: np.ndarray,
                       tvec: np.ndarray, camera_matrix: np.ndarray,
                       distortion: np.ndarray) -> dict[str, float]:
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, distortion)
    errors = np.linalg.norm(projected.reshape(-1, 2) - image_points.reshape(-1, 2), axis=1)
    return {
        "rms_px": float(np.sqrt(np.mean(errors ** 2))),
        "median_px": float(np.median(errors)),
        "p95_px": float(np.percentile(errors, 95)),
        "max_px": float(np.max(errors)),
        "corner_count": int(len(errors)),
    }


def _load_samples(root: Path, include_candidates: bool) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    dataset = load_dataset_spec(root)
    if dataset is None:
        raise ValueError(f"dataset.json not found in {root}")
    if not has_valid_layout_confirmation(root, dataset["target"]):
        raise ValueError("缺少 layout_verification.json；先导入/采集一张完整 36-ID 标板图进行布局核验")
    roles = {"train", "holdout"}
    if include_candidates:
        roles.add("candidate")
    result = []
    seen_hashes: set[str] = set()
    for metadata_path in sorted((root / "samples").glob("*/metadata.json")):
        record = json.loads(metadata_path.read_text(encoding="utf-8"))
        if record.get("role") not in roles:
            continue
        if not record.get("analysis", {}).get("eligible_for_fit"):
            continue
        image_path = metadata_path.parent / record.get("png_file", "camera.png")
        png_bytes = image_path.read_bytes()
        if sha256(png_bytes) != record.get("png_sha256"):
            raise ValueError(f"PNG hash mismatch: {image_path}")
        if record.get("png_sha256") in seen_hashes:
            raise ValueError(f"重复图像不能计为独立姿态: {image_path}")
        seen_hashes.add(record.get("png_sha256"))
        if record.get("analysis", {}).get("geometry_check", {}).get("layout_id") != LAYOUT_ID:
            raise ValueError(f"unrecognized layout metadata: {metadata_path}")
        sample_target = record.get("target", {})
        if (not math.isclose(float(sample_target.get("tag_size_m", -1)), float(dataset["target"]["tag_size_m"]), abs_tol=1e-12)
                or not math.isclose(float(sample_target.get("tag_spacing_ratio", -1)),
                                    float(dataset["target"]["tag_spacing_ratio"]), abs_tol=1e-12)):
            raise ValueError(f"样本靶标尺寸参数与 dataset.json 不一致: {metadata_path}")
        record["_directory"] = str(metadata_path.parent)
        result.append(record)
    return result, dataset


def fit_calibration(args: argparse.Namespace) -> None:
    root = Path(args.dataset).expanduser().resolve()
    records, dataset = _load_samples(root, False)
    train_records = [r for r in records if r["role"] == "train"]
    holdout_records = [r for r in records if r["role"] == "holdout"]
    if len(train_records) < 3:
        raise ValueError(f"至少需要 3 张有效训练姿态，当前 {len(train_records)} 张")
    if not holdout_records:
        raise ValueError("至少需要 1 张单独标为 holdout 的有效姿态")

    target = dataset["target"]
    tag_size = args.tag_size_m if args.tag_size_m is not None else float(target["tag_size_m"])
    spacing = args.tag_spacing_ratio if args.tag_spacing_ratio is not None else float(target["tag_spacing_ratio"])
    train_obj, train_img = [], []
    train_counts = []
    for rec in train_records:
        obj, img = points_for_sample(rec, tag_size, spacing)
        if len(obj) < 12:
            raise ValueError(f"训练样本点数不足 12: {rec['sample_id']}")
        train_obj.append(obj)
        train_img.append(img.reshape(-1, 1, 2))
        train_counts.append(len(rec["analysis"]["unique_valid_ids"]))

    rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        train_obj, train_img, (IMAGE_WIDTH, IMAGE_HEIGHT), None, None,
        flags=0,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-9),
    )
    train_metrics = []
    view_normals = []
    view_centers = []
    view_areas = []
    for rec, obj, img, rvec, tvec in zip(train_records, train_obj, train_img, rvecs, tvecs):
        stats = reprojection_stats(obj, img, rvec, tvec, camera_matrix, distortion)
        rotation, _ = cv2.Rodrigues(rvec)
        normal = rotation[:, 2]
        view_normals.append(normal / max(float(np.linalg.norm(normal)), 1e-12))
        points = img.reshape(-1, 2)
        lo, hi = points.min(axis=0), points.max(axis=0)
        view_centers.append(((lo + hi) * 0.5).tolist())
        view_areas.append(float((hi[0] - lo[0]) * (hi[1] - lo[1])))
        stats.update({
            "sample_id": rec["sample_id"],
            "role": rec["role"],
            "tag_count": int(len(rec["analysis"]["unique_valid_ids"])),
            "distance_m": float(np.linalg.norm(tvec)),
            "target_normal_camera": [float(v) for v in view_normals[-1]],
            "image_bbox_px": [float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1])],
        })
        train_metrics.append(stats)

    holdout_metrics = []
    for rec in holdout_records:
        obj, img = points_for_sample(rec, tag_size, spacing)
        if len(obj) < 12:
            raise ValueError(f"留出样本点数不足 12: {rec['sample_id']}")
        ok, rvec, tvec = cv2.solvePnP(
            obj, img, camera_matrix, distortion, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            raise ValueError(f"holdout solvePnP failed: {rec['sample_id']}")
        stats = reprojection_stats(obj, img, rvec, tvec, camera_matrix, distortion)
        stats.update({
            "sample_id": rec["sample_id"],
            "role": "holdout",
            "tag_count": int(len(rec["analysis"]["unique_valid_ids"])),
            "distance_m": float(np.linalg.norm(tvec)),
        })
        holdout_metrics.append(stats)

    normal_angles = []
    for i in range(len(view_normals)):
        for j in range(i + 1, len(view_normals)):
            dot = float(np.clip(np.dot(view_normals[i], view_normals[j]), -1.0, 1.0))
            normal_angles.append(math.degrees(math.acos(abs(dot))))
    center_x_span = float(np.ptp([p[0] for p in view_centers]) / IMAGE_WIDTH) if view_centers else 0.0
    center_y_span = float(np.ptp([p[1] for p in view_centers]) / IMAGE_HEIGHT) if view_centers else 0.0
    area_ratio = (max(view_areas) / min(view_areas)) if view_areas and min(view_areas) > 0 else 1.0
    warnings = []
    if len(train_records) < 10:
        warnings.append("训练姿态少于建议的 10 张")
    if len(holdout_records) < 3:
        warnings.append("holdout 少于建议的 3 张")
    if not normal_angles or max(normal_angles) < 15.0:
        warnings.append("训练板面倾角变化不足 15 度；焦距/畸变可能约束较弱")
    if center_x_span < 0.25 or center_y_span < 0.20:
        warnings.append("训练图像中标板中心覆盖不足；建议移到画面四周")
    if area_ratio < 1.5:
        warnings.append("训练标板成像面积变化不足 1.5 倍；建议补近/远姿态")

    fx, fy = float(camera_matrix[0, 0]), float(camera_matrix[1, 1])
    cx, cy = float(camera_matrix[0, 2]), float(camera_matrix[1, 2])
    r_identity = np.eye(3, dtype=float).reshape(-1).tolist()
    p_matrix = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    calibration = {
        "image_width": IMAGE_WIDTH,
        "image_height": IMAGE_HEIGHT,
        "distortion_model": "plumb_bob",
        "camera_matrix": camera_matrix.reshape(-1).astype(float).tolist(),
        "distortion_coefficients": distortion.reshape(-1).astype(float).tolist(),
        "rectification_matrix": r_identity,
        "projection_matrix": p_matrix,
    }
    result = {
        "tool": "calibrate_camera_aprilgrid.py",
        "tool_version": TOOL_VERSION,
        "created_utc": now_utc(),
        "source_dataset": str(root),
        "target": {**target, "tag_size_m_used_for_fit": tag_size, "tag_spacing_ratio_used_for_fit": spacing},
        "detector_pixel_coordinates": "original 640x360 BGR8 pixels; 2x cubic detections mapped back",
        "calibration_method": "OpenCV calibrateCamera, five-parameter plumb_bob; no rational model",
        "train": {
            "view_count": len(train_records),
            "tag_counts": train_counts,
            "global_rms_px": float(rms),
            "views": train_metrics,
        },
        "holdout": {
            "view_count": len(holdout_records),
            "method": "solvePnP per held-out frame with K and D fixed to training fit",
            "global_rms_px": float(np.sqrt(sum(v["rms_px"] ** 2 * v["corner_count"] for v in holdout_metrics)
                                            / sum(v["corner_count"] for v in holdout_metrics))),
            "views": holdout_metrics,
        },
        "pose_diversity": {
            "max_pairwise_target_normal_angle_deg": max(normal_angles) if normal_angles else 0.0,
            "training_center_x_span_fraction": center_x_span,
            "training_center_y_span_fraction": center_y_span,
            "training_projected_area_max_min_ratio": area_ratio,
        },
        "quality_warnings": warnings,
        "camera_info_parameters": calibration,
        "scope_note": (
            "Single-camera intrinsics and distortion only. It does not calibrate camera-to-base_link, "
            "lens focus/zoom changes, or real-world target print scale. Check final holdout residuals "
            "and the target's measured tag size/spacing before publishing CameraInfo."
        ),
    }
    output_path = (Path(args.output).expanduser().resolve() if args.output else
                   root / ("calibration_result_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ") + ".json"))
    with output_path.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print(f"calibration result: {output_path}")
    print(f"train RMS={rms:.3f}px; holdout RMS={result['holdout']['global_rms_px']:.3f}px")
    if warnings:
        for warning in warnings:
            print(f"WARNING: {warning}")
    print("ROS camera parameters (review against holdout before copying to bringup.yaml):")
    print(f"camera_width: {IMAGE_WIDTH}")
    print(f"camera_height: {IMAGE_HEIGHT}")
    print("camera_matrix: [" + ", ".join(f"{v:.10g}" for v in calibration["camera_matrix"]) + "]")
    print("distortion_coefficients: [" + ", ".join(f"{v:.10g}" for v in calibration["distortion_coefficients"]) + "]")
    print("rectification_matrix: [" + ", ".join(f"{v:.10g}" for v in calibration["rectification_matrix"]) + "]")
    print("projection_matrix: [" + ", ".join(f"{v:.10g}" for v in calibration["projection_matrix"]) + "]")
    print("distortion_model: plumb_bob")


def add_target_options(parser: argparse.ArgumentParser, include_scale: bool = True) -> None:
    parser.add_argument("--tag-size-m", type=float, default=DEFAULT_TAG_SIZE_M,
                        help="AprilTag black-square edge length [m] (default 0.025)")
    parser.add_argument("--tag-spacing-ratio", type=float, default=DEFAULT_TAG_SPACING,
                        help="gap/tag-size ratio (default 0.3; Kalibr convention)")
    if include_scale:
        parser.add_argument("--scale", type=int, default=DEFAULT_SCALE,
                            help="grayscale detection upsample (default 2)")


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="BX72 AprilGrid single-frame collector and train/holdout camera calibration")
    commands = parser.add_subparsers(dest="command", required=True)

    capture = commands.add_parser("capture", help="subscribe to ROS and save one operator-triggered frame")
    capture.add_argument("--dataset", required=True, help="dataset output directory")
    capture.add_argument("--role", choices=("train", "holdout", "candidate"), required=True)
    capture.add_argument("--topic", default=DEFAULT_TOPIC)
    capture.add_argument("--max-frame-age-s", type=float, default=0.5)
    add_target_options(capture)

    importer = commands.add_parser("import-frame", help="import an existing ROS BGR8 PNG and metadata")
    importer.add_argument("--dataset", required=True)
    importer.add_argument("--image", required=True, help="saved 640x360 ROS PNG")
    importer.add_argument("--metadata", required=True, help="observer summary.json or one-frame header JSON")
    importer.add_argument("--role", choices=("train", "holdout", "candidate"), default="candidate")
    importer.add_argument("--topic", default=DEFAULT_TOPIC)
    add_target_options(importer)

    fit = commands.add_parser("fit", help="fit K/D on train views and report fixed-K/D holdout errors")
    fit.add_argument("--dataset", required=True)
    fit.add_argument("--output", help="result JSON path; default <dataset>/calibration_result.json")
    fit.add_argument("--tag-size-m", type=float, default=None,
                     help="measured tag edge length override [m]")
    fit.add_argument("--tag-spacing-ratio", type=float, default=None,
                     help="measured gap/tag-size ratio override")
    return parser


def main() -> int:
    parser = make_parser()
    args = parser.parse_args()
    try:
        if args.tag_size_m is not None and args.tag_size_m <= 0:
            raise ValueError("tag size must be positive")
        if args.tag_spacing_ratio is not None and not 0.0 < args.tag_spacing_ratio <= 1.0:
            raise ValueError("tag spacing ratio must be in (0, 1]")
        if args.command == "import-frame":
            import_frame(args)
        elif args.command == "capture":
            if args.max_frame_age_s <= 0:
                raise ValueError("max frame age must be positive")
            _ros_capture(args)
        elif args.command == "fit":
            fit_calibration(args)
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
