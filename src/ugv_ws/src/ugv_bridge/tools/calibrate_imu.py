#!/usr/bin/env python3
"""采集 BX72 的原始 IMU 遥测，并对静止、多面数据做离线质量评估。"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Iterable

from calibration_common import (EXPECTED_WHO_AM_I, EXPECTED_READY_FLAGS, GYRO_DPS_PER_LSB,
                                UINT16_MODULUS, UINT32_MODULUS, sequence_quality)

TOPIC_DEFAULT = "/fpga/imu_raw"
SCHEMA = "bx72_imu_calibration_v1"
ACCEL_G_PER_LSB = 0.000122
GRAVITY_M_S2 = 9.80665


def parse_vector(text: str) -> list[float]:
    try:
        values = [float(part.strip()) for part in text.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("向量格式应为 x,y,z") from exc
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise argparse.ArgumentTypeError("向量必须包含三个有限数值")
    norm = math.sqrt(sum(value * value for value in values))
    if not 0.95 <= norm <= 1.05:
        raise argparse.ArgumentTypeError("静止姿态的 expected-accel-g 向量模长应接近 1g")
    return values


def mean_std(values: Iterable[float]) -> tuple[float | None, float | None]:
    data = list(values)
    if not data:
        return None, None
    mean = math.fsum(data) / len(data)
    if len(data) < 2:
        return mean, None
    variance = math.fsum((value - mean) ** 2 for value in data) / (len(data) - 1)
    return mean, math.sqrt(max(0.0, variance))


def message_ready(sample: dict[str, Any]) -> bool:
    return (
        int(sample["flags"]) == EXPECTED_READY_FLAGS
        and int(sample["who_am_i"]) == EXPECTED_WHO_AM_I
    )


def summarize_samples(samples: list[dict[str, Any]]) -> dict[str, Any]:
    ready = [sample for sample in samples if message_ready(sample)]
    accel_axes = list(zip(*(sample["accel_raw"] for sample in ready))) if ready else [[], [], []]
    gyro_axes = list(zip(*(sample["gyro_raw"] for sample in ready))) if ready else [[], [], []]
    accel_raw_mean = [mean_std(axis)[0] for axis in accel_axes]
    accel_raw_std = [mean_std(axis)[1] for axis in accel_axes]
    gyro_raw_mean = [mean_std(axis)[0] for axis in gyro_axes]
    gyro_raw_std = [mean_std(axis)[1] for axis in gyro_axes]

    accel_mean_m_s2 = [
        None if value is None else value * ACCEL_G_PER_LSB * GRAVITY_M_S2
        for value in accel_raw_mean
    ]
    accel_std_m_s2 = [
        None if value is None else value * ACCEL_G_PER_LSB * GRAVITY_M_S2
        for value in accel_raw_std
    ]
    gyro_mean_rad_s = [
        None if value is None else value * GYRO_DPS_PER_LSB * math.pi / 180.0
        for value in gyro_raw_mean
    ]
    gyro_std_rad_s = [
        None if value is None else value * GYRO_DPS_PER_LSB * math.pi / 180.0
        for value in gyro_raw_std
    ]

    recv_times = [int(sample["received_monotonic_ns"]) for sample in samples]
    recv_intervals_s = [
        (current - previous) / 1e9
        for previous, current in zip(recv_times, recv_times[1:])
        if current > previous
    ]
    rx_elapsed = (recv_times[-1] - recv_times[0]) / 1e9 if len(recv_times) > 1 else 0.0
    rx_rate = (len(recv_times) - 1) / rx_elapsed if rx_elapsed > 0 else None

    fpga_times = [int(sample["timestamp_us"]) % UINT32_MODULUS for sample in samples]
    fpga_intervals_us = [
        (current - previous) % UINT32_MODULUS
        for previous, current in zip(fpga_times, fpga_times[1:])
    ]
    usable_fpga_intervals = [
        value for value in fpga_intervals_us if 0 < value < UINT32_MODULUS // 2
    ]
    fpga_elapsed_us = sum(usable_fpga_intervals)
    fpga_rate = (
        len(usable_fpga_intervals) * 1e6 / fpga_elapsed_us
        if fpga_elapsed_us > 0
        else None
    )

    drop_values = [int(sample["dropped"]) % UINT32_MODULUS for sample in samples]
    dropped_delta = (
        (drop_values[-1] - drop_values[0]) % UINT32_MODULUS if len(drop_values) > 1 else 0
    )
    ready_fraction = len(ready) / len(samples) if samples else 0.0

    def vector_norm(vector: list[float | None]) -> float | None:
        if not vector or any(value is None for value in vector):
            return None
        return math.sqrt(sum(float(value) ** 2 for value in vector))

    accel_std_norm = vector_norm(accel_std_m_s2)
    gyro_std_norm = vector_norm(gyro_std_rad_s)
    return {
        "sample_count": len(samples),
        "ready_sample_count": len(ready),
        "ready_fraction": ready_fraction,
        "flags_seen": sorted({int(sample["flags"]) for sample in samples}),
        "who_am_i_seen": sorted({int(sample["who_am_i"]) for sample in samples}),
        "receive_elapsed_s": rx_elapsed,
        "receive_rate_hz": rx_rate,
        "receive_interval_median_ms": (
            statistics.median(recv_intervals_s) * 1000.0 if recv_intervals_s else None
        ),
        "fpga_snapshot_rate_hz": fpga_rate,
        "fpga_timestamp_interval_median_ms": (
            statistics.median(usable_fpga_intervals) / 1000.0
            if usable_fpga_intervals
            else None
        ),
        "telemetry_dropped_counter_delta": dropped_delta,
        "packet_sequence": sequence_quality(
            [int(sample["sequence"]) for sample in samples], UINT32_MODULUS
        ),
        "reader_sequence": sequence_quality(
            [int(sample["reader_sequence"]) for sample in samples], UINT16_MODULUS
        ),
        "accel_raw_mean_lsb": accel_raw_mean,
        "accel_raw_std_lsb": accel_raw_std,
        "accel_nominal_mean_m_s2_sensor_axes": accel_mean_m_s2,
        "accel_nominal_std_m_s2_sensor_axes": accel_std_m_s2,
        "accel_nominal_mean_magnitude_m_s2": vector_norm(accel_mean_m_s2),
        "accel_nominal_std_vector_norm_m_s2": accel_std_norm,
        "gyro_raw_mean_lsb": gyro_raw_mean,
        "gyro_raw_std_lsb": gyro_raw_std,
        "gyro_nominal_mean_rad_s_sensor_axes": gyro_mean_rad_s,
        "gyro_nominal_std_rad_s_sensor_axes": gyro_std_rad_s,
        "gyro_nominal_mean_magnitude_rad_s": vector_norm(gyro_mean_rad_s),
        "gyro_nominal_std_vector_norm_rad_s": gyro_std_norm,
        "ready_sequence_span": (
            (int(ready[-1]["reader_sequence"]) - int(ready[0]["reader_sequence"]))
            % UINT16_MODULUS
            if len(ready) > 1
            else 0
        ),
    }


def _sample_record(message: Any, node: Any) -> dict[str, Any]:
    ros_now_ns = int(node.get_clock().now().nanoseconds)
    header = message.header
    return {
        "record_type": "sample",
        "header_stamp_sec": int(header.stamp.sec),
        "header_stamp_nanosec": int(header.stamp.nanosec),
        "header_stamp_ns": int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec),
        "frame_id": str(header.frame_id),
        "sequence": int(message.sequence),
        "timestamp_us": int(message.timestamp_us),
        "dropped": int(message.dropped),
        "reader_sequence": int(message.reader_sequence),
        "flags": int(message.flags),
        "who_am_i": int(message.who_am_i),
        "sensor_state": {
            "identity_checked": bool(message.flags & 0x08),
            "identity_ok": bool(message.flags & 0x04),
            "configured": bool(message.flags & 0x02),
            "sample_valid": bool(message.flags & 0x01),
            "ready": bool(
                message.flags == EXPECTED_READY_FLAGS
                and message.who_am_i == EXPECTED_WHO_AM_I
            ),
        },
        "accel_raw": [int(value) for value in message.accel_raw],
        "gyro_raw": [int(value) for value in message.gyro_raw],
        "received_wall_time_ns": time.time_ns(),
        "received_monotonic_ns": time.monotonic_ns(),
        "received_ros_time_ns": ros_now_ns,
    }


def capture(args: argparse.Namespace) -> int:
    output = Path(args.output).expanduser()
    if output.exists():
        raise FileExistsError(f"输出文件已存在，为防止覆盖而停止：{output}")
    if not output.parent.is_dir():
        raise FileNotFoundError(f"输出目录不存在：{output.parent}")

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from ugv_bridge.msg import Bx72ImuRaw
    except ImportError as exc:
        raise RuntimeError(
            "无法导入 rclpy 或 ugv_bridge.msg；请先 source ROS 2 与 ugv_ws/install/setup.bash"
        ) from exc

    class Collector(Node):
        def __init__(self) -> None:
            super().__init__("bx72_imu_calibration_capture")
            self.samples: list[dict[str, Any]] = []
            self.subscription = self.create_subscription(
                Bx72ImuRaw, args.topic, self.on_message, qos_profile_sensor_data
            )

        def on_message(self, message: Any) -> None:
            self.samples.append(_sample_record(message, self))

    rclpy.init(args=[])
    node = Collector()
    interrupted = False
    try:
        print(
            f"等待 {args.topic} 首个消息（最多 {args.start_timeout:g}s）；"
            "只订阅 ROS 原始话题，不打开 FPGA UDP 端口。",
            flush=True,
        )
        deadline = time.monotonic() + args.start_timeout
        while not node.samples and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=min(0.2, max(0.0, deadline - time.monotonic())))
        if not node.samples:
            raise TimeoutError(f"等待 {args.topic} 超时；没有创建输出文件。")

        settle_deadline = time.monotonic() + args.settle_seconds
        if args.settle_seconds:
            print(f"收到遥测；静置等待 {args.settle_seconds:g}s 后开始计时。", flush=True)
        while time.monotonic() < settle_deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=min(0.2, max(0.0, settle_deadline - time.monotonic())))

        # 丢弃发现话题及静置阶段的消息，只保留指定采集窗口。
        node.samples.clear()
        collection_start_wall_ns = time.time_ns()
        collection_start_mono_ns = time.monotonic_ns()
        collection_deadline = time.monotonic() + args.duration
        print(f"开始采集姿态 {args.label!r}，窗口 {args.duration:g}s。", flush=True)
        try:
            while time.monotonic() < collection_deadline and rclpy.ok():
                rclpy.spin_once(
                    node,
                    timeout_sec=min(0.2, max(0.0, collection_deadline - time.monotonic())),
                )
        except KeyboardInterrupt:
            interrupted = True
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not node.samples:
        raise RuntimeError("采集窗口内没有收到消息；没有创建输出文件。")

    collection_end_wall_ns = time.time_ns()
    collection_end_mono_ns = time.monotonic_ns()
    metadata = {
        "record_type": "capture_metadata",
        "schema": SCHEMA,
        "topic": args.topic,
        "label": args.label,
        "expected_accel_g_sensor_axes": args.expected_accel_g,
        "requested_duration_s": args.duration,
        "settle_seconds": args.settle_seconds,
        "collection_start_wall_time_ns": collection_start_wall_ns,
        "collection_end_wall_time_ns": collection_end_wall_ns,
        "collection_elapsed_monotonic_s": (collection_end_mono_ns - collection_start_mono_ns) / 1e9,
        "interrupted": interrupted,
        "source_units": {
            "accel": "raw LSB; nominal conversion uses 0.122 mg/LSB then 9.80665 m/s^2 per g",
            "gyro": "raw LSB; nominal conversion uses 17.5 mdps/LSB then radians/s",
        },
        "timestamp_notes": (
            "header.stamp is the bridge ROS receive time; timestamp_us is the FPGA telemetry snapshot time "
            "in a wrapping 32-bit microsecond clock, not the IMU conversion instant; receive_* fields are "
            "local listener times and are not hardware synchronized."
        ),
        "axis_notes": "All raw values remain in the IMU660RC sensor XYZ axes; no TF rotation or Rx(pi) was applied.",
        "accel_vector_note": (
            "The reported accelerometer vector is specific force. At rest it points upward and has magnitude near g; "
            "the physical gravity vector points in the opposite direction."
        ),
    }
    summary = summarize_samples(node.samples)
    records = [metadata]
    records.extend(node.samples)
    records.append({"record_type": "capture_summary", "summary": summary})
    with output.open("x", encoding="utf-8", newline="\n") as stream:
        for record in records:
            stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False, separators=(",", ":")))
            stream.write("\n")

    print(
        json.dumps(
            {"output": str(output), "label": args.label, "summary": summary},
            ensure_ascii=False,
            indent=2,
            allow_nan=False,
        )
    )
    return 130 if interrupted else 0


def read_capture(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    metadata: dict[str, Any] | None = None
    samples: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number}: JSONL 格式错误：{exc}") from exc
            if record.get("record_type") == "capture_metadata":
                if metadata is not None:
                    raise ValueError(f"{path}:{line_number}: 出现重复的采集头记录")
                metadata = record
            elif record.get("record_type") == "sample":
                samples.append(record)
    if metadata is None or metadata.get("schema") != SCHEMA:
        raise ValueError(f"{path}: 缺少兼容的采集头记录")
    return metadata, samples


def nearest_face(vector: list[float]) -> tuple[str | None, float]:
    faces = {
        "+x": [1.0, 0.0, 0.0],
        "-x": [-1.0, 0.0, 0.0],
        "+y": [0.0, 1.0, 0.0],
        "-y": [0.0, -1.0, 0.0],
        "+z": [0.0, 0.0, 1.0],
        "-z": [0.0, 0.0, -1.0],
    }
    candidates = [
        (
            name,
            math.sqrt(sum((float(vector[i]) - target[i]) ** 2 for i in range(3))),
        )
        for name, target in faces.items()
    ]
    name, distance = min(candidates, key=lambda item: item[1])
    return (name if distance <= 0.15 else None), distance


def linear_fit(x_values: list[float], y_values: list[float]) -> dict[str, float] | None:
    if len(x_values) != len(y_values) or len(x_values) < 3:
        return None
    x_mean = math.fsum(x_values) / len(x_values)
    y_mean = math.fsum(y_values) / len(y_values)
    denominator = math.fsum((value - x_mean) ** 2 for value in x_values)
    if denominator < 0.5:
        return None
    slope = math.fsum(
        (x_values[index] - x_mean) * (y_values[index] - y_mean)
        for index in range(len(x_values))
    ) / denominator
    intercept = y_mean - slope * x_mean
    residuals = [y_values[i] - (intercept + slope * x_values[i]) for i in range(len(x_values))]
    rmse_counts = math.sqrt(math.fsum(value * value for value in residuals) / len(residuals))
    return {
        "offset_counts_at_zero_g": intercept,
        "counts_per_g": slope,
        "g_per_count": (1.0 / slope) if abs(slope) > 1e-12 else 0.0,
        "fit_rmse_counts": rmse_counts,
        "fit_rmse_g": rmse_counts / abs(slope) if abs(slope) > 1e-12 else math.inf,
    }


def analyze(args: argparse.Namespace) -> int:
    captures: list[dict[str, Any]] = []
    for source in args.files:
        metadata, samples = read_capture(source)
        summary = summarize_samples(samples)
        expected = metadata.get("expected_accel_g_sensor_axes")
        face, face_error = nearest_face(expected) if expected is not None else (None, None)
        summary["likely_static"] = bool(
            summary["ready_sample_count"] >= args.min_valid_samples
            and summary["ready_fraction"] >= 0.99
            and summary["telemetry_dropped_counter_delta"] == 0
            and summary["packet_sequence"]["gap_events"] == 0
            and summary["packet_sequence"]["backward_steps"] == 0
            and summary["reader_sequence"]["gap_events"] == 0
            and summary["reader_sequence"]["backward_steps"] == 0
            and summary["accel_nominal_std_vector_norm_m_s2"] is not None
            and summary["accel_nominal_std_vector_norm_m_s2"] <= args.max_accel_std_m_s2
            and summary["gyro_nominal_std_vector_norm_rad_s"] is not None
            and summary["gyro_nominal_std_vector_norm_rad_s"] <= args.max_gyro_std_rad_s
            and summary["gyro_nominal_mean_magnitude_rad_s"] is not None
            and summary["gyro_nominal_mean_magnitude_rad_s"] <= args.max_gyro_mean_rad_s
            and summary["accel_nominal_mean_magnitude_m_s2"] is not None
            and abs(summary["accel_nominal_mean_magnitude_m_s2"] - GRAVITY_M_S2)
            <= args.max_gravity_norm_error_m_s2
        )
        summary["gyro_bias_candidate_rad_s"] = (
            summary["gyro_nominal_mean_rad_s_sensor_axes"] if summary["likely_static"] else None
        )
        measured = summary["accel_nominal_mean_m_s2_sensor_axes"]
        expected_error_g = (
            math.sqrt(sum((float(measured[i]) / GRAVITY_M_S2 - float(expected[i])) ** 2
                          for i in range(3)))
            if expected is not None and all(value is not None for value in measured) else None
        )
        expected_direction_consistent = (
            expected_error_g is not None and expected_error_g <= args.max_expected_axis_error_g
        )
        captures.append(
            {
                "file": str(source),
                "metadata": metadata,
                "summary": summary,
                "expected_accel_g_sensor_axes": expected,
                "nearest_six_face": face,
                "nearest_face_error_g": face_error,
                "measured_vs_expected_error_g": expected_error_g,
                "measured_vs_expected_consistent": expected_direction_consistent,
            }
        )

    required_faces = ["+x", "-x", "+y", "-y", "+z", "-z"]
    eligible: dict[str, list[dict[str, Any]]] = {face: [] for face in required_faces}
    for capture_item in captures:
        face = capture_item["nearest_six_face"]
        if (face is not None and capture_item["summary"]["likely_static"]
                and not capture_item["metadata"].get("interrupted", True)
                and capture_item["measured_vs_expected_consistent"]):
            eligible[face].append(capture_item)
    missing_faces = [face for face in required_faces if not eligible[face]]

    axis_fits: dict[str, Any] = {}
    if not any(missing_faces):
        accepted_captures = [item for face in required_faces for item in eligible[face]]
        for axis_index, axis_name in enumerate(("x", "y", "z")):
            x_values = [float(item["expected_accel_g_sensor_axes"][axis_index]) for item in accepted_captures]
            y_values = [float(item["summary"]["accel_raw_mean_lsb"][axis_index]) for item in accepted_captures]
            fit = linear_fit(x_values, y_values)
            if fit is not None:
                fit["fit_rmse_g"] = (
                    fit["fit_rmse_g"] if math.isfinite(fit["fit_rmse_g"]) else None
                )
                fit["positive_sensitivity"] = fit["counts_per_g"] > 0.0
                axis_fits[axis_name] = fit
            else:
                axis_fits[axis_name] = None

    fit_solved = (
        not missing_faces
        and len(axis_fits) == 3
        and all(
            fit is not None
            and fit["positive_sensitivity"]
            and fit["fit_rmse_g"] is not None
            and fit["fit_rmse_g"] <= args.max_fit_rmse_g
            for fit in axis_fits.values()
        )
    )
    if fit_solved:
        fit_reason = "六个已知传感器轴正负面均有足量、稳定且无序号/丢包异常的数据。"
    elif missing_faces:
        fit_reason = "未求解：缺少有效的六面姿态：" + ", ".join(missing_faces)
    else:
        fit_reason = "未求解：轴向敏感度符号、线性残差或姿态可观测性未通过条件。"

    report = {
        "schema": SCHEMA,
        "nominal_sensor_config": {
            "accel_range": "±4g",
            "accel_sensitivity_g_per_lsb": ACCEL_G_PER_LSB,
            "gyro_range": "±500 deg/s",
            "gyro_sensitivity_deg_s_per_lsb": GYRO_DPS_PER_LSB,
            "nominal_sample_odr_hz": 120,
        },
        "quality_thresholds": {
            "minimum_ready_samples_per_pose": args.min_valid_samples,
            "maximum_accel_std_vector_norm_m_s2": args.max_accel_std_m_s2,
            "maximum_gyro_std_vector_norm_rad_s": args.max_gyro_std_rad_s,
            "maximum_gyro_mean_magnitude_rad_s": args.max_gyro_mean_rad_s,
            "maximum_gravity_norm_error_m_s2": args.max_gravity_norm_error_m_s2,
            "maximum_fit_rmse_g": args.max_fit_rmse_g,
            "maximum_measured_vs_expected_vector_error_g": args.max_expected_axis_error_g,
        },
        "captures": captures,
        "six_face_calibration": {
            "solved": fit_solved,
            "missing_faces": missing_faces,
            "axis_fits": axis_fits,
            "reason": fit_reason,
            "model": "每轴独立线性模型：raw_counts = offset + counts_per_g * expected_accel_g；未估计轴间非正交或完整旋转矩阵。",
        },
        "orientation_limits": (
            "静止重力可检查传感器轴的俯仰/横滚方向和符号；它不能确定车头方向或绝对偏航。"
            " Rx(pi) 只按当前安装假设参与坐标变换，本分析不应用 TF，也不验证该假设。"
            "方向/旋转验证需外部已知姿态或已知正向的手动旋转参考；不得仅凭静止重力声称偏航已验证。"
        ),
    }
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if fit_solved else 2


def positive_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("数值必须为正的有限数")
    return number


def nonnegative_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError("数值必须为非负的有限数")
    return number


def bounded_duration(value: str) -> float:
    number = positive_float(value)
    if number > 600.0:
        raise argparse.ArgumentTypeError("单次采集时长最多 600 秒")
    return number


def bounded_wait(value: str) -> float:
    number = nonnegative_float(value)
    if number > 120.0:
        raise argparse.ArgumentTypeError("等待时间最多 120 秒")
    return number


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="只订阅 /fpga/imu_raw，保存 BX72 IMU 原始样本并评估静止/六面标定数据。"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    capture_parser = subparsers.add_parser("capture", help="采集一个静止或指定姿态的定时样本文件")
    capture_parser.add_argument("--topic", default=TOPIC_DEFAULT)
    capture_parser.add_argument("--label", required=True, help="姿态标签，如 level、+x、turn-positive-z")
    capture_parser.add_argument("--duration", type=bounded_duration, default=15.0)
    capture_parser.add_argument("--settle-seconds", type=bounded_wait, default=2.0)
    capture_parser.add_argument("--start-timeout", type=positive_float, default=30.0)
    capture_parser.add_argument("--expected-accel-g", type=parse_vector, default=None,
                                help="已知传感器轴静止比力向量，单位 g，例如 0,0,1；不确定时省略")
    capture_parser.add_argument("--output", required=True, help="新 JSONL 文件路径；已存在时拒绝覆盖")
    capture_parser.set_defaults(handler=capture)

    analyze_parser = subparsers.add_parser("analyze", help="离线汇总一个或多个 JSONL 姿态文件")
    analyze_parser.add_argument("files", nargs="+", type=Path)
    analyze_parser.add_argument("--min-valid-samples", type=int, default=100)
    analyze_parser.add_argument("--max-accel-std-m-s2", type=positive_float, default=0.20)
    analyze_parser.add_argument("--max-gyro-std-rad-s", type=positive_float, default=0.02)
    analyze_parser.add_argument("--max-gyro-mean-rad-s", type=positive_float, default=0.20)
    analyze_parser.add_argument("--max-gravity-norm-error-m-s2", type=positive_float, default=1.0)
    analyze_parser.add_argument("--max-fit-rmse-g", type=positive_float, default=0.20)
    analyze_parser.add_argument("--max-expected-axis-error-g", type=positive_float, default=0.15,
                                help="多面拟合前，实测比力向量与声明姿态的最大差值")
    analyze_parser.set_defaults(handler=analyze)
    return parser


def main() -> int:
    parser = make_parser()
    args = parser.parse_args()
    if args.command == "analyze" and args.min_valid_samples < 2:
        parser.error("--min-valid-samples 必须至少为 2")
    try:
        return int(args.handler(args))
    except KeyboardInterrupt:
        print("已中断。", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
