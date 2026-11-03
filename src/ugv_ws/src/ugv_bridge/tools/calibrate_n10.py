#!/usr/bin/env python3
"""Passively capture BX72 N10 ROS packets and summarize selected angle sectors.

This records the existing /fpga/n10_packets message stream. It does not open a
UDP socket, command the robot, change TF, or infer a calibrated angle zero or
sensor extrinsic. Output is one JSON object per line (session, packets, summary).
"""

import argparse
from array import array
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import sys
import time
from typing import Dict, List, Optional, Tuple


UINT32_MASK = (1 << 32) - 1
NO_ECHO_VALUES = (0, 0xFFFF)
POLAR_MIN_MM = 50
POLAR_MAX_MM = 8000


def _finite_number(value: str, label: str) -> float:
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("{} must be numeric".format(label)) from exc
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("{} must be finite".format(label))
    return number


def _degrees_to_cdeg(value: str, label: str) -> int:
    number = _finite_number(value, label)
    if not 0.0 <= number < 360.0:
        raise argparse.ArgumentTypeError("{} must be in [0, 360) degrees".format(label))
    scaled = number * 100.0
    rounded = int(round(scaled))
    if abs(scaled - rounded) > 1e-7:
        raise argparse.ArgumentTypeError("{} supports 0.01 degree steps".format(label))
    return rounded % 36000


def _sector_spec(text: str) -> Dict[str, object]:
    parts = text.split(":")
    if len(parts) != 3 or not parts[0].strip():
        raise argparse.ArgumentTypeError("sector format: NAME:START_DEG:END_DEG")
    try:
        start = _degrees_to_cdeg(parts[1], "sector start")
        end = _degrees_to_cdeg(parts[2], "sector end")
    except argparse.ArgumentTypeError:
        raise
    if start == end:
        raise argparse.ArgumentTypeError("sector start and end must differ")
    return {"name": parts[0].strip(), "start_cdeg": start, "end_cdeg": end}


def _target_spec(text: str) -> Dict[str, object]:
    """NAME:RANGE_MM:BODY_BEARING_DEG[:RAW_START_DEG:RAW_END_DEG]"""
    parts = text.split(":")
    if len(parts) not in (3, 5) or not parts[0].strip():
        raise argparse.ArgumentTypeError(
            "target format: NAME:RANGE_MM:BODY_BEARING_DEG[:RAW_START_DEG:RAW_END_DEG]"
        )
    try:
        distance = int(parts[1], 10)
        bearing = _finite_number(parts[2], "target body bearing")
        if distance <= 0:
            raise argparse.ArgumentTypeError("target range must be positive millimetres")
        sector = None
        if len(parts) == 5:
            start = _degrees_to_cdeg(parts[3], "target raw-sector start")
            end = _degrees_to_cdeg(parts[4], "target raw-sector end")
            if start == end:
                raise argparse.ArgumentTypeError("target raw-sector start and end must differ")
            sector = {"name": parts[0].strip(), "start_cdeg": start, "end_cdeg": end}
    except ValueError as exc:
        raise argparse.ArgumentTypeError("target range must be integer millimetres") from exc
    except argparse.ArgumentTypeError:
        raise
    return {
        "name": parts[0].strip(),
        "measured_distance_mm": distance,
        "measured_body_bearing_deg": bearing,
        "raw_sector": sector,
    }


def _sector_contains(angle_cdeg: int, sector: Dict[str, object]) -> bool:
    start = int(sector["start_cdeg"])
    end = int(sector["end_cdeg"])
    return (angle_cdeg - start) % 36000 < (end - start) % 36000


def _sector_width_deg(sector: Dict[str, object]) -> float:
    return ((int(sector["end_cdeg"]) - int(sector["start_cdeg"])) % 36000) / 100.0


def _sector_json(sector: Dict[str, object]) -> Dict[str, object]:
    return {
        "name": sector["name"],
        "raw_angle_start_deg_inclusive": int(sector["start_cdeg"]) / 100.0,
        "raw_angle_end_deg_exclusive": int(sector["end_cdeg"]) / 100.0,
        "wraps_zero": int(sector["start_cdeg"]) > int(sector["end_cdeg"]),
        "width_deg": _sector_width_deg(sector),
    }


class NumericSummary:
    """Streaming uint16 histogram for exact range/strength quantiles."""

    def __init__(self) -> None:
        self.count = 0
        self.total = 0
        self.minimum = None
        self.maximum = None
        self.histogram = array("I", [0]) * 65536

    def add(self, value: int) -> None:
        value = int(value)
        self.count += 1
        self.total += value
        self.histogram[value] += 1
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)

    def quantile(self, fraction: float) -> Optional[int]:
        if not self.count:
            return None
        index = max(0, min(self.count - 1, int(math.ceil(fraction * self.count) - 1)))
        return self.value_at_rank(index)

    def value_at_rank(self, index: int) -> int:
        seen = 0
        for value, count in enumerate(self.histogram):
            seen += count
            if seen > index:
                return value
        return int(self.maximum)

    def median(self) -> Optional[float]:
        if not self.count:
            return None
        upper = self.count // 2
        if self.count % 2:
            return float(self.value_at_rank(upper))
        lower_value = self.value_at_rank(upper - 1)
        upper_value = self.value_at_rank(upper)
        return (lower_value + upper_value) / 2.0

    def json(self) -> Dict[str, object]:
        if not self.count:
            return {"count": 0, "min": None, "median": None, "p95": None,
                    "max": None, "mean": None}
        return {
            "count": self.count,
            "min": self.minimum,
            "median": self.median(),
            "p95": self.quantile(0.95),
            "max": self.maximum,
            "mean": round(self.total / self.count, 3),
        }


class SectorSummary:
    def __init__(self, sector: Dict[str, object]) -> None:
        self.sector = sector
        self.candidate_points = 0
        self.unknown_points = 0
        self.polar_eligible_points = 0
        self.packets_with_candidate = 0
        self.packets_with_return = 0
        self.ranges = NumericSummary()
        self.strengths = NumericSummary()
        self.angle_sin = 0.0
        self.angle_cos = 0.0
        self.angle_count = 0

    def add(self, angle_cdeg: int, raw_distance_mm: int, strength: int) -> bool:
        self.candidate_points += 1
        self.angle_sin += math.sin(angle_cdeg * math.pi / 18000.0)
        self.angle_cos += math.cos(angle_cdeg * math.pi / 18000.0)
        self.angle_count += 1
        if raw_distance_mm in NO_ECHO_VALUES:
            self.unknown_points += 1
            return False
        self.ranges.add(raw_distance_mm)
        self.strengths.add(strength)
        if POLAR_MIN_MM <= raw_distance_mm <= POLAR_MAX_MM:
            self.polar_eligible_points += 1
        return True

    def json(self) -> Dict[str, object]:
        angle_mean = None
        if self.angle_count and (self.angle_sin != 0.0 or self.angle_cos != 0.0):
            angle_mean = math.degrees(math.atan2(self.angle_sin, self.angle_cos)) % 360.0
        return {
            "sector": _sector_json(self.sector),
            "candidate_points": self.candidate_points,
            "unknown_no_echo_points": self.unknown_points,
            "valid_return_points": self.ranges.count,
            "valid_return_fraction_of_sector_samples": (
                self.ranges.count / self.candidate_points if self.candidate_points else None
            ),
            "packets_with_sector_samples": self.packets_with_candidate,
            "packets_with_valid_return": self.packets_with_return,
            "raw_angle_circular_mean_deg": round(angle_mean, 4) if angle_mean is not None else None,
            "distance_mm": self.ranges.json(),
            "strength": self.strengths.json(),
            "polar_navigation_eligible_returns_50_to_8000_mm": self.polar_eligible_points,
            "empty_sector_means_unknown": self.ranges.count == 0,
        }


class PacketTotals:
    def __init__(self) -> None:
        self.packet_count = 0
        self.point_count = 0
        self.echo_count = 0
        self.unknown_count = 0
        self.polar_eligible_count = 0
        self.raw_angle_5deg_slots = set()
        self.return_angle_5deg_slots = set()
        self.ranges = NumericSummary()

    def json(self) -> Dict[str, object]:
        return {
            "packets": self.packet_count,
            "points": self.point_count,
            "valid_returns": self.echo_count,
            "unknown_no_echo_points": self.unknown_count,
            "valid_return_fraction": self.echo_count / self.point_count if self.point_count else None,
            "distance_mm": self.ranges.json(),
            "polar_navigation_eligible_returns_50_to_8000_mm": self.polar_eligible_count,
            "observed_5deg_slots_of_72": len(self.raw_angle_5deg_slots),
            "5deg_slots_with_valid_returns_of_72": len(self.return_angle_5deg_slots),
            "missing_5deg_slots": sorted(set(range(72)) - self.raw_angle_5deg_slots),
        }


class SequenceTracker:
    def __init__(self) -> None:
        self.last_sequence = None
        self.gaps = 0
        self.duplicates = 0
        self.out_of_order_or_restart = 0

    def observe(self, sequence: int) -> Tuple[Optional[int], int]:
        sequence = int(sequence) & UINT32_MASK
        if self.last_sequence is None:
            self.last_sequence = sequence
            return None, 0
        delta = (sequence - self.last_sequence) & UINT32_MASK
        gap = 0
        if delta == 0:
            self.duplicates += 1
        elif delta < 0x80000000:
            gap = max(0, delta - 1)
            self.gaps += gap
        else:
            self.out_of_order_or_restart += 1
        self.last_sequence = sequence
        return delta, gap


def _message_stamp_ns(stamp: object) -> int:
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def _write_json_line(stream: object, value: Dict[str, object]) -> None:
    stream.write(json.dumps(value, ensure_ascii=False, separators=(",", ":"), allow_nan=False))
    stream.write("\n")
    stream.flush()


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Passively capture the existing ROS /fpga/n10_packets stream. "
            "No UDP socket, movement command, TF edit, or calibration pass/fail is used."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python3 calibrate_n10.py --duration 20 --output n10_front.jsonl "
            "--sector front:350:10\n"
            "  python3 calibrate_n10.py --target left_wall:2000:90:85:95\n\n"
            "A sector is in the N10 raw angle frame. A target bearing is the user's "
            "measured robot-frame bearing (x forward, y left, positive counter-clockwise); "
            "the tool does not equate those frames."
        ),
    )
    parser.add_argument("--topic", default="/fpga/n10_packets", help="existing ROS topic to subscribe to")
    parser.add_argument("--duration", type=float, default=20.0,
                        help="capture duration in wall-clock seconds (default: 20; maximum: 3600)")
    parser.add_argument("--output", type=Path, default=None,
                        help="new JSONL output file; default is a timestamped file in the current directory")
    parser.add_argument("--qos-depth", type=int, default=1024,
                        help="subscriber keep-last depth; reliability is best effort to match the bridge sensor stream")
    parser.add_argument("--sector", action="append", type=_sector_spec, default=[], metavar="NAME:START:END",
                        help="summarize a raw-angle sector in degrees; repeat as needed")
    parser.add_argument("--target", action="append", type=_target_spec, default=[], metavar="SPEC",
                        help=("record measured target NAME:RANGE_MM:BODY_BEARING_DEG; optionally "
                              "append :RAW_START_DEG:RAW_END_DEG for sector statistics"))
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration <= 0 or args.duration > 3600:
        parser.error("--duration must be greater than 0 and at most 3600 seconds")
    if args.qos_depth < 1 or args.qos_depth > 100000:
        parser.error("--qos-depth must be in [1, 100000]")
    if not args.topic.startswith("/"):
        parser.error("--topic must be an absolute ROS topic name")

    sectors = list(args.sector)
    sector_names = {str(sector["name"]) for sector in sectors}
    for target in args.target:
        sector = target["raw_sector"]
        if sector is not None:
            if str(sector["name"]) in sector_names:
                parser.error("sector names must be unique across --sector and --target")
            sector_names.add(str(sector["name"]))
            sectors.append(sector)

    if args.output is None:
        suffix = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        output_path = Path.cwd() / ("n10_capture_{}.jsonl".format(suffix))
    else:
        output_path = args.output.expanduser()

    if not output_path.parent.exists():
        parser.error("output directory does not exist: {}".format(output_path.parent))

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from ugv_bridge.msg import N10Packet
    except ImportError as exc:
        print(
            "ROS 2 Python environment or ugv_bridge message is unavailable. "
            "Source the installed ROS and workspace setup files first. ({})".format(exc),
            file=sys.stderr,
        )
        return 2

    try:
        output_stream = output_path.open("x", encoding="utf-8", newline="\n")
    except FileExistsError:
        print("Refusing to overwrite existing output: {}".format(output_path), file=sys.stderr)
        return 2
    except OSError as exc:
        print("Cannot create output {}: {}".format(output_path, exc), file=sys.stderr)
        return 2

    totals = PacketTotals()
    sector_summaries = {str(sector["name"]): SectorSummary(sector) for sector in sectors}
    sequence_tracker = SequenceTracker()
    previous_source_us = None
    previous_receive_mono_ns = None
    source_elapsed_us = 0
    source_clock_resets = 0
    receive_intervals_ms = []
    counter_previous = {}
    counter_totals = {"dropped": 0, "crc_errors": 0, "length_errors": 0, "framing_errors": 0}
    counter_resets = {name: 0 for name in counter_totals}
    counter_available = {name: True for name in counter_totals}
    counter_available["framing_errors"] = False
    first_wall_ns = 0
    first_mono_ns = 0
    requested_end_mono_ns = 0
    stopped_by_interrupt = False

    class CaptureNode(Node):
        def __init__(self) -> None:
            super().__init__("bx72_n10_calibration_collector")
            qos = QoSProfile(depth=args.qos_depth, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.subscription = self.create_subscription(N10Packet, args.topic, self.on_packet, qos)
            self.logger = self.get_logger()

        def on_packet(self, message: object) -> None:
            nonlocal previous_source_us, previous_receive_mono_ns, source_elapsed_us
            nonlocal source_clock_resets
            receive_wall_ns = time.time_ns()
            receive_mono_ns = time.monotonic_ns()
            packet_index = totals.packet_count
            sequence = int(message.sequence) & UINT32_MASK
            sequence_delta, sequence_gap_before = sequence_tracker.observe(sequence)
            source_us = int(message.timestamp_us)
            source_delta_us = None
            if previous_source_us is not None:
                if source_us >= previous_source_us:
                    source_delta_us = source_us - previous_source_us
                elif previous_source_us - source_us > 0x80000000:
                    source_delta_us = (source_us - previous_source_us) & UINT32_MASK
                else:
                    source_clock_resets += 1
                if source_delta_us is not None:
                    source_elapsed_us += source_delta_us
            if previous_receive_mono_ns is not None:
                receive_intervals_ms.append((receive_mono_ns - previous_receive_mono_ns) / 1e6)
            previous_source_us = source_us
            previous_receive_mono_ns = receive_mono_ns

            counters = {
                "dropped": int(message.dropped),
                "crc_errors": int(message.crc_errors),
                "length_errors": int(message.length_errors),
                "framing_errors": int(message.framing_errors),
            }
            framing_available = bool(message.framing_errors_available)
            counter_available["framing_errors"] = counter_available["framing_errors"] or framing_available
            counter_deltas = {}
            for name, current in counters.items():
                if name == "framing_errors" and not framing_available:
                    counter_previous.pop(name, None)
                    counter_deltas[name] = None
                    continue
                previous = counter_previous.get(name)
                delta = None
                if previous is not None:
                    candidate = (current - previous) & UINT32_MASK
                    if candidate < 0x80000000:
                        delta = candidate
                        counter_totals[name] += candidate
                    else:
                        counter_resets[name] += 1
                counter_previous[name] = current
                counter_deltas[name] = delta

            points = []
            candidate_in_packet = set()
            return_in_packet = set()
            point_arrays = zip(message.angle_cdeg, message.distance_mm, message.strength)
            for point_index, values in enumerate(point_arrays):
                angle_cdeg, raw_distance_mm, strength = (int(value) for value in values)
                angle_valid = 0 <= angle_cdeg < 36000
                unknown = raw_distance_mm in NO_ECHO_VALUES
                if angle_valid:
                    totals.raw_angle_5deg_slots.add(angle_cdeg // 500)
                if unknown:
                    totals.unknown_count += 1
                else:
                    totals.echo_count += 1
                    totals.ranges.add(raw_distance_mm)
                    if POLAR_MIN_MM <= raw_distance_mm <= POLAR_MAX_MM:
                        totals.polar_eligible_count += 1
                    if angle_valid:
                        totals.return_angle_5deg_slots.add(angle_cdeg // 500)

                for name, summary in sector_summaries.items():
                    if angle_valid and _sector_contains(angle_cdeg, summary.sector):
                        candidate_in_packet.add(name)
                        if summary.add(angle_cdeg, raw_distance_mm, strength):
                            return_in_packet.add(name)

                points.append({
                    "point_index": point_index,
                    "angle_cdeg": angle_cdeg,
                    "angle_deg": round(angle_cdeg / 100.0, 2),
                    "angle_valid": angle_valid,
                    "raw_distance_mm": raw_distance_mm,
                    "distance_mm": None if unknown else raw_distance_mm,
                    "return_status": "unknown_no_echo" if unknown else "valid_return",
                    "polar_navigation_eligible_50_to_8000_mm": (
                        not unknown and POLAR_MIN_MM <= raw_distance_mm <= POLAR_MAX_MM
                    ),
                    "strength": strength,
                })

            for name in candidate_in_packet:
                sector_summaries[name].packets_with_candidate += 1
            for name in return_in_packet:
                sector_summaries[name].packets_with_return += 1

            packet_record = {
                "type": "packet",
                "packet_index": packet_index,
                "sequence": sequence,
                "sequence_delta": sequence_delta,
                "sequence_gap_before": sequence_gap_before,
                "source_timestamp_us": source_us,
                "source_timestamp_delta_us": source_delta_us,
                "source_timestamp_elapsed_us_from_capture_start": source_elapsed_us,
                "source_clock_bits": 32,
                "ros_header_stamp_ns": _message_stamp_ns(message.header.stamp),
                "collector_receive_wall_time_ns": receive_wall_ns,
                "collector_receive_monotonic_ns": receive_mono_ns,
                "frame_id": str(message.header.frame_id),
                "queue_dropped": counters["dropped"],
                "crc_errors": counters["crc_errors"],
                "length_errors": counters["length_errors"],
                "framing_errors": counters["framing_errors"],
                "framing_errors_available": framing_available,
                "counter_deltas_since_previous_packet": counter_deltas,
                "points": points,
            }
            _write_json_line(output_stream, packet_record)
            totals.packet_count += 1
            totals.point_count += len(points)

    metadata = {
        "type": "session",
        "tool": "calibrate_n10.py",
        "topic": args.topic,
        "subscriber_qos": "best_effort",
        "requested_duration_s": args.duration,
        "output_path": str(output_path.resolve()),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "sectors": [_sector_json(sector) for sector in sectors],
        "targets": [
            {
                "name": target["name"],
                "measured_distance_mm": target["measured_distance_mm"],
                "measured_body_bearing_deg": target["measured_body_bearing_deg"],
                "bearing_convention": "user-measured from robot x-forward; +y left; positive counter-clockwise",
                "raw_sector": _sector_json(target["raw_sector"]) if target["raw_sector"] else None,
                "range_comparison": "descriptive only; no pass/fail threshold",
            }
            for target in args.target
        ],
        "units": {
            "raw_angle": "angle_cdeg is 0.01 degree; angle_deg is the same value in degrees",
            "raw_distance": "millimetres; raw 0 and 65535 are retained and interpreted as unknown/no echo",
            "strength": "raw unitless byte",
            "source_timestamp_us": "FPGA packet-group timestamp in microseconds; 32-bit source clock, not per-beam acquisition time",
            "ros_header_stamp_ns": "bridge ROS clock at N10Packet publication/receive processing",
            "collector_receive_wall_time_ns": "host wall clock when this subscriber callback ran",
            "collector_receive_monotonic_ns": "host monotonic clock when this subscriber callback ran",
        },
        "coordinate_and_time_limits": [
            "Angles are recorded as supplied by the N10 parser; raw angle zero, physical handedness, and sensor-to-body yaw are not certified here.",
            "Raw angle sectors use the untransformed N10 angle values. The tool does not apply TF or infer a sensor-to-body transform.",
            "The FPGA timestamp is packet-group receive/commit time and is not a laser beam's individual scan time or a calibrated sensor-clock timestamp.",
            "header.stamp and collector callback timestamps are preserved separately; no clock synchronization is claimed.",
            "The target bearing is metadata in robot coordinates and is not used to select raw-angle points because the angle-zero/extrinsic mapping is unverified.",
            "A target raw sector is an explicit user association and can contain background returns; reported range residuals are descriptive only.",
            "Missing returns remain unknown and are never counted as free space.",
            "This is a ROS packet capture, not a raw UART byte/packet capture.",
        ],
        "calibration_status": "unverified",
    }

    try:
        _write_json_line(output_stream, metadata)
        rclpy.init(args=[])
        node = CaptureNode()
        first_wall_ns = time.time_ns()
        first_mono_ns = time.monotonic_ns()
        requested_end_mono_ns = first_mono_ns + int(args.duration * 1e9)
        print("Subscribing passively to {} for {:.1f}s; output: {}".format(
            args.topic, args.duration, output_path.resolve()), flush=True)
        try:
            while rclpy.ok():
                remaining_s = (requested_end_mono_ns - time.monotonic_ns()) / 1e9
                if remaining_s <= 0:
                    break
                rclpy.spin_once(node, timeout_sec=min(0.1, remaining_s))
        except KeyboardInterrupt:
            stopped_by_interrupt = True
        actual_end_wall_ns = time.time_ns()
        actual_end_mono_ns = time.monotonic_ns()
        durations = max(0.0, (actual_end_mono_ns - first_mono_ns) / 1e9)
        sector_results = {name: value.json() for name, value in sector_summaries.items()}
        target_results = []
        for target in args.target:
            result = {
                "name": target["name"],
                "measured_distance_mm": target["measured_distance_mm"],
                "measured_body_bearing_deg": target["measured_body_bearing_deg"],
                "raw_sector": _sector_json(target["raw_sector"]) if target["raw_sector"] else None,
                "range_residual_at_sector_median_mm": None,
                "status": "unverified",
            }
            if target["raw_sector"]:
                sector_result = sector_results[str(target["raw_sector"]["name"])]
                observed_median = sector_result["distance_mm"]["median"]
                if observed_median is not None:
                    result["range_residual_at_sector_median_mm"] = (
                        observed_median - int(target["measured_distance_mm"])
                    )
            target_results.append(result)

        receive_hz = totals.packet_count / durations if durations > 0 else None
        interval_median = statistics.median(receive_intervals_ms) if receive_intervals_ms else None
        summary = {
            "type": "summary",
            "requested_duration_s": args.duration,
            "actual_duration_s": round(durations, 6),
            "completed_duration": not stopped_by_interrupt and durations >= args.duration,
            "stopped_by_keyboard_interrupt": stopped_by_interrupt,
            "start_wall_time_ns": first_wall_ns,
            "end_wall_time_ns": actual_end_wall_ns,
            "packets_per_second": round(receive_hz, 3) if receive_hz is not None else None,
            "median_subscriber_receive_interval_ms": round(interval_median, 3) if interval_median is not None else None,
            "packet_totals": totals.json(),
            "sequence": {
                "gaps_inferred_from_sequence": sequence_tracker.gaps,
                "duplicate_sequence_packets": sequence_tracker.duplicates,
                "out_of_order_or_restart_events": sequence_tracker.out_of_order_or_restart,
            },
            "cumulative_counter_deltas_since_first_observed_packet": {
                name: (value if counter_available[name] else None)
                for name, value in counter_totals.items()
            },
            "counter_availability": counter_available,
            "counter_reset_or_backward_events": counter_resets,
            "source_clock_reset_or_backward_events": source_clock_resets,
            "source_timestamp_elapsed_us": source_elapsed_us,
            "sector_statistics": sector_results,
            "target_references": target_results,
            "calibration_status": "unverified",
            "calibration_note": (
                "Counts, distributions, and range residuals are observations only. "
                "No angle zero, handedness, TF/extrinsic, sensor delay, or calibration pass/fail is inferred."
            ),
        }
        _write_json_line(output_stream, summary)
        print("Capture summary: {} packets, {} points, {} valid returns, {} unknown/no-echo points.".format(
            totals.packet_count, totals.point_count, totals.echo_count, totals.unknown_count))
        for name, result in sector_results.items():
            dist = result["distance_mm"]
            if dist["count"]:
                print("  {} raw {}..{} deg: {} returns, median {} mm, {} unknown samples".format(
                    name,
                    result["sector"]["raw_angle_start_deg_inclusive"],
                    result["sector"]["raw_angle_end_deg_exclusive"],
                    dist["count"], dist["median"], result["unknown_no_echo_points"]))
            else:
                print("  {} raw sector: no valid returns (unknown, not free space)".format(name))
        print("Calibration status: unverified. JSONL: {}".format(output_path.resolve()))
    finally:
        try:
            if "node" in locals():
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            output_stream.close()
    return 0 if totals.packet_count else 2


if __name__ == "__main__":
    sys.exit(main())
