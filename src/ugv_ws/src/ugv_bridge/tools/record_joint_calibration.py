#!/usr/bin/env python3
"""Passively record camera, IMU and N10 through the existing ROS bridge.

Image headers are bridge publication times, not measured exposure times. IMU
and N10 FPGA timestamps remain in their original 32-bit microsecond domain.
Type 'mark LABEL' to annotate an action or 'q' to finish. This program does
not open FPGA UDP ports or send commands to the vehicle.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import platform
import queue
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path


def stamp(header):
    return {"sec": int(header.stamp.sec), "nanosec": int(header.stamp.nanosec),
            "frame_id": str(header.frame_id)}


def write_line(stream, value):
    stream.write(json.dumps(value, ensure_ascii=False, separators=(",", ":")) + "\n")


class SequenceStats:
    def __init__(self, modulus):
        self.modulus = modulus
        self.last = None
        self.gaps = 0
        self.duplicates = 0
        self.backward = 0

    def observe(self, value):
        value = int(value) % self.modulus
        if self.last is not None:
            delta = (value - self.last) % self.modulus
            if delta == 0:
                self.duplicates += 1
            elif delta >= self.modulus // 2:
                self.backward += 1
            else:
                self.gaps += delta - 1
        self.last = value

    def result(self):
        return {"missing_between_received": self.gaps, "duplicates": self.duplicates,
                "backward_or_reset": self.backward}


def run(args):
    try:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from ugv_bridge.msg import Bx72ImuRaw, N10Packet
        import numpy as np
        import cv2
        from calibrate_camera_aprilgrid import detect_aprilgrid
    except ImportError as exc:
        raise RuntimeError("source ROS Humble and ugv_ws install; OpenCV, numpy, and the camera tool are required") from exc

    output = Path(args.output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / "camera").mkdir()
    files = {name: (output / (name + ".jsonl")).open("x", encoding="utf-8", buffering=65536)
             for name in ("imu", "n10", "camera", "telemetry", "events")}
    started_wall = time.time_ns()
    started_mono = time.monotonic_ns()
    repo = Path(__file__).resolve().parents[3]
    try:
        git_head = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"],
                                           text=True, stderr=subprocess.DEVNULL).strip()
        git_status = subprocess.check_output(["git", "-C", str(repo), "status", "--short"],
                                             text=True, stderr=subprocess.DEVNULL).splitlines()
    except (OSError, subprocess.CalledProcessError):
        git_head, git_status = None, None
    metadata = {
        "schema": "bx72_joint_capture_v1", "host": platform.node(),
        "started_wall_unix_ns": started_wall, "started_monotonic_ns": started_mono,
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "camera_detector_sha256": hashlib.sha256(Path(__file__).with_name("calibrate_camera_aprilgrid.py").read_bytes()).hexdigest(),
        "source_git_head": git_head, "source_git_status": git_status,
        "opencv_version": cv2.__version__, "numpy_version": np.__version__,
        "free_disk_bytes_at_start": shutil.disk_usage(output).free,
        "camera_topic": "/camera/image_raw", "imu_topic": "/fpga/imu_raw",
        "n10_topic": "/fpga/n10_packets", "requested_camera_rate_hz": args.camera_hz,
        "coordinate_and_time_notes": [
            "Camera frame is raw BGR8; its ROS header is a bridge publication time, not a measured exposure time.",
            "IMU and N10 timestamp_us are the FPGA 32-bit microsecond snapshot clock, not per-sensor acquisition times.",
            "ROS headers and subscriber wall/monotonic receive timestamps are retained separately; no hardware synchronization is assumed.",
            "N10 0 and 65535 distances denote unknown/no echo; raw values are preserved.",
            "The recorder only subscribes to ROS; it does not send motion or FPGA control commands.",
        ],
    }
    (output / "session.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    images = queue.Queue(maxsize=8)
    commands = queue.Queue()
    counts = {"imu": 0, "n10": 0, "image_seen": 0, "image_selected": 0,
              "image_saved": 0, "image_queue_dropped": 0, "image_bad_format": 0,
              "image_write_errors": 0}
    trackers = {"imu_packet": SequenceStats(1 << 32), "imu_reader": SequenceStats(1 << 16),
                "n10_packet": SequenceStats(1 << 32)}
    last_seen = {name: None for name in ("imu", "n10", "camera", "motor", "navigation", "fusion")}
    first_seen = {name: None for name in ("imu", "n10", "camera")}
    max_gap_ns = {name: 0 for name in first_seen}
    last_preview = {"tags": None, "corners": None, "bbox": None, "frame": None,
                    "received_monotonic_ns": None, "error": None}
    safety = {"pwm_percent": None, "target_mm_s": None}
    counter_ranges = {"imu_dropped": [], "n10_dropped": [], "n10_crc_errors": [], "n10_length_errors": []}

    def camera_writer():
        next_detection = 0.0
        while True:
            item = images.get()
            try:
                if item is None:
                    return
                index, raw, info = item
                name = f"camera/frame_{index:06d}.bgr8"
                with (output / name).open("xb") as stream:
                    stream.write(raw)
                info.update({"file": name, "sha256": hashlib.sha256(raw).hexdigest()})
                counts["image_saved"] += 1
                if info["received_monotonic_ns"] / 1e9 >= next_detection:
                    next_detection = info["received_monotonic_ns"] / 1e9 + 2.0
                    try:
                        rows = np.frombuffer(raw, dtype=np.uint8).reshape(info["height"], info["step"])
                        bgr = rows[:, :info["width"] * 3].reshape(info["height"], info["width"], 3)
                        result = detect_aprilgrid(bgr)
                        points = [p for marker in result["records"] if marker["id"] in result["unique_valid_ids"]
                                  for p in marker["corners_used_original_px"]]
                        box = ([round(min(p[0] for p in points)), round(min(p[1] for p in points)),
                                round(max(p[0] for p in points)), round(max(p[1] for p in points))]
                               if points else None)
                        preview = {"tags": result["detected_tags"], "ids": result["unique_valid_ids"],
                                   "corners": result["detected_corners"], "bbox": box,
                                   "geometry_ok": result["geometry_check"]["geometry_ok"]}
                        info["preview"] = preview
                        last_preview.update({"tags": preview["tags"], "corners": preview["corners"],
                                             "bbox": box, "frame": index,
                                             "received_monotonic_ns": info["received_monotonic_ns"], "error": None})
                    except Exception as exc:
                        info["preview_error"] = str(exc)
                        last_preview["error"] = str(exc)
                write_line(files["camera"], info)
            except Exception as exc:
                counts["image_write_errors"] += 1
                last_preview["error"] = str(exc)
            finally:
                images.task_done()

    def read_commands():
        for line in sys.stdin:
            commands.put(line.strip())

    worker = threading.Thread(target=camera_writer, daemon=True)
    worker.start()
    threading.Thread(target=read_commands, daemon=True).start()
    rclpy.init(args=[])
    node = rclpy.create_node("bx72_joint_calibration_capture")
    qos = QoSProfile(depth=1024, reliability=ReliabilityPolicy.BEST_EFFORT)
    camera_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    previous_image_at = 0.0

    def reception():
        return {"received_wall_unix_ns": time.time_ns(), "received_monotonic_ns": time.monotonic_ns()}

    def seen(name, at):
        if name in first_seen:
            if first_seen[name] is None:
                first_seen[name] = at
            if last_seen[name] is not None:
                max_gap_ns[name] = max(max_gap_ns[name], at - last_seen[name])
        last_seen[name] = at

    def observe_counter(name, value):
        values = counter_ranges[name]
        if not values:
            values.append(int(value))
        if len(values) == 1:
            values.append(int(value))
        else:
            values[-1] = int(value)

    def imu_callback(msg):
        now = reception()
        seen("imu", now["received_monotonic_ns"])
        trackers["imu_packet"].observe(msg.sequence)
        trackers["imu_reader"].observe(msg.reader_sequence)
        observe_counter("imu_dropped", msg.dropped)
        write_line(files["imu"], {**now, "header": stamp(msg.header), "sequence": int(msg.sequence),
                   "timestamp_us": int(msg.timestamp_us), "dropped": int(msg.dropped),
                   "reader_sequence": int(msg.reader_sequence), "flags": int(msg.flags),
                   "who_am_i": int(msg.who_am_i), "accel_raw": [int(value) for value in msg.accel_raw],
                   "gyro_raw": [int(value) for value in msg.gyro_raw]})
        counts["imu"] += 1

    def n10_callback(msg):
        now = reception()
        seen("n10", now["received_monotonic_ns"])
        trackers["n10_packet"].observe(msg.sequence)
        for name, value in (("n10_dropped", msg.dropped), ("n10_crc_errors", msg.crc_errors),
                            ("n10_length_errors", msg.length_errors)):
            observe_counter(name, value)
        write_line(files["n10"], {**now, "header": stamp(msg.header), "sequence": int(msg.sequence),
                   "timestamp_us": int(msg.timestamp_us), "dropped": int(msg.dropped),
                   "crc_errors": int(msg.crc_errors), "length_errors": int(msg.length_errors),
                   "framing_errors": int(msg.framing_errors), "framing_errors_available": bool(msg.framing_errors_available),
                   "angle_cdeg": [int(value) for value in msg.angle_cdeg],
                   "distance_mm": [int(value) for value in msg.distance_mm],
                   "strength": [int(value) for value in msg.strength]})
        counts["n10"] += 1

    def image_callback(msg):
        nonlocal previous_image_at
        now = reception()
        seen("camera", now["received_monotonic_ns"])
        counts["image_seen"] += 1
        if msg.width != 640 or msg.height != 360 or msg.encoding != "bgr8" or msg.step < msg.width * 3:
            counts["image_bad_format"] += 1
            return
        seconds = now["received_monotonic_ns"] / 1e9
        if seconds - previous_image_at < 1.0 / args.camera_hz:
            return
        previous_image_at = seconds
        data = bytes(msg.data)
        if len(data) != msg.step * msg.height:
            counts["image_bad_format"] += 1
            return
        counts["image_selected"] += 1
        info = {**now, "header": stamp(msg.header), "width": int(msg.width), "height": int(msg.height),
                "encoding": msg.encoding, "step": int(msg.step), "is_bigendian": int(msg.is_bigendian)}
        try:
            images.put_nowait((counts["image_selected"], data, info))
        except queue.Full:
            counts["image_queue_dropped"] += 1

    def telemetry_callback(name, msg):
        now = reception()
        if name in last_seen:
            last_seen[name] = now["received_monotonic_ns"]
        try:
            payload = json.loads(msg.data)
        except ValueError:
            payload = {"unparsed": msg.data}
        write_line(files["telemetry"], {**now, "type": name, "payload": payload})
        if name == "motor":
            safety["pwm_percent"] = payload.get("pwm_percent")
            safety["target_mm_s"] = payload.get("target_mm_s")

    subscriptions = [node.create_subscription(Image, "/camera/image_raw", image_callback, camera_qos),
                     node.create_subscription(Bx72ImuRaw, "/fpga/imu_raw", imu_callback, qos),
                     node.create_subscription(N10Packet, "/fpga/n10_packets", n10_callback, qos)]
    for topic in ("motor", "navigation", "fusion", "health"):
        subscriptions.append(node.create_subscription(String, "/fpga/round5/" + topic,
                              lambda msg, name=topic: telemetry_callback(name, msg), qos))
    print(f"Recording existing ROS streams to {output}; type 'mark LABEL' or 'q'.", flush=True)
    write_line(files["events"], {"event": "started", **reception()})
    deadline = time.monotonic() + args.duration
    next_status = 0.0
    interrupted = False
    halt_reason = None
    capture_error = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                while True:
                    command = commands.get_nowait()
                    if command.lower() in ("q", "quit"):
                        deadline = 0
                        break
                    if command.startswith("mark "):
                        write_line(files["events"], {"event": "operator_mark", "label": command[5:128], **reception()})
            except queue.Empty:
                pass
            targets = safety["target_mm_s"]
            if ((safety["pwm_percent"] is not None and safety["pwm_percent"] != 0)
                    or (isinstance(targets, dict) and any(value != 0 for value in targets.values()))):
                halt_reason = "nonzero motor output observed"
                print("STOP recording: " + halt_reason + "; no control command was sent", flush=True)
                break
            now = time.monotonic()
            if now >= next_status:
                next_status = now + 2.0
                ages = {name: None if value is None else round((time.monotonic_ns() - value) / 1e9, 3)
                        for name, value in last_seen.items()}
                stale = [name for name in ("camera", "imu", "n10") if ages[name] is None or ages[name] > 1.0]
                preview_age = (None if last_preview["received_monotonic_ns"] is None else
                               round((time.monotonic_ns() - last_preview["received_monotonic_ns"]) / 1e9, 3))
                print(json.dumps({"elapsed_s": round(now - started_mono / 1e9, 1), "counts": counts,
                                  "ages_s": ages, "stale": stale, "tag_preview": last_preview,
                                  "tag_preview_age_s": preview_age,
                                  "safety_last_observed": safety}, ensure_ascii=False), flush=True)
                for stream in files.values():
                    stream.flush()
    except KeyboardInterrupt:
        interrupted = True
    except Exception as exc:
        capture_error = f"{type(exc).__name__}: {exc}"
        print("ERROR during capture: " + capture_error, file=sys.stderr, flush=True)
    finally:
        collection_end_ns = time.monotonic_ns()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        images.put(None)
        worker.join()
        write_line(files["events"], {"event": "stopped", "interrupted": interrupted, **reception()})
        for stream in files.values():
            stream.close()

    streams = {name: {"first_received_monotonic_ns": first_seen[name],
                      "last_received_monotonic_ns": last_seen[name],
                      "receive_span_s": (last_seen[name] - first_seen[name]) / 1e9 if first_seen[name] is not None else None,
                      "largest_receive_gap_s": max_gap_ns[name] / 1e9 if first_seen[name] is not None else None,
                      "age_at_stop_s": (collection_end_ns - last_seen[name]) / 1e9 if last_seen[name] is not None else None}
               for name in first_seen}
    summary = {"schema": "bx72_joint_capture_v1", "ended_wall_unix_ns": time.time_ns(),
               "elapsed_s": (time.monotonic_ns() - started_mono) / 1e9, "interrupted": interrupted,
               "halt_reason": halt_reason, "capture_error": capture_error,
               "counts": counts, "sequences": {k: v.result() for k, v in trackers.items()},
               "stream_receive_timing": streams,
               "counter_first_last": counter_ranges, "safety_last_observed": safety,
               "camera_preview_last": last_preview,
               "data_quality_note": "Sequence gaps can occur anywhere before this subscriber; check counters and inspect actual data.",
               "calibration_status": "unverified; recording alone is not an extrinsic solution"}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    return 0 if (counts["imu"] and counts["n10"] and counts["image_saved"]
                 and not counts["image_write_errors"] and not halt_reason and not capture_error) else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="new directory for raw records")
    parser.add_argument("--duration", type=float, default=180.0, help="maximum duration in seconds")
    parser.add_argument("--camera-hz", type=float, default=10.0, help="maximum saved image rate")
    args = parser.parse_args()
    if not 1 <= args.duration <= 600 or not 0.5 <= args.camera_hz <= 15:
        parser.error("duration must be 1..600 seconds and camera rate 0.5..15 Hz")
    try:
        return run(args)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
