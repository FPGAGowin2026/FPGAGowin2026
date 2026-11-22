#!/usr/bin/env python3
"""ACG720 UDP 解析；复用 fpga_ws 提交 6bab90d 的视频、IMU、雷达与步态状态实现。"""

from __future__ import annotations

import select
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any

try:
    from PIL import Image
except ImportError:  # 仅在保存或显示图像时需要 Pillow
    Image = None  # type: ignore[assignment,misc]


# ACV1 每包 400 像素；ACV2 每包 640 个连续像素，两者均使用 !4sHH 头部。
MAGIC = b"ACV1"
MAGIC_V2 = b"ACV2"
HEADER = struct.Struct("!4sHH")
HEADER_BYTES = HEADER.size
WIDTH = 800
HEIGHT = 480
BLOCK_WIDTH = 400
BLOCK_HEIGHT = 1
BLOCKS_PER_ROW = WIDTH // BLOCK_WIDTH
BLOCK_ROWS = HEIGHT // BLOCK_HEIGHT
BLOCK_COUNT = BLOCKS_PER_ROW * BLOCK_ROWS
PIXELS_PER_BLOCK = BLOCK_WIDTH * BLOCK_HEIGHT
PIXEL_BYTES = 2
PAYLOAD_BYTES = PIXELS_PER_BLOCK * PIXEL_BYTES
DATAGRAM_BYTES = HEADER_BYTES + PAYLOAD_BYTES
FRAME_BYTES = WIDTH * HEIGHT * PIXEL_BYTES
V2_PIXELS_PER_BLOCK = 640
V2_PAYLOAD_BYTES = V2_PIXELS_PER_BLOCK * PIXEL_BYTES
V2_BLOCK_COUNT = FRAME_BYTES // V2_PAYLOAD_BYTES
VIDEO_FORMATS = {
    MAGIC: (BLOCK_COUNT, PAYLOAD_BYTES),
    MAGIC_V2: (V2_BLOCK_COUNT, V2_PAYLOAD_BYTES),
}
# One spare byte keeps an oversized datagram from looking like a valid packet
# after POSIX truncation. Winsock reports WSAEMSGSIZE instead, so read it whole.
RECEIVE_BYTES = (65535 if sys.platform == "win32" else
                 HEADER_BYTES + max(size for _, size in VIDEO_FORMATS.values()) + 1)
MAX_IN_FLIGHT_FRAMES = 4
DEFAULT_PORT = 6102
DEFAULT_CAPTURE = "camera_capture.png"
FRAME_ID_MASK = 0xFFFF
FRAME_ID_HALF_RANGE = 0x8000
PREVIEW_RECOVERY_MIN_PERCENT = 80
VIDEO_RESTART_GAP_SECONDS = 0.75

# IMU1 与视频共用 UDP 端口；序号为网络字节序，轴数据为有符号大端 int16。
IMU_MAGIC = b"IMU1"
IMU_PACKET = struct.Struct("!4sIBB6h")
IMU_PACKET_BYTES = IMU_PACKET.size
IMU_READY_FLAG = 0x01
IMU_STALE_SECONDS = 0.75

# LDR1 与视频和 IMU 共用 UDP 端口：4 字节封装头后是未经改写的 47 字节雷达帧。
LIDAR_MAGIC = b"LDR1"
LIDAR_UART_HEADER = b"\x54\x2c"
LIDAR_UART_PACKET_BYTES = 47
LIDAR_DATAGRAM_BYTES = len(LIDAR_MAGIC) + LIDAR_UART_PACKET_BYTES
LIDAR_POINTS_PER_PACKET = 12
LIDAR_FULL_TURN_CDEG = 36000
LIDAR_CRC_POLY = 0x4D
LIDAR_STALE_SECONDS = 0.75
LIDAR_MAX_FORWARD_STEP_CDEG = 18000
LIDAR_VIEW_SIZE = 480
LIDAR_MAX_RANGE_MM = 12000

# HXS1 与视频、IMU 和雷达共用 UDP 6102；末字节 bit 7 表示 PWM 已使能。
TELEOP_STATUS_MAGIC = b"HXS1"
TELEOP_STATUS_BYTES = 7
TELEOP_STALE_SECONDS = 1.5
TELEOP_MOVEMENT_COUNT = 8


@dataclass
class _FrameBuffer:
    pixels: bytearray
    received: bytearray
    magic: bytes
    block_payload_bytes: int
    expected_blocks: int
    last_receipt: tuple[int, float]
    block_count: int = 0

    @classmethod
    def create(
        cls, magic: bytes, expected_blocks: int, block_payload_bytes: int, receipt: tuple[int, float]
    ) -> "_FrameBuffer":
        return cls(
            bytearray(FRAME_BYTES),
            bytearray(expected_blocks),
            magic,
            block_payload_bytes,
            expected_blocks,
            receipt,
        )

    def add_block(self, block_index: int, payload: bytes, receipt: tuple[int, float]) -> bool:
        """按块在帧中的线性偏移放入 RGB565 数据。"""
        if self.received[block_index]:
            return False

        destination_start = block_index * self.block_payload_bytes
        self.pixels[
            destination_start : destination_start + self.block_payload_bytes
        ] = payload

        self.received[block_index] = 1
        self.block_count += 1
        self.last_receipt = receipt
        return True


@dataclass(frozen=True)
class _LidarPacket:
    speed_deg_s: int
    start_angle_cdeg: int
    end_angle_cdeg: int
    timestamp_ms: int
    points: tuple[tuple[float, int, int], ...]


def _make_lidar_crc_table() -> tuple[int, ...]:
    table = []
    for value in range(256):
        for _ in range(8):
            if value & 0x80:
                value = ((value << 1) ^ LIDAR_CRC_POLY) & 0xFF
            else:
                value = (value << 1) & 0xFF
        table.append(value)
    return tuple(table)


_LIDAR_CRC_TABLE = _make_lidar_crc_table()


def _lidar_crc8(data: bytes) -> int:
    """计算雷达帧使用的 CRC-8（poly=0x4D，初值 0）。"""
    crc = 0
    table = _LIDAR_CRC_TABLE
    for byte in data:
        crc = table[crc ^ byte]
    return crc


def _decode_lidar_uart_packet(raw: bytes) -> _LidarPacket:
    """校验并解析一个完整的 47 字节雷达串口帧。"""
    if len(raw) != LIDAR_UART_PACKET_BYTES:
        raise ValueError("雷达帧长度必须为 47 字节")
    if raw[:2] != LIDAR_UART_HEADER:
        raise ValueError("雷达帧头必须为 54 2C")
    if _lidar_crc8(raw[:-1]) != raw[-1]:
        raise ValueError("雷达帧 CRC-8 不匹配")

    speed_deg_s = int.from_bytes(raw[2:4], "little")
    start_angle = int.from_bytes(raw[4:6], "little")
    end_angle = int.from_bytes(raw[42:44], "little")
    timestamp_ms = int.from_bytes(raw[44:46], "little")
    if start_angle >= LIDAR_FULL_TURN_CDEG or end_angle >= LIDAR_FULL_TURN_CDEG:
        raise ValueError("雷达角度超出 [0, 36000) 百分之一度")

    angle_delta = (end_angle - start_angle) % LIDAR_FULL_TURN_CDEG
    points = []
    for index in range(LIDAR_POINTS_PER_PACKET):
        offset = 6 + index * 3
        distance_mm = int.from_bytes(raw[offset : offset + 2], "little")
        intensity = raw[offset + 2]
        angle_cdeg = (
            start_angle + angle_delta * index / (LIDAR_POINTS_PER_PACKET - 1)
        ) % LIDAR_FULL_TURN_CDEG
        points.append((angle_cdeg / 100.0, distance_mm, intensity))

    return _LidarPacket(
        speed_deg_s=speed_deg_s,
        start_angle_cdeg=start_angle,
        end_angle_cdeg=end_angle,
        timestamp_ms=timestamp_ms,
        points=tuple(points),
    )


def _frame_id_is_newer(frame_id: int, reference_id: int) -> bool:
    distance = (frame_id - reference_id) & FRAME_ID_MASK
    return 0 < distance < FRAME_ID_HALF_RANGE


def _gray_to_binary(value: int) -> int:
    value ^= value >> 1
    value ^= value >> 2
    value ^= value >> 4
    return value


def _packet_receipt(received_ns: int | None) -> tuple[int, float]:
    # SO_TIMESTAMPNS uses the wall clock. Carry the queue delay into monotonic freshness checks.
    now_ns = time.time_ns()
    now = time.monotonic()
    if received_ns is None:
        return now_ns, now
    return received_ns, now - max(0, now_ns - received_ns) * 1e-9


class UdpCameraReceiver:
    """后台接收线程和帧组装器。"""

    def __init__(
        self, bind_address: str, port: int, *,
        source_address: str | None = None, source_port: int | None = None,
    ) -> None:
        if type(port) is not int or not 0 <= port <= 65535:
            raise ValueError("port 必须在 0..65535 范围")
        if source_address:
            try:
                socket.inet_pton(socket.AF_INET, source_address)
            except OSError as exc:
                raise ValueError("source_address 必须是 IPv4 地址") from exc
        if source_port is not None and (type(source_port) is not int or not 1 <= source_port <= 65535):
            raise ValueError("source_port 必须在 1..65535 范围")
        self.bind_address = bind_address
        self.source_address = source_address or None
        self.source_port = source_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            self.receive_buffer_bytes = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
            # Linux 64-bit SO_TIMESTAMPNS retains arrival time while the UDP queue drains.
            self._timestamp_ns = getattr(socket, 'SO_TIMESTAMPNS', 35) if sys.platform == 'linux' else None
            if self._timestamp_ns is not None:
                self.sock.setsockopt(socket.SOL_SOCKET, self._timestamp_ns, 1)
            self.sock.bind((bind_address, port))
            self.port = self.sock.getsockname()[1]
            self.sock.setblocking(False)
        except Exception:
            self.sock.close()
            raise

        self._condition = threading.Condition()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._receive_loop, name="udp-camera-receiver", daemon=True
        )
        self._frames: dict[int, _FrameBuffer] = {}

        self.packets_received = 0
        self.valid_packets = 0
        self.invalid_packets = 0
        self.filtered_packets = 0
        self.video_packets = 0
        self.stale_video_packets = 0
        self.duplicate_packets = 0
        self.complete_frames = 0
        self.incomplete_frames_dropped = 0
        self.latest_packet_frame_id: int | None = None
        self.latest_packet_frame_blocks = 0
        self.latest_packet_frame_block_count = BLOCK_COUNT
        self.latest_completed_frame_id: int | None = None
        self.latest_completed_frame: bytes | None = None
        self.latest_completed_frame_stamp_ns: int | None = None
        self.frame_version = 0
        self._latest_complete_epoch: int | None = None
        self.latest_preview_frame_id: int | None = None
        self.latest_preview_frame: bytes | None = None
        self.latest_preview_stamp_ns: int | None = None
        self._latest_preview_rx_monotonic: float | None = None
        self.latest_preview_recovered = False
        self.preview_version = 0
        self.recovered_preview_count = 0
        self._latest_preview_epoch: int | None = None
        self._latest_source_frame_id: int | None = None
        self._video_epoch = 0
        self._last_video_packet_time: float | None = None
        self._last_valid_pixels: bytearray | None = None
        self._last_valid_epoch: int | None = None
        self._last_valid_frame_id: int | None = None
        self.imu_packets = 0
        self.imu_ready_packets = 0
        self.imu_unready_packets = 0
        self.invalid_imu_packets = 0
        self.latest_imu_seq: int | None = None
        self.latest_imu_ready: bool | None = None
        self.latest_imu_who_am_i: int | None = None
        self.latest_imu_raw: tuple[int, int, int, int, int, int] | None = None
        self.latest_imu_rx_monotonic: float | None = None
        self.latest_imu_stamp_ns: int | None = None
        self.imu_version = 0
        self.lidar_valid_packets = 0
        self.lidar_invalid_packets = 0
        self.lidar_valid_points = 0
        self.lidar_sweeps = 0
        self.lidar_out_of_order_packets = 0
        self.latest_lidar_speed_deg_s: int | None = None
        self.latest_lidar_start_angle_cdeg: int | None = None
        self.latest_lidar_end_angle_cdeg: int | None = None
        self.latest_lidar_timestamp_ms: int | None = None
        self.latest_lidar_rx_monotonic: float | None = None
        self.lidar_version = 0
        self._lidar_sweep_points: list[tuple[float, int, int]] = []
        self._lidar_completed_sweep: tuple[tuple[float, int, int], ...] = ()
        self._lidar_sweep_started = False
        self._lidar_sweep_start_ns: int | None = None
        self._lidar_sweep_start_monotonic: float | None = None
        self._lidar_completed_sweep_stamp_ns: int | None = None
        self._lidar_completed_sweep_time_s = 0.
        self._last_lidar_start_angle_cdeg: int | None = None
        self.teleop_status_packets = 0
        self.invalid_teleop_status_packets = 0
        self.latest_teleop_requested_command: int | None = None
        self.latest_teleop_active_gait: int | None = None
        self.latest_teleop_ripple_selected: bool | None = None
        self.latest_teleop_trajectory_ripple: bool | None = None
        self.latest_teleop_homing: bool | None = None
        self.latest_teleop_frame_index: int | None = None
        self.latest_teleop_debug: str | None = None
        self.latest_teleop_rx_debug: dict[str, int] | None = None
        self.latest_teleop_pwm_enabled: bool | None = None
        self.latest_teleop_rx_monotonic: float | None = None
        self.receiver_error: str | None = None

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        with self._condition:
            self._condition.notify_all()
        try:
            self.sock.close()
        except OSError:
            pass
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def wait_for_frames(self, target_count: int, timeout: float | None) -> bool:
        with self._condition:
            return self._condition.wait_for(
                lambda: self.complete_frames >= target_count
                or self.receiver_error is not None or self._stop_event.is_set(),
                timeout=timeout,
            ) and self.complete_frames >= target_count

    def wait_for_imu(self, target_count: int, timeout: float | None) -> bool:
        with self._condition:
            return self._condition.wait_for(
                lambda: self.imu_packets >= target_count
                or self.receiver_error is not None or self._stop_event.is_set(),
                timeout=timeout,
            ) and self.imu_packets >= target_count

    def preview(self):
        with self._condition:
            return self.preview_version, self.latest_preview_frame

    def snapshot(self) -> dict[str, Any]:
        with self._condition:
            now = time.monotonic()

            def age(stamp):
                return None if stamp is None else max(0.0, now - stamp)

            def state(elapsed, expiry):
                if elapsed is None:
                    return "WAITING"
                return "STALE" if elapsed > expiry else "READY"

            lidar_age = age(self.latest_lidar_rx_monotonic)
            teleop_age = age(self.latest_teleop_rx_monotonic)
            return {
                "bind_address": self.bind_address,
                "port": self.port,
                "source_address": self.source_address,
                "source_port": self.source_port,
                "receive_buffer_bytes": self.receive_buffer_bytes,
                "video_age_s": age(self._last_video_packet_time),
                "packets_received": self.packets_received,
                "valid_packets": self.valid_packets,
                "invalid_packets": self.invalid_packets,
                "filtered_packets": self.filtered_packets,
                "video_packets": self.video_packets,
                "stale_video_packets": self.stale_video_packets,
                "duplicate_packets": self.duplicate_packets,
                "complete_frames": self.complete_frames,
                "incomplete_frames_dropped": self.incomplete_frames_dropped,
                "inflight_frames": len(self._frames),
                "latest_packet_frame_id": self.latest_packet_frame_id,
                "latest_packet_frame_blocks": self.latest_packet_frame_blocks,
                "latest_packet_frame_block_count": self.latest_packet_frame_block_count,
                "latest_completed_frame_id": self.latest_completed_frame_id,
                "latest_completed_frame": self.latest_completed_frame,
                "latest_completed_frame_stamp_ns": self.latest_completed_frame_stamp_ns,
                "frame_version": self.frame_version,
                "latest_preview_frame_id": self.latest_preview_frame_id,
                "latest_preview_frame": self.latest_preview_frame,
                "latest_preview_stamp_ns": self.latest_preview_stamp_ns,
                "preview_age_s": age(self._latest_preview_rx_monotonic),
                "latest_preview_recovered": self.latest_preview_recovered,
                "preview_version": self.preview_version,
                "recovered_preview_count": self.recovered_preview_count,
                "imu_packets": self.imu_packets,
                "imu_ready_packets": self.imu_ready_packets,
                "imu_unready_packets": self.imu_unready_packets,
                "invalid_imu_packets": self.invalid_imu_packets,
                "latest_imu_seq": self.latest_imu_seq,
                "latest_imu_ready": self.latest_imu_ready,
                "latest_imu_who_am_i": self.latest_imu_who_am_i,
                "latest_imu_raw": self.latest_imu_raw,
                "latest_imu_age_s": age(self.latest_imu_rx_monotonic),
                "latest_imu_stamp_ns": self.latest_imu_stamp_ns,
                "imu_version": self.imu_version,
                "lidar_completed_sweep": self._lidar_completed_sweep,
                "lidar_completed_sweep_stamp_ns": self._lidar_completed_sweep_stamp_ns,
                "lidar_completed_sweep_time_s": self._lidar_completed_sweep_time_s,
                "lidar_version": self.lidar_version,
                "lidar_age_s": lidar_age,
                "lidar_state": state(lidar_age, LIDAR_STALE_SECONDS),
                "lidar_speed_deg_s": self.latest_lidar_speed_deg_s,
                "lidar_start_angle_deg": (
                    None
                    if self.latest_lidar_start_angle_cdeg is None
                    else self.latest_lidar_start_angle_cdeg / 100.0
                ),
                "lidar_end_angle_deg": (
                    None
                    if self.latest_lidar_end_angle_cdeg is None
                    else self.latest_lidar_end_angle_cdeg / 100.0
                ),
                "lidar_timestamp_ms": self.latest_lidar_timestamp_ms,
                "lidar_valid_packets": self.lidar_valid_packets,
                "lidar_invalid_packets": self.lidar_invalid_packets,
                "lidar_valid_points": self.lidar_valid_points,
                "lidar_sweeps": self.lidar_sweeps,
                "lidar_out_of_order_packets": self.lidar_out_of_order_packets,
                "teleop_status_packets": self.teleop_status_packets,
                "teleop_status_version": self.teleop_status_packets,
                "invalid_teleop_status_packets": self.invalid_teleop_status_packets,
                "teleop_state": state(teleop_age, TELEOP_STALE_SECONDS),
                "teleop_age_s": teleop_age,
                "teleop_requested_command": self.latest_teleop_requested_command,
                "teleop_active_gait": self.latest_teleop_active_gait,
                "teleop_ripple_selected": self.latest_teleop_ripple_selected,
                "teleop_trajectory_ripple": self.latest_teleop_trajectory_ripple,
                "teleop_homing": self.latest_teleop_homing,
                "teleop_frame_index": self.latest_teleop_frame_index,
                "teleop_debug": self.latest_teleop_debug,
                "teleop_rx_debug": None if self.latest_teleop_rx_debug is None else self.latest_teleop_rx_debug.copy(),
                "teleop_pwm_enabled": self.latest_teleop_pwm_enabled,
                "receiver_error": self.receiver_error,
            }

    def _receive_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                # Drain queued packets directly; wait only when the queue is empty.
                # A timeout socket otherwise polls before every recvfrom call.
                try:
                    received_ns = None
                    if self._timestamp_ns is None:
                        datagram, source = self.sock.recvfrom(RECEIVE_BYTES)
                    else:
                        datagram, ancillary, _, source = self.sock.recvmsg(
                            RECEIVE_BYTES, socket.CMSG_SPACE(struct.calcsize('@ll')))
                        for level, kind, value in ancillary:
                            if level == socket.SOL_SOCKET and kind == self._timestamp_ns:
                                seconds, nanoseconds = struct.unpack('@ll', value)
                                received_ns = seconds * 1000000000 + nanoseconds
                except BlockingIOError:
                    select.select([self.sock], [], [], 0.25)
                    continue
            except (OSError, ValueError) as exc:
                if not self._stop_event.is_set():
                    with self._condition:
                        self.receiver_error = str(exc)
                        self._condition.notify_all()
                break

            self._handle_datagram(datagram, source, received_ns=received_ns)

    def _accept_lidar_packet(self, packet: _LidarPacket, now: float, received_ns: int | None = None) -> bool:
        """按角度顺序收集当前一圈的测点；返回 False 表示重复或乱序。"""
        if (
            self.latest_lidar_rx_monotonic is not None
            and now - self.latest_lidar_rx_monotonic > LIDAR_STALE_SECONDS
        ):
            # 长时间断流后不把新一圈和旧点云拼在一起。
            self._lidar_sweep_points.clear()
            self._lidar_completed_sweep = ()
            self._lidar_sweep_started = False
            self._lidar_sweep_start_ns = None
            self._lidar_sweep_start_monotonic = None
            self._lidar_completed_sweep_stamp_ns = None
            self._lidar_completed_sweep_time_s = 0.
            self._last_lidar_start_angle_cdeg = None

        previous_start = self._last_lidar_start_angle_cdeg
        if previous_start is not None:
            forward_step = (
                packet.start_angle_cdeg - previous_start
            ) % LIDAR_FULL_TURN_CDEG
            if forward_step == 0 or forward_step > LIDAR_MAX_FORWARD_STEP_CDEG:
                self.lidar_out_of_order_packets += 1
                return False

            if packet.start_angle_cdeg < previous_start:
                if self._lidar_sweep_started:
                    self._lidar_completed_sweep = tuple(self._lidar_sweep_points)
                    self._lidar_completed_sweep_stamp_ns = self._lidar_sweep_start_ns
                    self._lidar_completed_sweep_time_s = (
                        ((received_ns - self._lidar_sweep_start_ns) * 1e-9)
                        if received_ns is not None else now - self._lidar_sweep_start_monotonic)
                self._lidar_sweep_started = True
                self._lidar_sweep_start_ns = time.time_ns() if received_ns is None else received_ns
                self._lidar_sweep_start_monotonic = now
                self._lidar_sweep_points.clear()
                self.lidar_sweeps += 1

        self._last_lidar_start_angle_cdeg = packet.start_angle_cdeg
        self._lidar_sweep_points.extend(
            point for point in packet.points if point[1] > 0
        )
        self.lidar_valid_packets += 1
        self.lidar_valid_points += sum(point[1] > 0 for point in packet.points)
        self.latest_lidar_speed_deg_s = packet.speed_deg_s
        self.latest_lidar_start_angle_cdeg = packet.start_angle_cdeg
        self.latest_lidar_end_angle_cdeg = packet.end_angle_cdeg
        self.latest_lidar_timestamp_ms = packet.timestamp_ms
        self.latest_lidar_rx_monotonic = now
        self.lidar_version += 1
        return True

    def _begin_new_video_epoch_after_gap(self, now: float) -> None:
        if (
            self._last_video_packet_time is not None
            and now - self._last_video_packet_time > VIDEO_RESTART_GAP_SECONDS
        ):
            for frame in self._frames.values():
                if frame.block_count < frame.expected_blocks:
                    self.incomplete_frames_dropped += 1
            self._frames.clear()
            self._latest_source_frame_id = None
            self._last_valid_pixels = None
            self._last_valid_epoch = None
            self._last_valid_frame_id = None
            self._video_epoch += 1
        self._last_video_packet_time = now

    def _can_publish_preview(self, frame_id: int, recovered: bool) -> bool:
        if (
            self._latest_preview_epoch != self._video_epoch
            or self.latest_preview_frame_id is None
        ):
            return True
        if frame_id == self.latest_preview_frame_id:
            return self.latest_preview_recovered and not recovered
        return _frame_id_is_newer(frame_id, self.latest_preview_frame_id)

    def _publish_preview(self, frame_id: int, pixels: bytes, recovered: bool,
                         receipt: tuple[int, float]) -> None:
        self.latest_preview_frame_id = frame_id
        self.latest_preview_frame = pixels
        self.latest_preview_stamp_ns, self._latest_preview_rx_monotonic = receipt
        self.latest_preview_recovered = recovered
        self._latest_preview_epoch = self._video_epoch
        self.preview_version += 1
        if recovered:
            self.recovered_preview_count += 1

    def _try_recover_preview(self, frame_id: int, frame: _FrameBuffer) -> None:
        if (
            frame.block_count * 100
            < frame.expected_blocks * PREVIEW_RECOVERY_MIN_PERCENT
        ):
            return
        if (
            self._last_valid_pixels is None
            or self._last_valid_epoch != self._video_epoch
            or self._last_valid_frame_id != frame_id
        ):
            return
        if not self._can_publish_preview(frame_id, recovered=True):
            return

        preview = bytes(self._last_valid_pixels)
        self._publish_preview(frame_id, preview, recovered=True, receipt=frame.last_receipt)

    def _handle_datagram(self, datagram: bytes, source: tuple[str, int] | None = None, *,
                         received_ns: int | None = None) -> None:
        with self._condition:
            self.packets_received += 1
            if ((self.source_address is not None and (source is None or source[0] != self.source_address))
                    or (self.source_port is not None and (source is None or source[1] != self.source_port))):
                self.filtered_packets += 1
                return

            if datagram.startswith(TELEOP_STATUS_MAGIC):
                if len(datagram) not in (TELEOP_STATUS_BYTES, 10):
                    self.invalid_packets += 1
                    self.invalid_teleop_status_packets += 1
                    return
                requested_flags, active_flags, pwm_frame = datagram[4:7]
                requested_command = requested_flags & 0x07
                active_gait = active_flags & 0x07
                frame_index = pwm_frame & 0x7F
                if (
                    requested_command >= TELEOP_MOVEMENT_COUNT
                    or requested_flags & 0x38
                    or active_flags & 0x78
                    or frame_index >= 90
                ):
                    self.invalid_packets += 1
                    self.invalid_teleop_status_packets += 1
                    return
                self.teleop_status_packets += 1
                self.valid_packets += 1
                self.latest_teleop_requested_command = requested_command
                self.latest_teleop_active_gait = active_gait
                self.latest_teleop_ripple_selected = bool(requested_flags & 0x80)
                self.latest_teleop_trajectory_ripple = bool(requested_flags & 0x40)
                self.latest_teleop_homing = bool(active_flags & 0x80)
                self.latest_teleop_frame_index = frame_index
                self.latest_teleop_debug = datagram[7:].hex()
                if len(datagram) == 10:
                    debug = int.from_bytes(datagram[7:], "big")
                    self.latest_teleop_rx_debug = {
                        name: _gray_to_binary((debug >> shift) & 0x3F)
                        for name, shift in (("rx_clock_heartbeat", 18), ("dv_start", 12),
                                            ("sfd", 6), ("crc_length", 0))
                    }
                else:
                    self.latest_teleop_rx_debug = None
                self.latest_teleop_pwm_enabled = bool(pwm_frame & 0x80)
                _, self.latest_teleop_rx_monotonic = _packet_receipt(received_ns)
                self._condition.notify_all()
                return

            if datagram.startswith(LIDAR_MAGIC):
                if len(datagram) != LIDAR_DATAGRAM_BYTES:
                    self.invalid_packets += 1
                    self.lidar_invalid_packets += 1
                    return
                try:
                    lidar_packet = _decode_lidar_uart_packet(
                        datagram[len(LIDAR_MAGIC) :]
                    )
                except ValueError:
                    self.invalid_packets += 1
                    self.lidar_invalid_packets += 1
                    return

                self.valid_packets += 1
                if self._accept_lidar_packet(lidar_packet, time.monotonic(), received_ns):
                    self._condition.notify_all()
                return

            if datagram.startswith(IMU_MAGIC):
                if len(datagram) != IMU_PACKET_BYTES:
                    self.invalid_packets += 1
                    self.invalid_imu_packets += 1
                    return

                _magic, seq, ready_flags, who_am_i, gx, gy, gz, ax, ay, az = (
                    IMU_PACKET.unpack(datagram)
                )
                if ready_flags & ~IMU_READY_FLAG:
                    self.invalid_packets += 1
                    self.invalid_imu_packets += 1
                    return
                ready = bool(ready_flags & IMU_READY_FLAG)
                self.valid_packets += 1
                self.imu_packets += 1
                if ready:
                    self.imu_ready_packets += 1
                else:
                    self.imu_unready_packets += 1
                self.latest_imu_seq = seq
                self.latest_imu_ready = ready
                self.latest_imu_who_am_i = who_am_i
                self.latest_imu_raw = (gx, gy, gz, ax, ay, az)
                self.latest_imu_stamp_ns, self.latest_imu_rx_monotonic = _packet_receipt(received_ns)
                self.imu_version += 1
                self._condition.notify_all()
                return

            if len(datagram) < HEADER_BYTES:
                self.invalid_packets += 1
                return

            magic, frame_id, block_index = HEADER.unpack_from(datagram)
            video_format = VIDEO_FORMATS.get(magic)
            if video_format is None:
                self.invalid_packets += 1
                return
            expected_blocks, block_payload_bytes = video_format
            if (
                len(datagram) != HEADER_BYTES + block_payload_bytes
                or block_index >= expected_blocks
            ):
                self.invalid_packets += 1
                return

            receipt = _packet_receipt(received_ns)
            self._begin_new_video_epoch_after_gap(receipt[1])

            frame = self._frames.get(frame_id)
            if frame is not None and frame.magic != magic:
                self.invalid_packets += 1
                return
            self.valid_packets += 1
            self.video_packets += 1
            if (self._latest_complete_epoch == self._video_epoch
                    and self.latest_completed_frame_id is not None
                    and not _frame_id_is_newer(frame_id, self.latest_completed_frame_id)):
                # A finished frame must not be allocated and published again.
                if frame_id == self.latest_completed_frame_id:
                    self.duplicate_packets += 1
                else:
                    self.stale_video_packets += 1
                return
            if frame is None:
                latest_source_id = self._latest_source_frame_id
                if latest_source_id is None:
                    self._latest_source_frame_id = frame_id
                elif _frame_id_is_newer(frame_id, latest_source_id):
                    previous_frame = self._frames.get(latest_source_id)
                    if previous_frame is not None:
                        self._try_recover_preview(latest_source_id, previous_frame)
                    self._latest_source_frame_id = frame_id

                if len(self._frames) >= MAX_IN_FLIGHT_FRAMES:
                    # UDP arrival order can differ from serial-number order.
                    old_frame_id = max(self._frames, key=lambda candidate: (
                        self._latest_source_frame_id - candidate) & FRAME_ID_MASK)
                    incoming_age = (self._latest_source_frame_id - frame_id) & FRAME_ID_MASK
                    oldest_age = (self._latest_source_frame_id - old_frame_id) & FRAME_ID_MASK
                    if incoming_age >= oldest_age:
                        self.stale_video_packets += 1
                        return
                    old_frame = self._frames.pop(old_frame_id)
                    if old_frame.block_count < old_frame.expected_blocks:
                        self.incomplete_frames_dropped += 1
                frame = _FrameBuffer.create(
                    magic, expected_blocks, block_payload_bytes, receipt
                )
                self._frames[frame_id] = frame

            payload = datagram[HEADER_BYTES:]
            block_added = frame.add_block(block_index, payload, receipt)
            if not block_added:
                self.duplicate_packets += 1
            elif (
                frame_id == self._latest_source_frame_id
                and self._last_valid_pixels is not None
                and self._last_valid_epoch == self._video_epoch
            ):
                start = block_index * frame.block_payload_bytes
                end = start + frame.block_payload_bytes
                self._last_valid_pixels[start:end] = payload
                self._last_valid_frame_id = frame_id

            self.latest_packet_frame_id = frame_id
            self.latest_packet_frame_blocks = frame.block_count
            self.latest_packet_frame_block_count = frame.expected_blocks

            if frame.block_count == frame.expected_blocks:
                self.complete_frames += 1
                complete_pixels = bytes(frame.pixels)
                if (
                    self.latest_completed_frame_id is None
                    or self._latest_complete_epoch != self._video_epoch
                    or _frame_id_is_newer(
                        frame_id, self.latest_completed_frame_id
                    )
                ):
                    self.latest_completed_frame_id = frame_id
                    self.latest_completed_frame = complete_pixels
                    self.latest_completed_frame_stamp_ns = frame.last_receipt[0]
                    self._latest_complete_epoch = self._video_epoch

                if frame_id == self._latest_source_frame_id:
                    self._last_valid_pixels = bytearray(complete_pixels)
                    self._last_valid_epoch = self._video_epoch
                    self._last_valid_frame_id = frame_id

                if self._can_publish_preview(frame_id, recovered=False):
                    self._publish_preview(frame_id, complete_pixels, recovered=False, receipt=frame.last_receipt)
                self.frame_version += 1
                del self._frames[frame_id]
                self._condition.notify_all()


def _image_from_rgb565(frame: bytes):
    if Image is None:
        raise RuntimeError(
            "保存或显示画面需要 Pillow；请运行：python -m pip install pillow"
        )
    # BGR;16 会将内存中的小端 RGB565 解码成标准 RGB 三通道图像。
    return Image.frombytes("RGB", (WIDTH, HEIGHT), frame, "raw", "BGR;16")
