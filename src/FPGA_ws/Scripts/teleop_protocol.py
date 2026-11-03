"""BX72 遥控报文与完整帧组装；单位 mm/s、mrad/s，网络字节序。"""
import binascii
import math
import struct

FPGA_PACKET = struct.Struct('!4sBBHIQhhhI')
JPEG_HEADER = struct.Struct('!4sIHH')
MAX_JPEG = 512 * 1024


def fpga_packet(seq, token, velocity=(0, 0, 0), kind=3):
    head = FPGA_PACKET.pack(b'BX72', 1, kind, 32, seq & 0xffffffff, token, *velocity, 0)
    return head + struct.pack('!H', binascii.crc_hqx(head, 0xffff))


def fpga_auto_packet(seq, token, enabled):
    """type 5 自动模式租约：字节 20 为 0/1，其余应用负载清零。"""
    packet = bytearray(fpga_packet(seq, token, kind=5))
    packet[20] = int(bool(enabled))
    packet[30:32] = struct.pack('!H', binascii.crc_hqx(packet[:30], 0xffff))
    return bytes(packet)


def valid_ack(data, request):
    return (len(data) == 32 and len(request) == 32 and data[5] == {1: 2, 3: 4, 5: 6, 7: 8}.get(request[5]) and data[:5] == request[:5]
            and data[6:30] == request[6:30]
            and binascii.crc_hqx(data[:30], 0xffff) == int.from_bytes(data[30:], 'big'))


def navigation_task_packets(first_seq, token, *, task_id, boot_id, odom_domain,
                            goal_x_q16, goal_y_q16, yaw_q16, speed, accel,
                            tolerance, yaw_tolerance, sent_at_us, duration_us):
    """独立单点任务，目标与航向均属于 FPGA 当前 odom 坐标域。"""
    if not (task_id and boot_id and odom_domain and 0 < duration_us <= 120000000 and
            1 <= speed <= 400 and 1 <= accel <= 1000 and 1 <= tolerance <= 500 and
            1 <= yaw_tolerance <= 1000 and -205887 <= yaw_q16 <= 205887):
        raise ValueError('导航任务身份、期限或参数超出范围')
    expiry = (sent_at_us + duration_us) & 0xffffffff
    # mode=3 独立任务，flag 2 只要求位置到达。
    task = struct.pack('!BBBBIIIiiiiHHHHQQQH6x', 1, 1, 3, 2,
                       task_id, odom_domain, boot_id, goal_x_q16, goal_y_q16,
                       yaw_q16, yaw_q16, speed, accel, tolerance, yaw_tolerance,
                       expiry, 0, sent_at_us, 0)
    packets = []
    for index in range(8):
        packet = (struct.pack('!4sBBHIQ', b'BX72', 1, 7, 32,
                              (first_seq + index) & 0xffffffff, token) +
                  bytes([index]) + task[index * 9:(index + 1) * 9])
        packets.append(packet + struct.pack('!H', binascii.crc_hqx(packet, 0xffff)))
    return packets


def newer(seq, previous):
    return 0 < ((seq - previous) & 0xffffffff) < 0x80000000


def key_velocity(keys, speed=100, turn=300):
    x = int('W' in keys) - int('S' in keys)
    y = int('A' in keys) - int('D' in keys)
    scale = speed / max(1, math.hypot(x, y))
    return round(x * scale), round(y * scale), turn * (int('Q' in keys) - int('E' in keys))


class PcLease:
    """仅接受同一客户端的递增序号；断流后必须零速度重新建立租约。"""
    def __init__(self):
        self.session = None
        self.peer = None
        self.seq = None
        self.last = float('-inf')
        self.velocity = (0, 0, 0)
        self.armed = False
        self.retired = set()

    def expire(self, now):
        if now - self.last > 0.25:
            self.armed = False
            self.velocity = (0, 0, 0)

    def accept(self, msg, peer, now):
        self.expire(now)
        try:
            if msg['v'] != 1 or msg['type'] != 'control':
                return False
            session, seq = msg['session'], msg['seq']
            vel = tuple(msg[k] for k in ('vx', 'vy', 'wz'))
            if (not isinstance(session, str) or not 1 <= len(session) <= 64
                    or type(seq) is not int or not 0 <= seq <= 0xffffffff
                    or any(type(x) is not int for x in vel)
                    or abs(vel[0]) > 300 or abs(vel[1]) > 300 or abs(vel[2]) > 1000):
                return False
            if session != self.session or peer != self.peer:
                if any(vel) or (session,peer) in self.retired:
                    return False
            elif not newer(seq, self.seq):
                return False
            if not self.armed and any(vel):
                return False
            if self.session is not None and (session,peer)!=(self.session,self.peer):
                self.retired.add((self.session,self.peer))
            self.session, self.peer, self.seq = session, peer, seq
            self.last, self.velocity, self.armed = now, vel, True
            return True
        except (KeyError, TypeError, ValueError):
            return False


class RtpFrames:
    """保留已有 800×480 RGB565 格式；缺包帧绝不提交。"""
    def __init__(self, width=800, height=480):
        self.size = width * height * 2
        self.stamp = self.expected = None
        self.frame = bytearray()
        self.bad = False
        self.complete = self.dropped = self.gaps = 0

    def feed(self, data):
        if not (14 <= len(data) <= 1500 and data[0] == 0x80 and data[1] & 127 == 96
                and data[8:12] == b'ACG6' and (len(data) - 12) % 2 == 0):
            return None
        seq, ts = struct.unpack_from('!HI', data, 2)
        gap = self.expected is not None and seq != self.expected
        self.gaps += int(gap)
        if ts != self.stamp:
            self.dropped += int(self.stamp is not None)
            self.stamp, self.bad = ts, False
            self.frame.clear()
        elif gap:
            self.bad = True
        self.expected = (seq + 1) & 65535
        if not self.bad:
            self.frame.extend(data[12:])
            self.bad = len(self.frame) > self.size
        result = None
        if data[1] & 128:
            if not self.bad and len(self.frame) == self.size:
                self.complete += 1
                result = bytes(self.frame)
            else:
                self.dropped += 1
            self.stamp = None
            self.frame.clear()
        return result


def jpeg_packets(frame_id, jpeg):
    if not 0 < len(jpeg) <= MAX_JPEG:
        return
    count = (len(jpeg) + 1199) // 1200
    for index in range(count):
        yield JPEG_HEADER.pack(b'BXJV', frame_id & 0xffffffff, index, count) + jpeg[index*1200:(index+1)*1200]


class JpegFrames:
    def __init__(self):
        self.frame_id = None
        self.parts = {}
        self.count = 0
        self.complete = self.dropped = 0

    def feed(self, data):
        if not JPEG_HEADER.size < len(data) <= JPEG_HEADER.size + 1200:
            return None
        magic, frame_id, index, count = JPEG_HEADER.unpack_from(data)
        if magic != b'BXJV' or not 1 <= count <= (MAX_JPEG + 1199)//1200 or index >= count:
            return None
        if frame_id != self.frame_id:
            if self.frame_id is not None and not newer(frame_id, self.frame_id):
                return None
            self.dropped += int(bool(self.parts))
            self.frame_id, self.count = frame_id, count
            self.parts = {}
        if count != self.count:
            return None
        self.parts[index] = data[JPEG_HEADER.size:]
        if len(self.parts) == count:
            jpeg = b''.join(self.parts[k] for k in range(count))
            self.parts.clear()
            if len(jpeg) <= MAX_JPEG:
                self.complete += 1
                return jpeg
        return None
