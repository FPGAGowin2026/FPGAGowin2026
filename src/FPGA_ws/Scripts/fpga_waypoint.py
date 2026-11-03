#!/usr/bin/env python3
"""电脑直连 BX72，向 FPGA 提交一个 odom 坐标系中的单点导航任务。"""
import argparse
import math
import secrets
import select
import socket
import struct
import sys
import time

if __package__:
    from .teleop_protocol import fpga_auto_packet, fpga_packet, navigation_task_packets, newer, valid_ack
else:
    from teleop_protocol import fpga_auto_packet, fpga_packet, navigation_task_packets, newer, valid_ack


ACTIVE = {1, 2, 3, 5, 9}
TERMINAL = {4, 6, 7, 8}


def q16(value):
    if not math.isfinite(value) or not -32768 <= value <= 32767.99998474121:
        raise ValueError('坐标超出 Q16 范围')
    scaled = value * 65536
    return math.floor(scaled + .5) if scaled >= 0 else math.ceil(scaled - .5)


def decode_navigation(data):
    if (len(data) != 84 or data[:6] != b'BX72\x01\x08' or
            struct.unpack_from('!H', data, 6)[0] != 84):
        return None
    flags = data[51]
    if not flags & 0x80 or flags & 0x08 or (flags & 0x60) == 0x60 or data[48] > 9 or data[50] > 3:
        return None
    boot, domain, task, last, expiry, now = struct.unpack_from('!6I', data, 16)
    vx, vy, wz = struct.unpack_from('!hhh', data, 52)
    return {'sequence': struct.unpack_from('!I', data, 8)[0], 'now_us': now,
            'boot_id': boot, 'odom_domain': domain, 'task_id': task, 'last_task_id': last,
            'expiry_us': expiry, 'token': struct.unpack_from('!Q', data, 40)[0],
            'status': data[48], 'error': data[49], 'mode': data[50], 'flags': flags,
            'grid': bool(flags & 0x20) and not bool(flags & 0x50),
            'rejected': bool(data[49] & 0x80), 'velocity': (vx, vy, wz)}


def decode_fusion(data):
    if (len(data) != 120 or data[:6] != b'BX72\x01\x06' or
            struct.unpack_from('!H', data, 6)[0] != 120):
        return None
    flags = struct.unpack_from('!I', data, 72)[0]
    if flags & 0xc0000001 != 0xc0000001:
        return None
    x, y, yaw = struct.unpack_from('!iii', data, 16)
    return {'sequence': struct.unpack_from('!I', data, 8)[0],
            'pose_stamp_us': struct.unpack_from('!I', data, 88)[0],
            'x': x / 1000.0, 'y': y / 1000.0, 'yaw': yaw / 1000.0}


def fusion_matches_navigation(fusion, nav):
    if fusion is None or nav is None:
        return False
    difference = (nav['now_us'] - fusion['pose_stamp_us']) & 0xffffffff
    if difference >= 0x80000000:
        difference -= 0x100000000
    return abs(difference) <= 500000


class DirectLink:
    def __init__(self, ack, telemetry, peer):
        self.ack, self.telemetry, self.peer = ack, telemetry, peer
        self.nav = None
        self.nav_sequence = None
        self.nav_at = 0.0
        self.fusion = None
        self.fusion_at = 0.0
        self.rejections = {}
        self.acks = []

    def poll(self, timeout):
        readable, _, _ = select.select([self.ack, self.telemetry], [], [], max(0, timeout))
        for sock in readable:
            batch_start = time.monotonic()
            for _ in range(128):
                try:
                    data, peer = sock.recvfrom(65535)
                except BlockingIOError:
                    break
                if peer[0] != self.peer[0]:
                    continue
                if sock is self.ack:
                    if peer == self.peer:
                        self.acks.append(data)
                        self.acks = self.acks[-32:]
                    continue
                now = time.monotonic()
                nav = decode_navigation(data)
                if nav is not None and (self.nav_sequence is None or newer(nav['sequence'], self.nav_sequence)):
                    self.nav_sequence = nav['sequence']
                    if nav['rejected']:
                        self.rejections[nav['task_id']] = nav['error'] & 0x7f
                    else:
                        self.nav, self.nav_at = nav, now
                fusion = decode_fusion(data)
                if fusion is not None and (self.fusion is None or newer(fusion['sequence'], self.fusion['sequence'])):
                    self.fusion, self.fusion_at = fusion, now
                if now - batch_start >= .004:
                    break

    def wait_for(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            self.poll(min(.02, deadline - time.monotonic()))
        return predicate()

    def send_ack(self, packet, timeout=.15, retry=.02):
        deadline = time.monotonic() + timeout
        next_send = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                self.ack.sendto(packet, self.peer)
                next_send = now + retry
            for index, received in enumerate(self.acks):
                if valid_ack(received, packet):
                    del self.acks[index]
                    return True
            self.poll(min(.01, deadline - now, max(0, next_send - now)))
        return False


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('x', type=float, help='目标 odom x，米')
    result.add_argument('y', type=float, help='目标 odom y，米')
    result.add_argument('--yaw', type=float, help='行驶保持航向，弧度；默认采用当前融合航向')
    result.add_argument('--speed', type=int, default=200, help='最大速度 mm/s，1–400')
    result.add_argument('--accel', type=int, default=200, help='最大加速度 mm/s²，1–1000')
    result.add_argument('--tolerance', type=int, default=100, help='位置容差 mm，1–500')
    result.add_argument('--yaw-tolerance', type=int, default=100, help='航向容差 mrad，1–1000')
    result.add_argument('--duration', type=float, default=60, help='FPGA 任务期限，秒，最多 120')
    result.add_argument('--local-ip', default='192.168.0.4')
    result.add_argument('--fpga-ip', default='192.168.0.2')
    result.add_argument('--control-port', type=int, default=5000)
    result.add_argument('--ack-port', type=int, default=6103)
    result.add_argument('--telemetry-port', type=int, default=6104)
    return result


def run(args):
    goal_x, goal_y = q16(args.x), q16(args.y)
    if (args.yaw is not None and (not math.isfinite(args.yaw) or abs(args.yaw) > math.pi)):
        raise ValueError('航向必须在 -π 至 π 之间')
    if not math.isfinite(args.duration) or not 0 < args.duration <= 120:
        raise ValueError('任务期限必须大于 0 且不超过 120 秒')
    duration_us = round(args.duration * 1_000_000)
    if not duration_us:
        raise ValueError('任务期限不足 1 微秒')
    if not (1 <= args.speed <= 400 and 1 <= args.accel <= 1000 and
            1 <= args.tolerance <= 500 and 1 <= args.yaw_tolerance <= 1000):
        raise ValueError('速度、加速度或容差超出协议范围')
    token = 0
    seq = secrets.randbits(32)
    started = False
    task_id = None
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as ack, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as telemetry:
        ack.bind((args.local_ip, args.ack_port))
        telemetry.bind((args.local_ip, args.telemetry_port))
        ack.setblocking(False)
        telemetry.setblocking(False)
        link = DirectLink(ack, telemetry, (args.fpga_ip, args.control_port))
        try:
            if not link.wait_for(lambda: link.nav is not None and not link.nav['rejected'], 3):
                raise RuntimeError('未收到 FPGA kind8 导航遥测')
            nav = link.nav
            if not nav['grid']:
                raise RuntimeError('设备没有报告 nav_grid_v1，不能提交 FPGA 局部任务')
            if nav['status'] in ACTIVE:
                raise RuntimeError('FPGA 已有活动任务，未接管该任务')
            # RTL 只接受比当前会话更大的 token 建立新的零速会话。
            if nav['token'] == 0xffffffffffffffff:
                raise RuntimeError('FPGA 会话 token 已用尽，无法建立新会话')
            token = max(time.time_ns(), nav['token'] + 1)
            if token > 0xffffffffffffffff:
                raise RuntimeError('本机时钟超出 FPGA 会话 token 范围')
            # 第一条零速建立新会话；它会改写控制权，因此从此必须执行退出停车。
            started = True
            seq = (seq + 1) & 0xffffffff
            if not link.send_ack(fpga_packet(seq, token)):
                raise RuntimeError('零速握手未收到匹配 ACK')
            if not link.wait_for(lambda: link.nav is not None and link.nav['token'] == token and
                                 link.nav['boot_id'] != 0 and link.nav['odom_domain'] != 0 and
                                 not link.nav['rejected'], 3):
                raise RuntimeError('未收到匹配新会话的 FPGA 身份与时钟')
            link.rejections.clear()
            if not link.wait_for(lambda: link.fusion is not None and
                                 time.monotonic() - link.fusion_at < .5 and
                                 fusion_matches_navigation(link.fusion, link.nav), 2):
                raise RuntimeError('缺少新鲜有效的 FPGA 融合位姿')
            nav = link.nav
            if not nav['flags'] & 2:
                raise RuntimeError('FPGA 报告位姿无效')
            yaw = args.yaw if args.yaw is not None else link.fusion['yaw']
            if abs(yaw) > math.pi:
                raise RuntimeError('融合航向不在协议范围')
            original_domain = (nav['boot_id'], nav['odom_domain'])
            seq = (seq + 1) & 0xffffffff
            if not link.send_ack(fpga_auto_packet(seq, token, False)):
                raise RuntimeError('手动模式切换未收到匹配 ACK')
            # 在模式 ACK 后重新读取时基；发片期间不能插入 type1/3/5。
            if not link.wait_for(lambda: link.nav is not None and link.nav['token'] == token and
                                 time.monotonic() - link.nav_at < .2, 1):
                raise RuntimeError('FPGA 会话或时钟已变化')
            nav = link.nav
            packets_boot, packets_domain = nav['boot_id'], nav['odom_domain']
            if (packets_boot, packets_domain) != original_domain:
                raise RuntimeError('模式切换期间 FPGA 坐标域发生变化')
            task_id = (nav['last_task_id'] + 1) & 0xffffffff or 1
            print(f"odom 当前 ({link.fusion['x']:.3f}, {link.fusion['y']:.3f}, {link.fusion['yaw']:.3f})，"
                  f"目标 ({args.x:.3f}, {args.y:.3f})，任务 {task_id}，"
                  f"坐标域 {packets_boot}:{packets_domain}", flush=True)
            packets = navigation_task_packets((seq + 1) & 0xffffffff, token,
                task_id=task_id, boot_id=nav['boot_id'], odom_domain=nav['odom_domain'],
                goal_x_q16=goal_x, goal_y_q16=goal_y, yaw_q16=q16(yaw), speed=args.speed,
                accel=args.accel, tolerance=args.tolerance, yaw_tolerance=args.yaw_tolerance,
                sent_at_us=nav['now_us'], duration_us=duration_us)
            transfer_deadline = time.monotonic() + .18
            for packet in packets:
                seq = struct.unpack_from('!I', packet, 8)[0]
                remaining = transfer_deadline - time.monotonic()
                if remaining <= 0 or not link.send_ack(packet, timeout=remaining):
                    raise RuntimeError('任务分片 ACK 不完整，设备执行结果未知')
                if task_id in link.rejections:
                    raise RuntimeError(f"FPGA 拒绝任务，原因码 {link.rejections[task_id]}")
            print('8 个分片已确认入队，等待 FPGA 任务状态。', flush=True)
            submitted_at = time.monotonic()
            last_report = None
            accepted = False
            missed_probes = 0
            next_probe = 0.0
            while time.monotonic() - submitted_at < args.duration + 1:
                now = time.monotonic()
                if now < next_probe:
                    link.poll(min(.02, next_probe - now))
                    continue
                next_probe = now + .075
                seq = (seq + 1) & 0xffffffff
                if link.send_ack(fpga_packet(seq, token, kind=1), timeout=.04, retry=.1):
                    missed_probes = 0
                else:
                    missed_probes += 1
                nav = link.nav
                if missed_probes >= 4 or time.monotonic() - link.nav_at > .75:
                    raise RuntimeError('FPGA 心跳或导航遥测失联，任务结果未知')
                if (not nav['grid'] or nav['boot_id'] != packets_boot or
                        nav['odom_domain'] != packets_domain or nav['token'] != token):
                    raise RuntimeError('FPGA 会话或 odom 坐标域发生变化')
                if task_id in link.rejections:
                    raise RuntimeError(f"FPGA 拒绝任务，原因码 {link.rejections[task_id]}")
                if nav['task_id'] == task_id and not nav['rejected']:
                    if nav['status'] != last_report:
                        print(f"任务 {task_id} 状态 {nav['status']}，输出 {nav['velocity']}", flush=True)
                        last_report = nav['status']
                    if nav['status'] == 4:
                        print('FPGA 遥测报告到达。', flush=True)
                        return 0
                    if nav['status'] in TERMINAL:
                        raise RuntimeError(f"FPGA 报告任务终态 {nav['status']}，error={nav['error']}")
                    if nav['status'] in ACTIVE and nav['mode'] in (2, 3):
                        accepted = True
                if not accepted and time.monotonic() - submitted_at > 2:
                    raise RuntimeError('FPGA 未报告匹配任务编号，执行结果未知')
            raise RuntimeError('等待任务终态超时')
        finally:
            if started:
                zero_ok = mode_ok = stopped = False
                try:
                    seq = (seq + 1) & 0xffffffff
                    zero_ok = link.send_ack(fpga_packet(seq, token), timeout=.25)
                except OSError:
                    pass
                try:
                    seq = (seq + 1) & 0xffffffff
                    mode_ok = link.send_ack(fpga_auto_packet(seq, token, False), timeout=.25)
                    sent_at = time.monotonic()
                    stopped = link.wait_for(lambda: task_id is not None and link.nav is not None and
                        link.nav_at >= sent_at and link.nav['token'] == token and not link.nav['rejected'] and
                        link.nav['task_id'] == task_id and link.nav['status'] in TERMINAL and
                        link.nav['velocity'] == (0, 0, 0), 1)
                except OSError:
                    pass
                print('已请求停车；' + ('对应任务的终态零输出遥测已确认。' if zero_ok and mode_ok and stopped
                                        else '停车结果未知，请查看设备。'), flush=True)


def main():
    try:
        return run(parser().parse_args())
    except KeyboardInterrupt:
        print('\n已中断。', file=sys.stderr)
        return 130
    except (OSError, RuntimeError, ValueError) as error:
        print(f'单点任务失败：{error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
