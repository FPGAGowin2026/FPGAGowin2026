#!/usr/bin/env python3
"""仅 localhost：用当前 BX72 二进制报文驱动实际 C++ 网关，供跨仓验收。"""
import argparse
import binascii
import json
import signal
import socket
import struct
import subprocess
import sys
import threading
import time


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def telemetry(kind, seq, micros, velocity, automatic, nav_state=None):
    lengths = {3: 48, 4: 104, 5: 160, 6: 120, 7: 128, 8: 84}
    packet = bytearray(lengths[kind])
    struct.pack_into('!4sBBHII', packet, 0, b'BX72', 1, kind, len(packet), seq, micros)
    if kind == 3:
        struct.pack_into('!BBH6h', packet, 16, 3, 0x70, seq & 65535, 100, -200, 300, 0, 0, 4096)
        struct.pack_into('!I', packet, 32, seq)
    elif kind == 4:
        packet[16:24] = bytes([0xd2, 0xd2, 0xb, 0xa, ord('S'), 0, 15, 0])
        vx, vy, wz = velocity
        r = round(wz * 178 / 1000)
        wheels = (vx-vy-r, vx+vy+r, vx+vy-r, vx-vy+r)
        struct.pack_into('!19i', packet, 24, *([seq]*8), *wheels, *wheels, vx, vy, wz)
    elif kind == 5:
        struct.pack_into('!I', packet, 16, seq)
        for i in range(16):
            struct.pack_into('!HHBBH', packet, 32+i*8, (seq*1600+i*100) % 36000, 1000+i*20, 80, i, 0)
    elif kind == 6:
        struct.pack_into('!7i', packet, 16, 65536, 32768, 6554, round(velocity[0]*65.536), round(velocity[1]*65.536), round(velocity[2]*65.536), 0)
        struct.pack_into('!7i', packet, 44, *([16777]*7))
        struct.pack_into('!12I', packet, 72, 1 | 2 | 128 | (int(automatic) << 11) | 4096, seq, 0, 0, micros, 0, 0, seq, 1000, 20000, 90, 0)
    elif kind == 7:
        struct.pack_into('!I', packet, 16, 31)
        struct.pack_into('!I', packet, 20, seq)
        struct.pack_into('!I', packet, 112, 0x70)
    else:
        nav = nav_state
        struct.pack_into('!IIIIQQ', packet, 16, nav['boot_id'], nav['odom_domain'], nav['task_id'],
                         nav['map_version'], nav['now_us'], nav['expiry_us'])
        struct.pack_into('!BBBBhhhHiiIII', packet, 48, nav['status'], nav['error'], nav['mode'],
                         nav['flags'], *nav['velocity'], 0, nav['goal_x_q16'], nav['goal_y_q16'],
                         nav['candidate_count'], nav['map_age_us'], nav['pose_age_us'])
        struct.pack_into('!I', packet, 80, nav['last_task_id'])
    return packet


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bridge', required=True)
    parser.add_argument('--duration', type=float, default=60)
    args = parser.parse_args()
    ports = {key: free_port() for key in ('pc', 'ack', 'telemetry', 'video')}
    fake = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fake.bind(('127.0.0.1', 0))
    fake.settimeout(.01)
    command = [args.bridge, '--bind-pc', '0.0.0.0', '--pc-port', str(ports['pc']),
               '--bind-fpga', '127.0.0.1', '--fpga-ip', '127.0.0.1', '--fpga-port', str(fake.getsockname()[1]),
               '--ack-port', str(ports['ack']), '--telemetry-port', str(ports['telemetry']), '--video-port', str(ports['video'])]
    process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    alive = threading.Event()
    alive.set()
    velocity, automatic = [0, 0, 0], False
    nav_state = {'boot_id':0, 'odom_domain':1, 'task_id':0, 'last_task_id':0, 'map_version':0, 'now_us':0,
                 'expiry_us':0, 'status':0, 'error':0, 'mode':0, 'flags':0, 'velocity':[0, 0, 0],
                 'goal_x_q16':0, 'goal_y_q16':0, 'candidate_count':0, 'map_age_us':0, 'pose_age_us':0}
    active_task_id = 0
    task_fragments = {}
    task_token = None
    task_next_index = 0
    started = time.monotonic()

    def video():
        # Current FPGA contract: PT97, BT.709 limited-range Y0 U Y1 V.
        frame = bytes((173, 42, 173, 26)) * (800*480//2)
        sequence, stamp = 0, 0
        while alive.is_set():
            at = time.monotonic()
            stamp += 9000
            for offset in range(0, len(frame), 1400):
                if not alive.is_set():
                    return
                payload = frame[offset:offset+1400]
                marker = offset+len(payload) == len(frame)
                header = struct.pack('!BBHI4s', 128, 97 | (128 if marker else 0), sequence & 65535, stamp, b'ACG6')
                fake.sendto(header+payload, ('127.0.0.1', ports['video']))
                sequence += 1
                if sequence % 32 == 0:
                    time.sleep(.0005)
            time.sleep(max(0, .1-(time.monotonic()-at)))

    worker = threading.Thread(target=video, daemon=True)
    worker.start()
    def input_commands():
        for line in sys.stdin:
            if line.strip() == 'stop':
                alive.clear()
                return
    threading.Thread(target=input_commands, daemon=True).start()
    print(json.dumps({'ready': True, 'pc_port': ports['pc']}), flush=True)
    next_data, seq = 0, 0
    try:
        while alive.is_set() and time.monotonic()-started < args.duration and process.poll() is None:
            now = time.monotonic()
            if now >= next_data:
                seq += 1
                micros = int((now-started)*1e6) & 0xffffffff
                nav_state['now_us'] = int((now-started)*1e6)
                if nav_state['expiry_us'] and nav_state['now_us'] >= nav_state['expiry_us'] and nav_state['status'] in (1, 2, 3, 5):
                    nav_state.update(status=6, error=0, mode=0, flags=0)
                for kind in range(3, 8):
                    fake.sendto(telemetry(kind, seq, micros, velocity, automatic), ('127.0.0.1', ports['telemetry']))
                fake.sendto(telemetry(8, seq, micros, velocity, automatic, nav_state), ('127.0.0.1', ports['telemetry']))
                next_data = now+.1
            try:
                request, peer = fake.recvfrom(2048)
            except socket.timeout:
                continue
            if len(request) != 32 or request[:5] != b'BX72\x01' or request[5] not in (1, 3, 5, 7):
                continue
            if binascii.crc_hqx(request[:30], 0xffff) != int.from_bytes(request[30:], 'big'):
                continue
            if request[5] == 7:
                index = request[20]
                token = request[12:20]
                if index == 0:
                    task_fragments = {}
                    task_token = token
                    task_next_index = 0
                if token == task_token and index == task_next_index:
                    task_fragments[index] = bytes(request[21:30])
                    task_next_index += 1
                    if task_next_index == 8:
                        task = b''.join(task_fragments[i] for i in range(8))
                        action, mode, flags = task[1], task[2], task[3]
                        transaction_id = struct.unpack_from('!I', task, 4)[0]
                        if action == 1:
                            active_task_id = transaction_id
                            nav_state.update(task_id=active_task_id,
                                last_task_id=transaction_id,
                                odom_domain=struct.unpack_from('!I', task, 8)[0],
                                boot_id=struct.unpack_from('!I', task, 12)[0],
                                goal_x_q16=struct.unpack_from('!i', task, 16)[0],
                                goal_y_q16=struct.unpack_from('!i', task, 20)[0],
                                mode=mode, flags=flags,
                                expiry_us=struct.unpack_from('!Q', task, 40)[0], error=0, status=5)
                        else:
                            # 取消事务号用于接收；终态仍标记原活动任务编号。
                            nav_state.update(task_id=active_task_id, last_task_id=transaction_id,
                                status=9, error=0, mode=0, flags=0)
                        task_fragments = {}
                        task_next_index = 0
            elif request[5] == 3:
                velocity[:] = struct.unpack_from('!hhh', request, 20)
                automatic = False
                if nav_state['boot_id'] == 0:
                    nav_state['boot_id'] = int.from_bytes(request[16:20], 'big') or 1
                elif nav_state['status'] in (1, 2, 3, 5):
                    nav_state.update(status=9, error=0, mode=0, flags=0)
            else:
                if request[5] == 5:
                    automatic = bool(request[20])
                    velocity[:] = [0, 0, 0]
            ack = bytearray(request)
            ack[5] += 1
            ack[30:] = struct.pack('!H', binascii.crc_hqx(ack[:30], 0xffff))
            fake.sendto(ack, peer)
            print(json.dumps({'request': request[5], 'velocity': velocity, 'auto': automatic, 'at': now}), flush=True)
    finally:
        alive.clear()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
        worker.join(timeout=1)
        fake.close()
        if process.returncode:
            raise SystemExit(process.stderr.read().decode(errors='replace'))


if __name__ == '__main__':
    main()
