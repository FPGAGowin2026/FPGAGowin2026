"""BX72网桥本机回环测试共用的纯报文编码器，不替代真实CPU。"""
import struct


def telemetry_packet(sequence, time_us):
    packet = bytearray(48)
    packet[:8] = struct.pack('!4sBBH', b'BX72', 1, 3, 48)
    packet[8:16] = struct.pack('!II', sequence, time_us)
    return bytes(packet)


def navigation_packet(sequence, now_us, state):
    packet = bytearray(84)
    struct.pack_into('!4sBBHII', packet, 0, b'BX72', 1, 8, 84,
                     sequence, now_us & 0xFFFFFFFF)
    struct.pack_into('!6IQ', packet, 16,
                     state['boot_id'], state['odom_domain'], state['task_id'],
                     state['last_task_id'], state['expiry_us'] & 0xFFFFFFFF,
                     now_us & 0xFFFFFFFF, state['session_token'])
    struct.pack_into('!BBBBhhhHiiIII', packet, 48,
                     state['status'], state['error'], state['mode'],
                     state['flags'] | 0x80,
                     state['velocity'][0], state['velocity'][1], state['velocity'][2],
                     state['planner_expansions'], state['goal_x_q16'], state['goal_y_q16'],
                     state['progress'], state['map_age_us'], state['pose_age_us'])
    struct.pack_into('!I', packet, 80, state['map_version'])
    return bytes(packet)
