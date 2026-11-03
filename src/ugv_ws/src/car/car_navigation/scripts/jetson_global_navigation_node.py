#!/usr/bin/env python3
"""ROS adapter for the Jetson global route and waypoint coordinator."""
import json
from pathlib import Path
import queue
import subprocess
import threading
import time
import uuid
from collections import OrderedDict

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from ugv_bridge.srv import GlobalNavigation, NavigationCommand


class GlobalNavigationNode(Node):
    def __init__(self):
        super().__init__('jetson_global_navigation')
        script = Path(__file__).resolve().parent / 'global_planner' / 'jetson_global_planner.mjs'
        self.child = subprocess.Popen(['node', str(script)], stdin=subprocess.PIPE,
                                      stdout=subprocess.PIPE, text=True, bufsize=1)
        self.write_lock = threading.Lock()
        self.response_lock = threading.Lock()
        self.responses = {}
        self.events = queue.Queue()
        self.pending = {}
        self.request_cache = OrderedDict()
        self.bridge_safety_epochs = {}
        self.bridge_safety_lock = threading.Lock()
        self.bridge_safety_condition = threading.Condition(self.bridge_safety_lock)
        self.bridge_safety_blocked = True
        self.bridge_control_busy = True
        self.bridge_safety_id = None
        self.bridge_safety_seq = None
        self.retired_bridge_ids = set()
        self.state = {'status': 'idle', 'active': False, 'goal_index': 0, 'goal_count': 0,
                      'goals': [], 'route': []}
        self.owned_task = None
        self.last_navigation = None
        self.child_failed = False
        self.failure_cancel_sent = False
        self.reader = threading.Thread(target=self.read_child, daemon=True)
        self.reader.start()
        self.executor_client = self.create_client(NavigationCommand, '/navigation/executor/command')
        self.state_publisher = self.create_publisher(String, '/jetson/global_navigation/state', 10)
        self.command_group = MutuallyExclusiveCallbackGroup()
        self.safety_group = MutuallyExclusiveCallbackGroup()
        self.service = self.create_service(GlobalNavigation, '/jetson/global_navigation/command',
                                           self.command, callback_group=self.command_group)
        safety_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.safety_sub = self.create_subscription(String, '/ugv_bridge/safety_state',
            self.receive_safety_state, safety_qos, callback_group=self.safety_group)
        self.nav_sub = self.create_subscription(String, '/navigation/executor/state',
            lambda msg: self.receive('navigation', msg), qos_profile_sensor_data)
        self.grid_sub = self.create_subscription(String, '/navigation/executor/local_grid',
            lambda msg: self.receive('local_grid', msg), qos_profile_sensor_data)
        self.fusion_sub = self.create_subscription(String, '/fpga/round5/fusion',
            lambda msg: self.receive('fusion', msg), qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.update)

    def send(self, value):
        text = json.dumps(value, ensure_ascii=False, separators=(',', ':'), allow_nan=False) + '\n'
        with self.write_lock:
            if self.child.poll() is not None or self.child.stdin is None:
                return False
            try:
                self.child.stdin.write(text)
                self.child.stdin.flush()
                return True
            except (BrokenPipeError, OSError, ValueError):
                return False

    def read_child(self):
        try:
            for line in self.child.stdout:
                try:
                    item = json.loads(line)
                except (ValueError, TypeError):
                    continue
                if not isinstance(item, dict):
                    continue
                if item.get('kind') == 'command_result':
                    with self.response_lock:
                        waiting = self.responses.get(item.get('id'))
                        if waiting is not None:
                            waiting['result'] = item
                            waiting['event'].set()
                else:
                    self.events.put(item)
        finally:
            self.events.put({'kind': 'child_stopped'})
            with self.response_lock:
                for waiting in self.responses.values():
                    waiting['event'].set()

    def receive(self, kind, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                return
        except (ValueError, TypeError):
            return
        if kind == 'navigation':
            self.last_navigation = data
        self.send({'kind': kind, 'data': data})

    def receive_safety_state(self, message):
        try:
            value = json.loads(message.data)
            bridge_id, seq = value['bridge_id'], value['sequence']
            emergency, pending = value['emergency_stop'], value['global_cancel_pending']
            epoch, busy = value['safety_epoch'], value['control_busy']
            if (not isinstance(bridge_id, str) or not bridge_id or type(seq) is not int or seq < 0 or
                    type(epoch) is not int or epoch < 0 or type(busy) is not bool or
                    type(emergency) is not bool or type(pending) is not bool):
                return
        except (ValueError, TypeError, KeyError):
            return
        with self.bridge_safety_lock:
            if bridge_id in self.retired_bridge_ids:
                return
            previous_epoch = self.bridge_safety_epochs.get(bridge_id, -1)
            if epoch < previous_epoch:
                return
            changed_bridge = bridge_id != self.bridge_safety_id
            if not changed_bridge and self.bridge_safety_seq is not None and seq <= self.bridge_safety_seq:
                return
            if changed_bridge and self.bridge_safety_id is not None:
                self.retired_bridge_ids.add(self.bridge_safety_id)
            was_blocked = self.bridge_safety_blocked
            was_busy = self.bridge_control_busy
            self.bridge_safety_id, self.bridge_safety_seq = bridge_id, seq
            self.bridge_safety_epochs[bridge_id] = epoch
            self.bridge_control_busy = busy
            self.bridge_safety_blocked = emergency or pending
            self.bridge_safety_condition.notify_all()
            if ((self.bridge_safety_blocked and (changed_bridge or not was_blocked)) or
                    (self.bridge_control_busy and not was_busy) or
                    (not changed_bridge and epoch > previous_epoch and not was_blocked)):
                self.send({'kind':'command', 'id':uuid.uuid4().hex, 'request':{'action':'cancel'}})

    def command(self, request, response):
        try:
            value = json.loads(request.request_json)
            if not isinstance(value, dict):
                raise ValueError('任务请求必须是 JSON 对象')
        except (ValueError, TypeError) as error:
            response.message = str(error)
            return response
        bridge_id = value.get('_bridge_id')
        safety_epoch = value.get('_bridge_safety_epoch')
        from_bridge = isinstance(bridge_id, str) and bridge_id and type(safety_epoch) is int and safety_epoch >= 0
        with self.bridge_safety_condition:
            if from_bridge and bridge_id in self.retired_bridge_ids:
                if value.get('action') in ('stop', 'cancel'):
                    from_bridge = False
                else:
                    response.message = '网桥会话已退役'
                    return response
            if from_bridge:
                if value.get('action') == 'start':
                    required_seq = value.get('_bridge_control_seq')
                    if type(required_seq) is not int or required_seq < 0:
                        response.message = '网桥控制状态序号缺失'
                        return response
                    deadline = time.monotonic() + 1.0
                    while (self.bridge_safety_id is None or
                           (self.bridge_safety_id == bridge_id and
                            (self.bridge_safety_seq is None or self.bridge_safety_seq < required_seq))):
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            response.message = '等待网桥控制状态超时'
                            return response
                        self.bridge_safety_condition.wait(remaining)
                    if bridge_id != self.bridge_safety_id or bridge_id in self.retired_bridge_ids:
                        response.message = '网桥会话不是当前安全状态来源'
                        return response
                previous = self.bridge_safety_epochs.get(bridge_id, -1)
                if safety_epoch < previous:
                    response.success = value.get('action') in ('stop', 'cancel')
                    response.message = '已忽略过期的全局导航请求'
                    return response
                if value.get('action') == 'start' and safety_epoch != previous:
                    response.message = '网桥安全状态代次尚未确认'
                    return response
                if value.get('action') in ('stop', 'cancel'):
                    if self.bridge_safety_id and self.bridge_safety_id != bridge_id:
                        self.retired_bridge_ids.add(self.bridge_safety_id)
                    if self.bridge_safety_id != bridge_id:
                        self.bridge_safety_id, self.bridge_safety_seq = bridge_id, None
                    self.bridge_safety_blocked = True
                self.bridge_safety_epochs[bridge_id] = safety_epoch
            if value.get('action') == 'start' and (self.bridge_safety_blocked or self.bridge_control_busy):
                response.message = '网桥急停、全局取消或其他控制仍在执行，不能开始导航'
                return response
        # A web datagram may be retried with a different transport sequence.
        # Deduplicate its business ID without silently accepting changed goals.
        request_id, session = value.get('request_id'), value.get('session')
        cache_key = (session, request_id) if isinstance(session, str) and isinstance(request_id, str) else None
        signature_fields = ('action', 'frame', 'domain', 'generation', 'goals', 'max_speed_mm_s',
                            'max_accel_mm_s2', 'tolerance_mm', 'yaw_tolerance_mrad', 'duration_s',
                            'allow_disconnect', '_bridge_id', '_bridge_safety_epoch')
        try:
            signature = json.dumps({key: value[key] for key in signature_fields if key in value},
                                   ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False)
        except ValueError:
            response.message = '任务请求包含非有限数值'
            return response
        if cache_key is not None and cache_key in self.request_cache:
            cached_signature, success, unknown, message, state_json = self.request_cache[cache_key]
            if cached_signature != signature:
                response.message = '相同请求编号不能对应不同导航任务'
                return response
            self.request_cache.move_to_end(cache_key)
            response.success, response.unknown = success, unknown
            response.message, response.state_json = message, state_json
            return response
        token = uuid.uuid4().hex
        waiting = {'event': threading.Event(), 'result': None}
        with self.response_lock:
            self.responses[token] = waiting
        with self.bridge_safety_lock:
            stale_bridge = from_bridge and (bridge_id != self.bridge_safety_id or
                bridge_id in self.retired_bridge_ids or safety_epoch < self.bridge_safety_epochs.get(bridge_id, -1))
            if value.get('action') == 'start' and (self.bridge_safety_blocked or self.bridge_control_busy or stale_bridge):
                with self.response_lock:
                    self.responses.pop(token, None)
                response.message = '网桥急停或全局取消尚未解除，不能开始导航'
                return response
            sent = self.send({'kind': 'command', 'id': token, 'request': value})
        if sent:
            waiting['event'].wait(1.5)
        with self.response_lock:
            self.responses.pop(token, None)
        result = waiting['result']
        if result is None:
            # A delayed start cannot remain queued after the caller sees an unknown result.
            if sent and value.get('action') == 'start':
                self.send({'kind': 'command', 'id': uuid.uuid4().hex,
                           'request': {'action': 'cancel'}})
            response.success = False
            response.unknown = sent
            response.message = ('规划器未及时应答，结果未知；已请求取消'
                                if sent and value.get('action') == 'start'
                                else '规划器未及时应答，停止结果未知' if sent else '规划器不可用')
            response.state_json = json.dumps(self.state, ensure_ascii=False, separators=(',', ':'))
        else:
            response.success = result.get('success') is True
            response.unknown = result.get('unknown') is True
            response.message = str(result.get('message', ''))[:240]
            response.state_json = json.dumps(result.get('state'), ensure_ascii=False, separators=(',', ':'))
        if cache_key is not None:
            self.request_cache[cache_key] = (signature, response.success, response.unknown,
                                             response.message, response.state_json)
            if len(self.request_cache) > 128:
                self.request_cache.popitem(last=False)
        return response

    def executor_command(self, event):
        request_id, action = event.get('request_id'), event.get('action')
        task_id = event.get('task_id')
        if not isinstance(request_id, str) or action not in ('local_start', 'local_stop', 'local_cancel'):
            return
        if not self.executor_client.service_is_ready():
            self.send({'kind': 'executor_result', 'request_id': request_id, 'action': action,
                       'task_id': task_id, 'success': False, 'unknown': False})
            return
        req = NavigationCommand.Request()
        try:
            fields = event.get('fields') or {}
            req.action = {'local_start': 1, 'local_stop': 2, 'local_cancel': 3}[action]
            req.mode = 3 if fields.get('mode') == 'independent' else 2
            req.flags = (1 if fields.get('allow_disconnect') else 0) | (2 if fields.get('position_only') else 0)
            req.task_id = int(task_id)
            req.boot_id = int(event['boot_id'])
            req.odom_domain = int(event['odom_domain'])
            if action == 'local_start':
                for name in ('goal_x_m', 'goal_y_m', 'hold_yaw_rad', 'final_yaw_rad', 'duration_s'):
                    setattr(req, name, float(fields[name]))
                for name in ('max_speed_mm_s', 'max_accel_mm_s2', 'tolerance_mm', 'yaw_tolerance_mrad'):
                    setattr(req, name, int(fields[name]))
            future = self.executor_client.call_async(req)
        except (KeyError, ValueError, TypeError, OverflowError) as error:
            self.get_logger().error(f'执行器请求无效: {error}')
            self.send({'kind': 'executor_result', 'request_id': request_id, 'action': action,
                       'task_id': task_id, 'success': False, 'unknown': False})
            return
        if action == 'local_start':
            self.owned_task = (req.boot_id, req.odom_domain, req.task_id)
        self.pending[request_id] = {'future': future, 'action': action, 'task_id': task_id,
                                    'started': time.monotonic()}
        future.add_done_callback(lambda value, key=request_id: self.executor_result(key, value))

    def executor_result(self, key, future):
        item = self.pending.pop(key, None)
        if item is None:
            return
        try:
            result = future.result()
            success = result.success is True
            # The executor service exposes no structured rejection/unknown bit.
            # A failed response must not be treated as proof the motion never started.
            unknown = not success
        except Exception:
            success, unknown = False, True
        self.send({'kind': 'executor_result', 'request_id': key, 'action': item['action'],
                   'task_id': item['task_id'], 'success': success, 'unknown': unknown})

    def stop_owned_after_failure(self):
        nav = self.last_navigation
        if self.failure_cancel_sent or self.owned_task is None or not isinstance(nav, dict):
            return
        boot, domain, task = self.owned_task
        if (nav.get('boot_id'), nav.get('odom_domain'), nav.get('task_id')) != (boot, domain, task):
            return
        if nav.get('active_task') is not True or not self.executor_client.service_is_ready():
            return
        watermark = nav.get('last_task_id')
        if type(watermark) is not int or not 0 <= watermark <= 0xffffffff:
            return
        req = NavigationCommand.Request()
        req.action, req.mode, req.flags = 3, 3, 1
        req.task_id = ((watermark + 1) & 0xffffffff) or 1
        req.boot_id, req.odom_domain = boot, domain
        self.executor_client.call_async(req)
        self.failure_cancel_sent = True

    def update(self):
        self.send({'kind': 'executor_ready', 'ready': self.executor_client.service_is_ready()})
        while True:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            if event.get('kind') == 'executor_command':
                self.executor_command(event)
            elif event.get('kind') == 'state' and isinstance(event.get('state'), dict):
                self.state = event['state']
            elif event.get('kind') == 'error':
                self.get_logger().error(str(event.get('message', '全局规划异常')))
            elif event.get('kind') == 'child_stopped':
                self.child_failed = True
                self.state = {**self.state, 'status': 'failed', 'active': False,
                              'route': [], 'message': '全局规划进程已退出；执行器状态待确认'}
        now = time.monotonic()
        for key, item in list(self.pending.items()):
            if now - item['started'] > 3.0:
                self.pending.pop(key, None)
                self.send({'kind': 'executor_result', 'request_id': key, 'action': item['action'],
                           'task_id': item['task_id'], 'success': False, 'unknown': True})
        if self.child_failed:
            self.stop_owned_after_failure()
        message = String()
        message.data = json.dumps(self.state, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
        self.state_publisher.publish(message)

    def destroy_node(self):
        if self.child.poll() is None:
            try:
                self.child.stdin.close()
                self.child.wait(timeout=2)
            except (BrokenPipeError, OSError, subprocess.TimeoutExpired):
                self.child.terminate()
        return super().destroy_node()


def main():
    rclpy.init()
    node = GlobalNavigationNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
