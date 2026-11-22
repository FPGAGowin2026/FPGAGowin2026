#!/usr/bin/env python3
"""Use fresh target detections to steer through the guarded FPGA velocity path."""

import json
import math
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from ugv_bridge.srv import MissionStart
from vision_msgs.msg import Detection2DArray


FRAME_TIMEOUT = 0.35
SEARCH_RESUME_DELAY = 0.8
NAV_TIMEOUT = 1.5


class VisionMission(Node):
    def __init__(self):
        super().__init__('vision_mission')
        self.image_width = self.declare_parameter('image_width', 640).value
        self.approach_speed = self.declare_parameter('approach_speed_m_s', 0.20).value
        self.search_speed = self.declare_parameter('search_angular_speed', 0.40).value
        self.turn_sign = self.declare_parameter('left_image_turn_sign', 1.0).value
        camera_info_file = self.declare_parameter('camera_info_file', '').value
        if not (self.image_width > 0 and 0 < self.approach_speed <= 0.4 and
                0 < self.search_speed <= 1.0 and self.turn_sign in (-1.0, 1.0)):
            raise ValueError('invalid vision servo parameters')
        self.image_center = self.image_width / 2
        if camera_info_file:
            try:
                with open(camera_info_file, encoding='utf-8') as stream:
                    calibration = yaml.safe_load(stream)
                self.set_image_center(calibration['image_width'], calibration['camera_matrix']['data'][2])
            except (OSError, ValueError, TypeError, KeyError, IndexError, yaml.YAMLError):
                self.get_logger().warning('Camera calibration unavailable; using the image center')
        else:
            self.create_subscription(CameraInfo, '/camera/camera_info', self.on_camera_info, qos_profile_sensor_data)
        self.detections = None
        self.detections_at = 0.0
        self.nav = None
        self.nav_at = 0.0
        self.manual_at = 0.0
        self.started_at = 0.0
        self.owned = False
        self.state, self.reason = 'idle', ''
        self.behavior = self.class_id = self.color = self.target_id = ''
        self.target = None
        self.target_at = 0.0
        self.create_subscription(Detection2DArray, '/vision/detections', self.on_detections, 10)
        self.create_subscription(String, '/robot/mission/status', self.on_nav, 10)
        self.create_subscription(PoseStamped, '/robot/mission/goal', self.on_goal, 10)
        self.create_subscription(Path, '/robot/mission/waypoints', self.on_goal, 10)
        self.create_subscription(Empty, '/robot/mission/explore', self.on_explore, 10)
        self.create_subscription(Empty, '/robot/mission/stop', self.on_stop, 10)
        self.create_subscription(String, '/fpga/command_status', self.on_command_status, 10)
        self.command_pub = self.create_publisher(String, '/robot/vision/command', 1)
        self.status_pub = self.create_publisher(String, '/vision/mission_status', 10)
        self.create_service(MissionStart, '/vision/mission/start', self.start)
        self.create_service(Trigger, '/vision/mission/stop', self.stop)
        self.create_timer(0.05, self.tick)

    def active(self):
        return self.state in ('searching', 'tracking', 'turning', 'following')

    def fresh_header(self, header):
        stamp = header.stamp.sec * 1000000000 + header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
        return bool(header.frame_id) and 0 <= age <= FRAME_TIMEOUT

    def set_image_center(self, width, center):
        if width == self.image_width and 0 < center < self.image_width:
            self.image_center = center

    def on_camera_info(self, message):
        self.set_image_center(message.width, message.k[2])

    def on_nav(self, message):
        try:
            status = json.loads(message.data)
            if isinstance(status, dict):
                self.nav, self.nav_at = status, time.monotonic()
        except ValueError:
            pass

    def on_goal(self, _message):
        if self.active():
            self.finish('stopped', '其他航点任务接管')

    def on_explore(self, _message):
        if self.active():
            self.finish('stopped', '其他探索任务接管')

    def on_stop(self, _message):
        if self.active():
            self.finish('stopped', '收到停车请求')

    def on_command_status(self, message):
        try:
            event = json.loads(message.data)
            if not isinstance(event, dict) or event.get('state') != 'manual_takeover':
                return
            stamp = event['stamp_ns']
            if type(stamp) is not int:
                return
        except (ValueError, KeyError):
            return
        age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
        if age < 0:
            return
        self.manual_at = max(self.manual_at, time.monotonic() - age)
        if self.active() and self.manual_at >= self.started_at:
            self.finish('stopped', '手动控制接管')

    def finish(self, state, reason, stop_motion=False):
        if stop_motion and self.active():
            self.send_command('stop')
        self.state, self.reason = state, reason
        self.target = None

    def send_command(self, action, **values):
        stamp_ns = self.get_clock().now().nanoseconds
        self.command_pub.publish(String(data=json.dumps({'action': action, 'stamp_ns': stamp_ns, **values},
                                                        allow_nan=False)))

    def candidates(self, array, class_id, target_id='', color=''):
        result = []
        for det in array.detections:
            box = det.bbox
            if not det.id or (target_id and det.id != target_id):
                continue
            if (not all(math.isfinite(value) for value in (box.center.position.x, box.center.position.y,
                                                          box.size_x, box.size_y)) or
                    not 0 <= box.center.position.x <= self.image_width or box.size_x <= 0 or box.size_y <= 0):
                continue
            scores = [result.hypothesis.score for result in det.results if result.hypothesis.class_id == class_id
                      and 0.5 <= result.hypothesis.score <= 1]
            if scores and (color != 'red' or any(item.hypothesis.class_id == 'color:red' for item in det.results)):
                result.append((max(scores), det))
        return sorted(result, key=lambda item: (-item[0], item[1].id))

    def on_detections(self, message):
        if not self.fresh_header(message.header):
            return
        self.detections, self.detections_at = message, time.monotonic()
        if not self.active():
            return
        matches = self.candidates(message, self.class_id, self.target_id,
                                  self.color if not self.target_id else '')
        if matches:
            self.target = matches[0][1]
            self.target_id, self.target_at = self.target.id, self.detections_at
        else:
            self.target = None

    def start(self, req, response):
        now = time.monotonic()
        requires_target = req.behavior != 'search'
        if self.active():
            response.message = '已有视觉任务，请先停止'
        elif (req.behavior not in ('follow', 'approach', 'search') or req.color not in ('', 'red') or
              not req.class_id.isascii() or not req.class_id.isdigit() or len(req.class_id) > 2 or
              not 0 <= int(req.class_id) <= 79):
            response.message = '类别、颜色或行为无效'
        elif (not self.nav or now - self.nav_at > NAV_TIMEOUT or self.nav.get('owner') == 'vision' or
              self.nav.get('mode') not in ('idle', 'arrived', 'blocked') or now - self.manual_at < 1.0):
            response.message = '底盘控制未空闲或状态不可用'
        elif not self.detections or not self.fresh_header(self.detections.header):
            response.message = '检测画面未就绪或已过期'
        else:
            matches = self.candidates(self.detections, req.class_id,
                                      req.target_id if requires_target else '', req.color)
            if requires_target and (not req.target_id or not matches):
                response.message = '所选目标已不在新鲜检测中'
                return response
            self.behavior, self.class_id, self.color = req.behavior, req.class_id, req.color
            self.target = matches[0][1] if matches else None
            self.target_id = self.target.id if self.target else ''
            self.target_at = self.detections_at if self.target else 0.0
            self.started_at, self.owned = now, False
            self.state, self.reason = ('tracking' if self.target else 'searching'), '等待底盘确认视觉控制'
            self.send_command('start')
            response.success, response.message = True, '任务已接收，执行状态见反馈'
        return response

    def stop(self, _req, response):
        self.finish('stopped', '已请求停止视觉任务', stop_motion=True)
        response.success, response.message = True, '已请求停止，等待底盘反馈'
        return response

    def tick(self):
        if self.active():
            self.advance(time.monotonic())
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state, 'reason': self.reason, 'behavior': self.behavior,
            'class_id': self.class_id, 'target_id': self.target_id}, ensure_ascii=False)))

    def advance(self, now):
        if not self.nav or now - self.nav_at > NAV_TIMEOUT:
            self.finish('stopped', '底盘状态中断，已请求停止', stop_motion=True)
            return
        if self.nav.get('mode') == 'blocked' and (self.owned or self.nav.get('owner') == 'vision'):
            self.finish('blocked', str(self.nav.get('reason', '底盘控制受阻')), stop_motion=True)
            return
        if self.nav.get('owner') != 'vision':
            if self.owned or self.nav.get('mode') in ('goal', 'explore'):
                self.finish('stopped', '底盘控制已被接管或停止')
            elif now - self.started_at > NAV_TIMEOUT:
                self.finish('blocked', '底盘未确认视觉控制', stop_motion=True)
            return
        self.owned = True
        guard = self.nav.get('vision_guard')
        if self.behavior != 'follow' and guard == 'front_stop':
            self.finish('arrived', 'FPGA 前向近障保护已触发，接近结束', stop_motion=True)
            return
        vx = wz = 0.0
        if self.detections is None or not self.fresh_header(self.detections.header):
            self.reason = '检测画面已过期，等待恢复'
        elif self.target is None or now - self.target_at > FRAME_TIMEOUT:
            if self.behavior == 'search' and now - self.target_at >= SEARCH_RESUME_DELAY:
                self.target_id = ''
                self.state, self.reason = 'searching', '原地转动搜索目标'
                wz = self.search_speed
            else:
                self.state, self.reason = 'tracking', '目标暂不可见，原地等待'
        else:
            error = (self.target.bbox.center.position.x - self.image_center) / (self.image_width / 2)
            if abs(error) > 0.05:
                wz = -self.turn_sign * max(-0.30, min(0.30, 1.2 * error))
            if self.behavior != 'follow' and abs(error) <= 0.12:
                vx = self.approach_speed
                self.state, self.reason = 'following', '正对目标，前进至 FPGA 近障保护范围'
            elif wz:
                self.state, self.reason = 'turning', '按检测框调整朝向'
            else:
                self.state, self.reason = 'tracking', '目标已居中，保持朝向'
        if vx or wz:
            if guard == 'unavailable':
                self.reason = '等待 FPGA 控制反馈'
            elif guard == 'sensor':
                self.reason = 'FPGA 雷达保护当前不可用'
            elif guard == 'obstacle':
                self.reason = 'FPGA 近障保护限制运动'
        self.send_command('servo', vx=vx, wz=wz)


def main():
    rclpy.init()
    node = VisionMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
