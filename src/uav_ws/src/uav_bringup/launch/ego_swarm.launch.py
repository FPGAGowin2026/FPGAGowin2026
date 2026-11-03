"""Launch only the real-sensor EGO planner, trajectory server and input bridge."""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    if value('use_sim_time').lower() not in ('false', '0', 'no'):
        raise RuntimeError('EGO-Swarm upstream clocks are not unified for use_sim_time; use system time')
    planning_frame, body_frame, camera_frame = (value(n) for n in
        ('planning_frame', 'body_frame', 'camera_frame'))
    odom_topic, depth_topic = value('odom_topic'), value('depth_topic')
    if not all((planning_frame, body_frame, camera_frame, odom_topic, depth_topic)):
        raise RuntimeError('EGO sensor topics and frame names must be supplied')
    if camera_frame == body_frame or planning_frame == body_frame or camera_frame == planning_frame:
        raise RuntimeError('planning, body and optical camera frames must be distinct')

    intrinsics = {key: float(value(key)) for key in ('fx', 'fy', 'cx', 'cy')}
    depth_scale = float(value('depth_scaling_factor'))
    max_vel, max_acc = float(value('max_vel')), float(value('max_acc'))
    horizon = float(value('planning_horizon'))
    if (not all(math.isfinite(v) for v in (*intrinsics.values(), depth_scale, max_vel, max_acc, horizon)) or
            intrinsics['fx'] <= 0 or intrinsics['fy'] <= 0 or depth_scale <= 0 or
            max_vel <= 0 or max_acc <= 0 or horizon <= 0):
        raise RuntimeError('Camera intrinsics, depth scale and planning limits are invalid')
    drone_id = int(value('drone_id'))
    if drone_id < 0:
        raise RuntimeError('drone_id must be nonnegative')

    sensor_bridge = Node(
        package='uav_ego_bridge', executable='ego_sensor_bridge', output='screen',
        parameters=[{'use_sim_time': False, 'planning_frame': planning_frame,
                     'body_frame': body_frame, 'camera_frame': camera_frame,
                     'odom_topic': odom_topic, 'depth_topic': depth_topic}])

    # Mapping/optimizer parameters are upstream defaults, not camera calibration.
    planner_parameters = {
        'use_sim_time': False, 'fsm/planning_frame': planning_frame,
        'fsm/flight_type': 1, 'fsm/thresh_replan_time': 1.0,
        'fsm/thresh_no_replan_meter': 1.0, 'fsm/planning_horizon': horizon,
        'fsm/planning_horizen_time': 3.0, 'fsm/emergency_time': 1.0,
        'fsm/realworld_experiment': False, 'fsm/fail_safe': True,
        'fsm/waypoint_num': 0,
        'grid_map/resolution': 0.1, 'grid_map/map_size_x': 42.0,
        'grid_map/map_size_y': 30.0, 'grid_map/map_size_z': 5.0,
        'grid_map/local_update_range_x': 5.5, 'grid_map/local_update_range_y': 5.5,
        'grid_map/local_update_range_z': 4.5, 'grid_map/obstacles_inflation': 0.099,
        'grid_map/local_map_margin': 10, 'grid_map/ground_height': -0.01,
        'grid_map/fx': intrinsics['fx'], 'grid_map/fy': intrinsics['fy'],
        'grid_map/cx': intrinsics['cx'], 'grid_map/cy': intrinsics['cy'],
        'grid_map/use_depth_filter': True, 'grid_map/depth_filter_tolerance': 0.15,
        'grid_map/depth_filter_maxdist': 5.0, 'grid_map/depth_filter_mindist': 0.2,
        'grid_map/depth_filter_margin': 2, 'grid_map/k_depth_scaling_factor': depth_scale,
        'grid_map/skip_pixel': 2, 'grid_map/p_hit': 0.65, 'grid_map/p_miss': 0.35,
        'grid_map/p_min': 0.12, 'grid_map/p_max': 0.90, 'grid_map/p_occ': 0.80,
        'grid_map/min_ray_length': 0.1, 'grid_map/max_ray_length': 4.5,
        'grid_map/virtual_ceil_height': 2.9, 'grid_map/visualization_truncate_height': 1.8,
        'grid_map/show_occ_time': False, 'grid_map/pose_type': 1,
        'grid_map/frame_id': planning_frame, 'grid_map/odom_depth_timeout': 1.0,
        'manager/max_vel': max_vel, 'manager/max_acc': max_acc,
        'manager/max_jerk': 4.0, 'manager/control_points_distance': 0.4,
        'manager/feasibility_tolerance': 0.05, 'manager/planning_horizon': horizon,
        'manager/use_distinctive_trajs': True, 'manager/drone_id': drone_id,
        'optimization/lambda_smooth': 1.0, 'optimization/lambda_collision': 0.5,
        'optimization/lambda_feasibility': 0.1, 'optimization/lambda_fitness': 1.0,
        'optimization/dist0': 0.5, 'optimization/swarm_clearance': 0.5,
        'optimization/max_vel': max_vel, 'optimization/max_acc': max_acc,
        'bspline/limit_vel': max_vel, 'bspline/limit_acc': max_acc, 'bspline/limit_ratio': 1.1,
        'prediction/obj_num': 0, 'prediction/lambda': 1.0, 'prediction/predict_rate': 1.0,
    }
    planner = Node(
        package='ego_planner', executable='ego_planner_node', output='screen',
        parameters=[planner_parameters],
        remappings=[
            ('odom_world', '/uav_ego/odom'),
            ('grid_map/odom', '/uav_ego/odom'),
            ('grid_map/pose', '/uav_ego/camera_pose'),
            ('grid_map/depth', '/uav_ego/depth'),
            ('grid_map/cloud', '/uav_ego/cloud'),
            ('/move_base_simple/goal', '/uav_ego/goal'),
            ('planning/bspline', '/uav_ego/planning/bspline'),
            ('planning/trajectory_goal', '/uav_ego/trajectory_goal'),
            ('planning/broadcast_bspline_from_planner', '/uav_ego/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/uav_ego/broadcast_bspline'),
        ])
    trajectory_server = Node(
        package='ego_planner', executable='traj_server', output='screen',
        parameters=[{'use_sim_time': False, 'traj_server/frame_id': planning_frame,
                     'traj_server/time_forward': 1.0}],
        remappings=[('planning/bspline', '/uav_ego/planning/bspline'),
                    ('/position_cmd', '/uav_ego/position_cmd')])
    return [sensor_bridge, planner, trajectory_server]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('depth_topic'),
        DeclareLaunchArgument('camera_frame'),
        DeclareLaunchArgument('fx'), DeclareLaunchArgument('fy'),
        DeclareLaunchArgument('cx'), DeclareLaunchArgument('cy'),
        DeclareLaunchArgument('depth_scaling_factor'),
        DeclareLaunchArgument('odom_topic', default_value='/mavros/local_position/odom'),
        DeclareLaunchArgument('planning_frame', default_value='map'),
        DeclareLaunchArgument('body_frame', default_value='base_link'),
        DeclareLaunchArgument('drone_id', default_value='0'),
        DeclareLaunchArgument('max_vel', default_value='1.0'),
        DeclareLaunchArgument('max_acc', default_value='1.0'),
        DeclareLaunchArgument('planning_horizon', default_value='5.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=_nodes)])
