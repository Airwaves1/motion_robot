#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明启动参数
    vrpn_tracker_name_arg = DeclareLaunchArgument(
        'vrpn_tracker_name',
        default_value='MCServer',
        description='VRPN跟踪器名称'
    )
    
    sensor_id_offset_arg = DeclareLaunchArgument(
        'sensor_id_offset',
        default_value='302',
        description='传感器ID偏移量'
    )
    
    num_sensors_arg = DeclareLaunchArgument(
        'num_sensors',
        default_value='29',
        description='传感器数量'
    )
    
    enable_continuous_mode_arg = DeclareLaunchArgument(
        'enable_continuous_mode',
        default_value='true',
        description='启用持续运行模式'
    )
    
    print_frequency_arg = DeclareLaunchArgument(
        'print_frequency',
        default_value='1.0',
        description='状态打印频率（Hz）'
    )
    
    
    # VRPN测试节点
    vrpn_test_node = Node(
        package='motion_robot',
        executable='vrpn_test_node',
        name='vrpn_test_node',
        output='screen',
        parameters=[{
            'vrpn_tracker_name': LaunchConfiguration('vrpn_tracker_name'),
            'sensor_id_offset': LaunchConfiguration('sensor_id_offset'),
            'num_sensors': LaunchConfiguration('num_sensors'),
            'print_frequency': LaunchConfiguration('print_frequency'),
            'enable_continuous_mode': LaunchConfiguration('enable_continuous_mode'),
        }],
        remappings=[
            # 可以在这里添加话题重映射
        ]
    )
    
    # 启动信息
    launch_info = LogInfo(
        msg=[
            '启动MotionRobot持续运行节点\n',
            '跟踪器名称: ', LaunchConfiguration('vrpn_tracker_name'), '\n',
            '传感器ID偏移: ', LaunchConfiguration('sensor_id_offset'), '\n',
            '传感器数量: ', LaunchConfiguration('num_sensors'), '\n',
            '打印频率: ', LaunchConfiguration('print_frequency'), ' Hz\n',
            '持续运行模式: ', LaunchConfiguration('enable_continuous_mode'), '\n'
        ]
    )
    
    return LaunchDescription([
        vrpn_tracker_name_arg,
        sensor_id_offset_arg,
        num_sensors_arg,
        enable_continuous_mode_arg,
        print_frequency_arg,
        launch_info,
        vrpn_test_node,
    ])
