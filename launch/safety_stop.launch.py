"""Gazebo + ブリッジ + AI安全停止ノード をまとめて起動する。

使い方:
    ros2 launch pai_safety_stop safety_stop.launch.py
    ros2 launch pai_safety_stop safety_stop.launch.py auto_forward_speed:=0.3   # 自動で前進させる
別ターミナルでキーボード操作:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    pkg = get_package_share_directory('pai_safety_stop')
    world = LaunchConfiguration('world').perform(context)
    if not os.path.isabs(world):
        world = os.path.join(pkg, 'worlds', world)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'

    # GUIなし（-s）のときもカメラを描画できるよう --headless-rendering を付ける
    gz_args = f'-r {world}' if gui else f'-r -s --headless-rendering {world}'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    # ROS 2 <-> Gazebo のメッセージ変換（初級サンプルと同じ ros_gz_bridge）
    #   ]  : ROS -> Gazebo,   [ : Gazebo -> ROS
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    safety = Node(
        package='pai_safety_stop',
        executable='safety_stop',
        parameters=[{
            'model': LaunchConfiguration('model'),
            'stop_ratio': LaunchConfiguration('stop_ratio'),
            'auto_forward_speed': LaunchConfiguration('auto_forward_speed'),
            'show_window': LaunchConfiguration('show_window'),
        }],
        output='screen',
    )
    return [gazebo, bridge, safety]


def generate_launch_description():
    pkg = get_package_share_directory('pai_safety_stop')
    models = os.path.join(pkg, 'models')
    resource_path = os.pathsep.join(
        p for p in [models, os.environ.get('GZ_SIM_RESOURCE_PATH', '')] if p
    )
    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        DeclareLaunchArgument('world', default_value='safety_person.sdf'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('model', default_value='yolov8n.pt'),
        DeclareLaunchArgument('stop_ratio', default_value='0.65'),
        DeclareLaunchArgument('auto_forward_speed', default_value='0.0'),
        DeclareLaunchArgument('show_window', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
