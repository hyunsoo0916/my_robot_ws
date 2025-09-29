from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # gazebo_ros 패키지 경로
    pkg_gazebo_ros = os.path.join(
        '/opt/ros/humble/share/gazebo_ros', 'launch', 'gazebo.launch.py'
    )

    # 내 로봇 urdf 경로
    urdf_file = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'turtlebot3_with_sensors.urdf'
    )

    return LaunchDescription([
        # Gazebo 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pkg_gazebo_ros)
        ),

        # robot_state_publisher 실행
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            arguments=[urdf_file]
        )
    ])