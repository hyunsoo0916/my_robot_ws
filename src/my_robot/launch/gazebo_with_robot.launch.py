from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_file = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'turtlebot3_with_sensors.urdf'
    )

    world = os.path.join('/opt/ros/humble/share/gazebo_ros/worlds', 'empty.world')

    urdf_content = open(urdf_file).read()

    return LaunchDescription([
        # Gazebo 서버 실행 (factory 플러그인 반드시 추가!)
        ExecuteProcess(
            cmd=['gzserver', world, '--verbose',
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),

        # robot_state_publisher 실행
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{'robot_description': urdf_content}]
        ),

        # Gazebo에 로봇 스폰
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', urdf_file],
            output='screen'
        )
    ])