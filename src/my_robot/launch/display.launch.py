from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_file = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'turtlebot3_with_sensors.urdf'
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=[urdf_file]
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui"
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen"
        )
    ])