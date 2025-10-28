# =============================
# run_goal_bridge_only.launch.py — Nav2가 이미 떠 있을 때 RViz에서 클릭만으로 주행
# =============================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    frame = LaunchConfiguration('frame')
    return LaunchDescription([
        DeclareLaunchArgument('frame', default_value='map'),
        Node(
            package='my_robot', executable='goal_bridge', name='goal_bridge', output='screen',
            parameters=[{'frame': frame}]
        )
    ])
