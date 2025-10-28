# ---
# 추가: Nav2 주행 런치 템플릿 2종
# 1) run_nav2_go.launch.py  — Nav2가 이미 실행 중일 때, 목표/초기포즈를 인자로 받아 nav2_go 노드만 실행

# =============================
# run_nav2_go.launch.py
# =============================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frame = LaunchConfiguration('frame')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_theta = LaunchConfiguration('start_theta')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_theta = LaunchConfiguration('goal_theta')
    no_initial_pose = LaunchConfiguration('no_initial_pose')

    return LaunchDescription([
        DeclareLaunchArgument('frame', default_value='map'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_theta', default_value='0.0'),  # deg
        DeclareLaunchArgument('goal_x', default_value='1.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_theta', default_value='0.0'),   # deg
        DeclareLaunchArgument('no_initial_pose', default_value='false'),

        Node(
            package='my_robot',
            executable='nav2_go',
            name='nav2_go',
            output='screen',
            arguments=[
                '--frame', frame,
                '--start-x', start_x,
                '--start-y', start_y,
                '--start-theta', start_theta,
                '--goal-x', goal_x,
                '--goal-y', goal_y,
                '--goal-theta', goal_theta,
                '--no-initial-pose'  # bool 문자열 지원 위해 조건 없이 넘기고, 노드 내에서 처리해도 됨
            ],
        )
    ])