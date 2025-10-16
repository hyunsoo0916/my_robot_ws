# gazebo_with_robot.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
import os

def generate_launch_description():
    pkg_my = get_package_share_directory('my_robot')

    # 1) Gazebo Classic + ROS 플러그인(init/factory) 자동 로드
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        # world 바꾸고 싶으면 아래 인자 사용:
        # launch_arguments={'world': os.path.join(get_package_share_directory('gazebo_ros'),'worlds','empty.world')}.items()
    )

    # 2) robot_state_publisher (xacro/urdf를 ament에서 로드, use_sim_time=True)
    urdf_file = PathJoinSubstitution([pkg_my, 'urdf', 'turtlebot3_with_sensors.urdf'])
    # xacro가 아니라 순수 urdf면 Command 대신 arguments로 파일을 넘겨도 됩니다.
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}],
        # urdf 내용을 파라미터로 넣고 싶다면 아래 두 줄 대신:
        # parameters=[{'use_sim_time': True, 'robot_description': Command([FindExecutable(name='xacro'), ' ', urdf_file])}],
        arguments=[urdf_file],
        output='screen'
    )

    # 3) Gazebo로 로봇 스폰 (Gazebo가 완전히 뜬 뒤 실행되도록 약간 지연)
    spawner = TimerAction(
        period=3.0,  # 서비스 준비 시간 여유
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'my_robot', '-file', urdf_file],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        rsp,
        spawner
    ])