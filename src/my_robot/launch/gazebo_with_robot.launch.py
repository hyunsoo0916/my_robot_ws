# gazebo_with_robot.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    pkg_my = get_package_share_directory('my_robot')

    # 1) Gazebo Classic + ROS 플러그인
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        # world 교체 시:
        # launch_arguments={'world': os.path.join(get_package_share_directory('gazebo_ros'),'worlds','empty.world')}.items()
    )

    # 2) robot_state_publisher (URDF 로드)
    urdf_file = PathJoinSubstitution([pkg_my, 'urdf', 'turtlebot3_with_sensors.urdf'])
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}],
        arguments=[urdf_file],
        output='screen'
    )

    # 3) Gazebo에 로봇 스폰 (조금 지연)
    spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'my_robot', '-file', urdf_file],
                output='screen'
            )
        ]
    )

    # 4) PointCloud2 -> LaserScan (높이 필터링 포함)
    #    입력: /velodyne_points (현재 토픽 리스트 기준)
    #    출력: /scan  (SLAM에 공급할 2D 스캔)
    pcl2laser = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'base_link',     # 기준 프레임 (TF 필수)
            'transform_tolerance': 0.02,
            'min_height': 0.1,              # ✅ 높이 필터 하한 (m)
            'max_height': 2.0,              # ✅ 높이 필터 상한 (m)
            'angle_min': -3.14159,
            'angle_max':  3.14159,
            'angle_increment': 0.00436,      # ≈ 0.25°
            'range_min': 0.10,
            'range_max': 30.0,
            'concurrency_level': 0
        }],
        remappings=[
            ('cloud_in', '/velodyne_points'),
            ('scan', '/scan')
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        rsp,
        spawner,
        pcl2laser,
    ])