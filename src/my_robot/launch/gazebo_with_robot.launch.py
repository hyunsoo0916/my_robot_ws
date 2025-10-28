# gazebo_with_robot.launch.py (수정본)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_my = get_package_share_directory('my_robot')

    # 1) Gazebo Classic + ROS 플러그인
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        )
        # world 교체 시 launch_arguments={'world': '...'} 사용
    )

    # 2) robot_state_publisher (URDF 내용을 robot_description으로 로드)
    urdf_abs = os.path.join(pkg_my, 'urdf', 'turtlebot3_with_sensors.urdf')
    with open(urdf_abs, 'r') as f:
        urdf_content = f.read()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': urdf_content
        }]
    )

    # 3) Gazebo에 로봇 스폰 (약간 지연)  ← 여기서 파일 경로는 urdf_abs 사용!
    spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'my_robot', '-file', urdf_abs],
                output='screen'
            )
        ]
    )

    # 4) PointCloud2 -> LaserScan (높이 필터링 포함)
    pcl2laser = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'base_link',
            'transform_tolerance': 0.02,
            'min_height': 0.1,
            'max_height': 2.0,
            'angle_min': -3.14159,
            'angle_max':  3.14159,
            'angle_increment': 0.00436,
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