#!/usr/bin/env python3
"""
nav2_go.py — 주행 시작점(초기 포즈)과 도착점(Goal)을 받아 Nav2에 주행을 요청하는 ROS2(Python) 노드

패키지 예시: my_robot
경로 예시: my_robot/scripts/nav2_go.py  (혹은 my_robot/src/my_robot/nav2_go.py)

사용법(예):
  # x[m], y[m], theta[deg] 기준 (map frame)
  ros2 run my_robot nav2_go -- \
      --start-x 0.0 --start-y 0.0 --start-theta 0 \
      --goal-x 2.5  --goal-y -1.0 --goal-theta 90 

RViz에서 수동으로 initialpose를 찍고 싶다면 --no-initial-pose 플래그를 사용:
  ros2 run my_robot nav2_go -- --no-initial-pose --goal-x 1.0 --goal-y 0.0 --goal-theta 0

사전 조건:
- Nav2 bringup이 완료되어 있어야 합니다(map_server, amcl/slam, bt_navigator 등).
- frame: map을 사용한다고 가정합니다.
"""

import math
import argparse
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


def yaw_to_quat(yaw_rad: float):
    """Z축 회전(yaw)만 있는 평면 내 2D 자세를 쿼터니언으로 변환."""
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    # roll=pitch=0
    qx = 0.0
    qy = 0.0
    qz = sy
    qw = cy
    return qx, qy, qz, qw


class Nav2GoNode(Node):
    def __init__(self, args):
        super().__init__('nav2_go')
        self.args = args

        # QoS: /initialpose는 일반적으로 기본 QoS로 충분하지만, subscriber가 늦게 붙는 상황 대비해 keep last 10
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos
        )

        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def stamp_now(self) -> Time:
        now = self.get_clock().now().to_msg()
        return now

    def publish_initial_pose(self) -> bool:
        if self.args.no_initial_pose:
            self.get_logger().info('[nav2_go] --no-initial-pose 플래그로 초기 포즈 전송 생략')
            return True

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.args.frame
        msg.header.stamp = self.stamp_now()

        yaw_rad = math.radians(self.args.start_theta)
        qx, qy, qz, qw = yaw_to_quat(yaw_rad)

        msg.pose.pose.position.x = float(self.args.start_x)
        msg.pose.pose.position.y = float(self.args.start_y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # 간단한 공분산(초기 불확실성): x,y,theta에만 적당한 값
        cov = [0.0]*36
        cov[0] = 0.25   # x
        cov[7] = 0.25   # y
        cov[35] = 0.10  # yaw (rad^2)
        msg.pose.covariance = cov

        # Nav2/AMCL이 구독할 시간을 줘서 여러 번 퍼블리시
        for i in range(5):
            self.initialpose_pub.publish(msg)
            self.get_logger().info(f'[nav2_go] 초기 포즈 전송 {i+1}/5: ({self.args.start_x:.2f}, {self.args.start_y:.2f}, {self.args.start_theta:.1f}deg) in {self.args.frame}')
            rclpy.sleep(0.2)
        return True

    def wait_for_action_server(self, timeout_sec: float = 10.0) -> bool:
        self.get_logger().info('[nav2_go] navigate_to_pose action 서버 대기 중...')
        ready = self.nav_action_client.wait_for_server(timeout_sec=timeout_sec)
        if not ready:
            self.get_logger().error('[nav2_go] action 서버 연결 실패. Nav2 bringup 상태를 확인하세요.')
        return ready

    def send_goal(self) -> bool:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.args.frame
        goal.pose.header.stamp = self.stamp_now()

        yaw_rad = math.radians(self.args.goal_theta)
        qx, qy, qz, qw = yaw_to_quat(yaw_rad)

        goal.pose.pose.position.x = float(self.args.goal_x)
        goal.pose.pose.position.y = float(self.args.goal_y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"[nav2_go] Goal 전송: ({self.args.goal_x:.2f}, {self.args.goal_y:.2f}, {self.args.goal_theta:.1f}deg) in {self.args.frame}"
        )

        send_future = self.nav_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('[nav2_go] Goal이 거부되었습니다.')
            return False

        self.get_logger().info('[nav2_go] Goal 수락됨. 결과 대기...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()

        if result.status == 4:
            self.get_logger().warn('[nav2_go] Goal 취소됨')
            return False
        elif result.status != 0:
            self.get_logger().error(f'[nav2_go] Goal 실패 status={result.status}')
            return False

        self.get_logger().info('[nav2_go] 목표 지점 도달!')
        return True


def parse_args():
    parser = argparse.ArgumentParser(description='Send initial pose and NavigateToPose goal to Nav2')
    parser.add_argument('--frame', type=str, default='map', help='기준 좌표계(frame). 기본값: map')

    parser.add_argument('--start-x', dest='start_x', type=float, default=0.0)
    parser.add_argument('--start-y', dest='start_y', type=float, default=0.0)
    parser.add_argument('--start-theta', dest='start_theta', type=float, default=0.0, help='deg')
    parser.add_argument('--no-initial-pose', dest='no_initial_pose', action='store_true', help='초기 포즈를 퍼블리시하지 않음')

    parser.add_argument('--goal-x', dest='goal_x', type=float, required=True)
    parser.add_argument('--goal-y', dest='goal_y', type=float, required=True)
    parser.add_argument('--goal-theta', dest='goal_theta', type=float, default=0.0, help='deg')

    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Nav2GoNode(args)

    try:
        # 1) initial pose (선택)
        node.publish_initial_pose()

        # 2) action 서버 대기
        if not node.wait_for_action_server(timeout_sec=20.0):
            rclpy.shutdown()
            return

        # 3) goal 전송 및 결과 대기
        node.send_goal()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
