import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class GoalBridge(Node):
    def __init__(self):
        super().__init__('goal_bridge')
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.declare_parameter('frame', 'map')
        self.frame = self.get_parameter('frame').get_parameter_value().string_value
        self.cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, qos)
        self.get_logger().info('[goal_bridge] Ready — Click "2D Goal Pose" in RViz to start navigation.')

    def goal_cb(self, msg: PoseStamped):
        if not msg.header.frame_id or msg.header.frame_id != self.frame:
            msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()

        if not self.cli.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('[goal_bridge] Nav2 action server not available!')
            return

        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.get_logger().info(f"[goal_bridge] Sending goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) in {msg.header.frame_id}")

        fut = self.cli.send_goal_async(goal)
        fut.add_done_callback(self._accepted_cb)

    def _accepted_cb(self, fut):
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error('[goal_bridge] Goal rejected by Nav2.')
            return
        self.get_logger().info('[goal_bridge] Goal accepted — navigating...')
        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self._result_cb)

    def _result_cb(self, fut):
        status = fut.result().status
        self.get_logger().info(f'[goal_bridge] Navigation finished. status={status}')
        self.get_logger().info('[goal_bridge] Ready for next goal.')

def main():
    rclpy.init()
    node = GoalBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
