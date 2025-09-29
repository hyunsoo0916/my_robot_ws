import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2

class LidarCameraNode(Node):
    def __init__(self):
        super().__init__('lidar_camera_node')

        self.bridge = CvBridge()

        # ✅ 카메라 Front / Back 구독
        self.create_subscription(Image, '/camera_front/image_raw', self.front_camera_callback, 10)
        self.create_subscription(Image, '/camera_back/image_raw', self.back_camera_callback, 10)

        # ✅ LiDAR 구독
        self.create_subscription(PointCloud2, '/gazebo_ros_laser/out', self.lidar_callback, 10)

        self.get_logger().info("🚀 Lidar + Camera Node Started")

    # ================= 콜백 =================
    def front_camera_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info(f"Front Camera Frame: {cv_image.shape}")

    def back_camera_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info(f"Back Camera Frame: {cv_image.shape}")

    def lidar_callback(self, msg: PointCloud2):
        # 여기서는 PointCloud2 크기만 로그
        self.get_logger().info(f"LiDAR data received: {msg.width} x {msg.height}")


def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()