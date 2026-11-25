import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.detected = False

        # Subscribe to Waffle Pi's camera
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )
        # Publish True/False for the patrol node
        self.pub = self.create_publisher(Bool, '/target_detected', 10)
        self.get_logger().info('Vision node started! Watching /camera/image_raw for RED objects...')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red wraps around the HSV hue scale (0-10 and 170-180)
        mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        red_mask = mask1 | mask2

        red_pixels = cv2.countNonZero(red_mask)
        is_red = red_pixels > 1500  # Adjust threshold if needed

        out = Bool()
        out.data = is_red
        self.pub.publish(out)

        # Log only when state changes so the terminal stays clean
        if is_red != self.detected:
            self.detected = is_red
            if is_red:
                self.get_logger().info(f'RED TARGET DETECTED! ({red_pixels} px)')
            else:
                self.get_logger().info('Target cleared.')


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
