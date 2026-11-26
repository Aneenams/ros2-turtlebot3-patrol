import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('frame_skip', 5)

        self.target_class = self.get_parameter('target_class').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.frame_skip = self.get_parameter('frame_skip').value

        self.get_logger().info(f'Loading YOLOv8n (watching for "{self.target_class}")...')
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()

        self.frame_count = 0
        self.detected_state = False

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.pub = self.create_publisher(Bool, '/target_detected', 10)
        self.get_logger().info('YOLOv8 Vision Node ready.')

    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, verbose=False)

        found = False
        best_conf = 0.0

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                conf = float(box.conf[0])

                if cls_name == self.target_class and conf >= self.conf_thresh:
                    found = True
                    best_conf = max(best_conf, conf)

        out = Bool()
        out.data = found
        self.pub.publish(out)

        if found and not self.detected_state:
            self.get_logger().info(
                f'TARGET DETECTED: {self.target_class} ({best_conf:.2f})!'
            )
        elif not found and self.detected_state:
            self.get_logger().info('Target cleared.')

        self.detected_state = found


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()