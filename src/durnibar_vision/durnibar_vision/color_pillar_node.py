import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorPillarNode(Node):
    def __init__(self):
        super().__init__('color_pillar_node')
        
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(String, '/traffic_sign', 10)

        # HSV Color Ranges for Red and Green Pillars
        self.red_lower1 = np.array([0, 120, 70]);   self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 120, 70]); self.red_upper2 = np.array([180, 255, 255])
        self.green_lower = np.array([35, 80, 70]);  self.green_upper = np.array([85, 255, 255])

        self.get_logger().info('Fifine K420 Color Pillar Vision Node Initialized!')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Red Pillar Mask
            mask_red1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
            mask_red2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)

            # Green Pillar Mask
            mask_green = cv2.inRange(hsv, self.green_lower, self.green_upper)

            # Calculate contour areas
            area_red = cv2.countNonZero(mask_red)
            area_green = cv2.countNonZero(mask_green)

            sign_msg = String()
            if area_red > 1500 and area_red > area_green:
                sign_msg.data = "RED_PASS_RIGHT"
            elif area_green > 1500 and area_green > area_red:
                sign_msg.data = "GREEN_PASS_LEFT"
            else:
                sign_msg.data = "NONE"

            self.publisher.publish(sign_msg)

        except Exception as e:
            self.get_logger().error(f'CV Bridge Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ColorPillarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
