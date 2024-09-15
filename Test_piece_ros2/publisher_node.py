import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
        self.text_publisher = self.create_publisher(String, 'image_info', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.cap = cv2.VideoCapture(0)  # デフォルトカメラを使用
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info('Failed to capture image')
            return
        
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)
        
        text_msg = String()
        text_msg.data = "Image captured"
        self.text_publisher.publish(text_msg)

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
