import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscriber = self.create_subscription(Image, 'camera_image', self.image_callback, 10)
        self.text_publisher = self.create_publisher(String, 'image_info', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().info(f'CvBridge Error: {e}')
            return
        
        processed_image = self.process_image(cv_image)
        
        processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
        self.publisher_.publish(processed_msg)
        
        text_msg = String()
        text_msg.data = "Processed image"
        self.text_publisher.publish(text_msg)

    def process_image(self, cv_image):
        # グレースケールに変換
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Cannyエッジ検出
        canny_thresh1 = 50
        canny_thresh2 = 150
        edges = cv2.Canny(gray, canny_thresh1, canny_thresh2, apertureSize=3)

        # Hough変換を使って直線を検出
        max_line_gap = 5
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=100, minLineLength=30, maxLineGap=max_line_gap)
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)  # 検出した直線を赤色で表示
                length = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
                length_cm = (length / cv_image.shape[1]) * 20  # スケールを設定
                cv2.putText(cv_image, f"Length: {length_cm:.2f} cm", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
                self.get_logger().info(f"Line length in cm: {length_cm:.2f}")

        return cv_image

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
