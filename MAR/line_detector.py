import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2

class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')
        # FIXED: Now listening to the correct camera topic!
        self.sub = self.create_subscription(Image, '/robot_camera/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Float32, '/line_error', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Camera translation failed: {e}")
            return

        h, w, _ = cv_image.shape
        # Look at the bottom half of the screen
        crop = cv_image[int(h/2):h, 0:w]
        
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        
        # Turn the geometric black track into pure white pixels
        _, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)
        
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                error = cx - (w / 2)
                
                msg_out = Float32()
                msg_out.data = float(error)
                self.pub.publish(msg_out)
                
                # Draw a red dot so you can physically see it working
                cv2.circle(crop, (cx, int(h/4)), 8, (0,0,255), -1)
        
        # DEMO WINDOWS: These will pop up and prove the camera works!
        cv2.imshow("Robot Camera", crop)
        cv2.imshow("Black Filter", thresh)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
