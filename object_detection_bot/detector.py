import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class ObjectFollower(Node):
    def __init__(self):
        super().__init__('object_follower')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        
        # P-Gain (Adjust this to make the robot turn faster or slower)
        self.kp = 0.005 
        self.width_center = 320 

    def listener_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(cv_image)
        cmd = Twist()

        if len(results) > 0 and len(results[0].boxes) > 0:
            # Get the horizontal center of the first box detected
            box = results[0].boxes[0].xywh[0]
            x_center = box[0].item()
            
            # Calculate Error
            error = self.width_center - x_center
            
            # Apply P-Control
            cmd.angular.z = self.kp * error
            self.get_logger().info(f'Tracking... Error: {error:.2f}')
        else:
            # Stop if no object is seen
            cmd.angular.z = 0.0

        self.publisher.publish(cmd)
        
        # Show the AI view
        cv_view = results[0].plot()
        cv2.imshow("Robot Perception", cv_view)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectFollower()
    rclpy.spin(node)
    rclpy.shutdown()