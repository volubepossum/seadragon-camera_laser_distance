import cv2
import numpy as np
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32

class ImageConverter(Node):
    def __init__(self):
        super().__init__('image_converter')
        
        # Declare and load parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('green_color_range.lower', [40, 50, 50]),
                ('green_color_range.upper', [90, 255, 255]),
                ('image_adjustments.contrast', 1.0),
                ('image_adjustments.brightness', 0),
                ('distance_calculation.rho', 0.083),
                ('distance_calculation.angle1', 75),
                ('distance_calculation.fov', 120),
            ]
        )
        
        # Get parameters
        self.lower_green = np.array(self.get_parameter('green_color_range.lower').value)
        self.upper_green = np.array(self.get_parameter('green_color_range.upper').value)
        self.alpha = self.get_parameter('image_adjustments.contrast').value
        self.beta = self.get_parameter('image_adjustments.brightness').value
        self.rho = self.get_parameter('distance_calculation.rho').value
        self.angle1 = self.get_parameter('distance_calculation.angle1').value
        self.FoV = self.get_parameter('distance_calculation.fov').value
        
        # Calculate derived parameters
        self.theta1 = math.radians(self.angle1)
        self.angle2 = self.FoV/2
        self.theta2 = math.radians(self.angle2)
        
        # Initialize other variables
        self.bridge = CvBridge()
        self.cv_image = None
        self.processed_frame = None
        self.processed_frame_pub = self.create_publisher(Image, 'processed_image', 10)
        self.distance = Float32()
        self.distance_pub = self.create_publisher(Float32, 'distance', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Replace with your image topic
            self.image_callback,
            10)  # QoS profile depth

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')
        # Process the image
        self.distance.data, self.processed_frame = self.process_image(self.cv_image)
        
        # Publish the processed image
        self.processed_frame_pub.publish(self.bridge.cv2_to_imgmsg(self.processed_frame, "bgr8"))
        # Publish the distance
        self.distance_pub.publish(self.distance)

    def process_image(self, frame):
        # Use the frame passed as a parameter
        
        # Adjust contrast and brightness
        adjusted_frame = cv2.convertScaleAbs(frame, alpha=self.alpha, beta=self.beta)
        
        # Convert the frame to HSV color space
        hsv_frame = cv2.cvtColor(adjusted_frame, cv2.COLOR_BGR2HSV)
        
        # Create a mask to filter only green colors
        green_mask = cv2.inRange(hsv_frame, self.lower_green, self.upper_green)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # If contours are found, find the largest one (assuming it's the laser dot)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            
            # Only consider if the contour is large enough (to avoid noise)
            if radius > 2:
                # Draw the circle on the original frame
                center = (int(x), int(y))
                cv2.circle(adjusted_frame, center, int(radius), (0, 255, 0), 2)
                cv2.circle(adjusted_frame, center, 5, (0, 0, 255), -1)  # Mark the center of the laser dot
                
                # Calculate the displacement from the image center
                frame_height, frame_width = frame.shape[:2]
                deltamax = frame_width // 2  # X-center of the image (camera center)
                delta = x - deltamax  # Horizontal displacement in pixels
                
                # Calculate the distance from the camera using trigonometry and FoV
                # Using cotangent for theta1
                cot_theta1 = 1 / math.tan(self.theta1)
                distance = self.rho / (math.tan(self.theta2 * delta / deltamax) + cot_theta1)
                
                # Display the position and calculated distance
                cv2.putText(adjusted_frame, f"Position: {int(x)}, {int(y)}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(adjusted_frame, f"Distance: {distance:.2f} meters", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                return distance, adjusted_frame
        return float('inf'), adjusted_frame


def main():
    rclpy.init()
    image_converter = ImageConverter()
    rclpy.spin(image_converter)
    image_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()