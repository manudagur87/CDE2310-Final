import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray

import rclpy
from rclpy.node import Node



THRESHOLD = 20
FIELD_OF_VIEW = 60

class ThermalFinderNav(Node):
    def __init__(self):
        super().__init__('thermal_finder_nav')
        self.subscriber = self.create_subscription(
            Float64MultiArray,
            '/thermal_image',
            self.thermal_image_callback,
            10
        )
        self.publisher = self.create_publisher(
            PointStamped,
            '/target_location',
            10
        )
    
    def thermal_image_callback(self, msg):
        image_data = np.frombuffer(msg.data, dtype=np.float64).reshape((8, 8))
        max_val = np.max(image_data)
        if max_val < THRESHOLD:
            self.get_logger().info('No target detected.')
            return
        
        (py,px) = np.unravel_index(np.argmax(image_data), image_data.shape)
        bearing = ((px-3.5)/7)*np.deg2rad(FIELD_OF_VIEW)

        point = PointStamped()
        #point.header = msg.header
        point.point.x = self.x_calc()
        point.point.y = np.tan(bearing)
        print(f"Target detected at: {point.point.x}, {point.point.y}")

        self.publisher.publish(point)
    
    def x_calc(self):
        return 1.0

def main(args=None):
    rclpy.init(args=args)
    thermal_finder_nav = ThermalFinderNav()
    rclpy.spin(thermal_finder_nav)
    thermal_finder_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()