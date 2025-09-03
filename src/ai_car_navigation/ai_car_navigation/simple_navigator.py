#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        
        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.latest_scan = None
        self.get_logger().info('Simple Navigator Started - Obstacle Avoidance Active')

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.navigate()
        
    def navigate(self):
        if self.latest_scan is None:
            return
            
        # Convert inf values to max range for processing
        ranges = np.array(self.latest_scan.ranges)
        ranges[np.isinf(ranges)] = self.latest_scan.range_max
        
        # Divide scan into sectors (left, front, right)
        num_ranges = len(ranges)
        left_sector = ranges[:num_ranges//3]
        front_sector = ranges[num_ranges//3:2*num_ranges//3]
        right_sector = ranges[2*num_ranges//3:]
        
        # Find minimum distance in each sector
        left_min = np.min(left_sector)
        front_min = np.min(front_sector)
        right_min = np.min(right_sector)
        
        # Obstacle avoidance logic
        cmd = Twist()
        obstacle_threshold = 2.0  # meters
        
        if front_min > obstacle_threshold:
            # Path is clear, go forward
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
            self.get_logger().info('Moving forward - path clear')
            
        elif left_min > right_min:
            # More space on left, turn left
            cmd.linear.x = 0.1
            cmd.angular.z = 0.5
            self.get_logger().info(f'Turning left - obstacle at {front_min:.2f}m')
            
        else:
            # More space on right, turn right
            cmd.linear.x = 0.1
            cmd.angular.z = -0.5
            self.get_logger().info(f'Turning right - obstacle at {front_min:.2f}m')
            
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    navigator = SimpleNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
