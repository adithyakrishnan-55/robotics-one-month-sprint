import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Color

class TurtleParamNode(Node):
    def __init__(self):
        super().__init__('turtle_param_node')

        # Declare parameters
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('pen_r', 255)
        self.declare_parameter('pen_g', 0)
        self.declare_parameter('pen_b', 0)

        # Publisher
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        speed = self.get_parameter('speed').get_parameter_value().double_value

        twist = Twist()
        twist.linear.x = speed
        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleParamNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
