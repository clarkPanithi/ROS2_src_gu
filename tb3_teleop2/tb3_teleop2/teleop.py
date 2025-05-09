import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
        
        # Subscriber to a custom topic (e.g., /cmd_vel_input)
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel_input',  # Change the input topic
            self.cmd_vel_callback,
            10
        )
        
        # Publisher to the /cmd_vel topic for robot movement
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',  # Change the output topic',
            10
        )
        
        # Log info
        self.get_logger().info('Teleop node has been started.')

    def cmd_vel_callback(self, twist_msg):
        """
        Callback function to process Twist messages from /cmd_vel_input.
        """
        # Forward the Twist message to the robot
        self.cmd_vel_publisher.publish(twist_msg)

        # Log the output for debugging
        self.get_logger().info(
            f'Forwarded Twist: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()