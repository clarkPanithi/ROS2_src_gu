import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64  # For camera control

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
        
        # Subscriber to the /joy topic
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Publisher to the /cmd_vel topic for robot movement
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Publisher to the /camera_control topic for camera movement
        self.camera_pan_publisher = self.create_publisher(
            Float64,
            '/camera_pan',
            10
        )
        self.camera_tilt_publisher = self.create_publisher(
            Float64,
            '/camera_tilt',
            10
        )
        
        # Log info
        self.get_logger().info('Teleop node has been started.')

    def joy_callback(self, joy_msg):
        """
        Callback function to process joystick input and publish Twist and camera control messages.
        """
        # Robot movement
        twist = Twist()
        twist.linear.x = joy_msg.axes[1]  # Forward/backward
        twist.angular.z = joy_msg.axes[0]  # Left/right
        self.cmd_vel_publisher.publish(twist)

        # Camera control
        camera_pan = Float64()
        camera_tilt = Float64()
        camera_pan.data = joy_msg.axes[3]  # Right joystick horizontal axis for pan
        camera_tilt.data = joy_msg.axes[4]  # Right joystick vertical axis for tilt
        self.camera_pan_publisher.publish(camera_pan)
        self.camera_tilt_publisher.publish(camera_tilt)

        # Log the output for debugging
        self.get_logger().info(
            f'Published Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}, '
            f'Camera Pan={camera_pan.data}, Camera Tilt={camera_tilt.data}'
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