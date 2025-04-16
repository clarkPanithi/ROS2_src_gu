# Teleop Node for TurtleBot3

This ROS 2 node (`teleop.py`) acts as an interface between a joystick (`joy_node`) and a TurtleBot3 robot. It subscribes to joystick inputs on the `/joy` topic and publishes velocity commands to the `/cmd_vel` topic.

---

## Code Explanation

### **1. Imports**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

>>rclpy: ROS 2 Python client library for creating and managing nodes.
Node: Base class for creating a ROS 2 node.
Joy: Message type for joystick input (axes and buttons).
Twist: Message type for velocity commands (linear and angular velocities).<<

2.
class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
>>TeleopNode: Custom ROS 2 node class.
super().__init__('teleop'): Initializes the node with the name teleop.<<

3.
self.joy_subscriber = self.create_subscription(
    Joy,
    '/joy',
    self.joy_callback,
    10
)
>>Subscribes to the /joy topic to receive joystick input (Joy messages).
Callback: self.joy_callback processes the joystick input.<<

4.
self.cmd_vel_publisher = self.create_publisher(
    Twist,
    '/cmd_vel',
    10
)
>>Publishes velocity commands (Twist messages) to the /cmd_vel topic.<<

5.
def joy_callback(self, joy_msg):
    twist = Twist()

    # Map joystick axes to linear and angular velocities
    twist.linear.x = joy_msg.axes[1]  # Forward/backward
    twist.angular.z = joy_msg.axes[0]  # Left/right

    # Publish the Twist message
    self.cmd_vel_publisher.publish(twist)

    # Log the output for debugging
    self.get_logger().info(f'Published Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}')
>>Processes joystick input (Joy message) and maps it to velocity commands (Twist message).
Publishes the velocity commands to the /cmd_vel topic.<<

6.
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
>>Initializes the ROS 2 client library and starts the TeleopNode.
Runs until interrupted (e.g., with Ctrl+C).<<

7. Entry Point
if __name__ == '__main__':
    main()
>>Ensures the main() function is called when the script is run directly.<<

SUMMARY
How to Run the Node
Build your workspace: colcon build
Source your workspace: source install/setup.bash
Run the node: ros2 run tb3_teleop teleop
SHORT SUMMARY TEXT
Subscribes to /joy for joystick input.
Publishes velocity commands to /cmd_vel.
Maps joystick axes to linear and angular velocities.
