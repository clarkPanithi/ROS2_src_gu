import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from math import sin, cos

class OdomTFNode(Node):
    def __init__(self):
        super().__init__('tf3_odom_tf')

        # Initialize robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Initialize time
        self.last_time = self.get_clock().now()

        # Subscriber to /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to broadcast transform
        self.create_timer(0.1, self.broadcast_transform)

        self.get_logger().info('tf3_odom_tf node has been started.')

    def cmd_vel_callback(self, msg):
        """
        Callback to process velocity commands and update robot pose.
        """
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        self.last_time = current_time

        # Extract velocities from the Twist message
        x_vel = msg.linear.x
        yaw_vel = msg.angular.z

        # Calculate odometry
        delta_yaw = yaw_vel * delta_time
        self.robot_yaw += delta_yaw
        self.robot_x += (x_vel * cos(self.robot_yaw)) * delta_time
        self.robot_y += (x_vel * sin(self.robot_yaw)) * delta_time

    def broadcast_transform(self):
        """
        Broadcast the transform from world to base_link.
        """
        t = TransformStamped()

        # Set the header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        # Set the translation
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.translation.z = 0.0

        # Set the rotation (quaternion from yaw)
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(0.0, 0.0, self.robot_yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


