import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import Int32MultiArray

class TagFilterNode(Node):
    def __init__(self):
        super().__init__('tag_filter_node')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',  # Topic for AprilTag detections
            self.detection_callback,
            10
        )
        self.publisher = self.create_publisher(
            Int32MultiArray,
            '/filtered_ids',  # Topic for filtered IDs
            10
        )
        self.detected_ids = set()  # Store unique IDs

    def detection_callback(self, msg):
        new_ids = []
        for detection in msg.detections:
            tag_id = detection.id[0]
            if tag_id not in self.detected_ids:
                self.detected_ids.add(tag_id)
                new_ids.append(tag_id)
                # Print the relative pose of the newly detected tag
                self.get_logger().info(
                    f"New tag detected: ID={tag_id}, Pose={detection.pose.pose.pose}"
                )

        if new_ids:
            # Publish the updated list of unique IDs
            filtered_ids_msg = Int32MultiArray()
            filtered_ids_msg.data = list(self.detected_ids)
            self.publisher.publish(filtered_ids_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TagFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()