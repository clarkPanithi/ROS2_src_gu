import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml

class StaticCameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('static_camera_info_publisher')
        self.declare_parameter('calibration_file', '')

        calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        if not calibration_file:
            self.get_logger().error('No calibration file provided!')
            return

        try:
            with open(calibration_file, 'r') as file:
                calibration_data = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f'Failed to load calibration file: {e}')
            return

        self.camera_info = CameraInfo()
        self.camera_info.width = calibration_data['image_width']
        self.camera_info.height = calibration_data['image_height']
        self.camera_info.distortion_model = calibration_data['distortion_model']
        self.camera_info.d = calibration_data['distortion_coefficients']['data']
        self.camera_info.k = calibration_data['camera_matrix']['data']
        self.camera_info.r = calibration_data['rectification_matrix']['data']
        self.camera_info.p = calibration_data['projection_matrix']['data']

        self.publisher = self.create_publisher(CameraInfo, '/camera/camera/camera_info', 10)
        self.timer = self.create_timer(0.1, self.publish_camera_info)

    def publish_camera_info(self):
        self.camera_info.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.camera_info)

def main(args=None):
    rclpy.init(args=args)
    node = StaticCameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()