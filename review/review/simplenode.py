#!/usr/bin/env python3

import rclpy # Import the rclpy library for ROS 2 Python API
def main(): # Main function to initialize ROS2 node
    rclpy.init() # Initialize the ROS 2 Python client library
    node = rclpy.create_node('simple_node') # Create a new ROS 2 node named 'simple_node'
    rclpy.spin(node) # Keep the node alive until manual shutdown

if __name__ == '__main__': # Check if this script is being run directly
    main() # Call the main function to start the node