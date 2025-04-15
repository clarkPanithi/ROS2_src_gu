#!/usr/bin/env python3

import rclpy
from std_srvs.srv import SetBool
#from open_gripper_service_serve..

def main():
  rclpy.init()
  gripper_service_client = rclpy.create_node('gripper_client_node')
  gripper_service = gripper_service_client.create_client(SetBool, 'open_gripper')

  while not gripper_service.wait_for_service(timeout_sec=1.0):
    print("Service not available, trying again...")

  req = SetBool.Request()
  # initially request to open the gripper
  req.data = True
  # invert gripper result initially false
  inverted = False

  ##############YOUR CODE IN BETWEEN HERE##############
  #The call_async method is used to send the request to the open_gripper service.
  # rclpy.spin_until_future_complete is used to wait for the service response.
  future = gripper_service.call_async(req)
  #Use spin_until_future_complete to wait for the ROS2 service server
  rclpy.spin_until_future_complete(gripper_service_client, future)
  try:
    result = future.result()
    if result.success:
      print(f"Gripper state inverted successfully: {result.message}")
    else:
        print(f"Gripper state inversion failed: {result.message}")
  except Exception as e:
    print(f"Service call failed: {e}")
    
    
  inverted = result.success
  print(inverted)

  gripper_service_client.destroy_node()
  rclpy.shutdown()
  ##############YOUR CODE IN BETWEEN HERE##############
  def callback(msg):

if __name__ == '__main__':
    main()