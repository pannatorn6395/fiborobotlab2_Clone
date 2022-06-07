#!/usr/bin/env  /usr/bin/python3

import sys

from dynamixel_sdk_custom_interfaces.srv import GetPosition
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(GetPosition, 'get_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetPosition.Request()
        self.send_request()

    def send_request(self):
        self.req.id = 1
        self.future = self.cli.call_async(self.req)
        if self.future.done():
            print("1")
            try:
                response = self.future.result()
            except Exception as e:
                self.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                print(1)
                self.get_logger().info(
                    'present_position= %d' %
                    ( response.position))


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()
    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Present_position : %d' %
                    ( response.position))
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()