#!/usr/bin/env python3

from gpp_action_examples_interface.srv import Print

import rclpy
from rclpy.node import Node

class PrintService(Node):

    def __init__(self):
        super().__init__('print_service')
        self.srv = self.create_service(Print, '/print_string', self.print_string_callback)

    def print_string_callback(self, request, response):
        response.response_print = request.request_print
        self.get_logger().info('Request_String: %s' % (request.request_print))
        self.get_logger().info('Response_String: %s' % (response.response_print))
        return response

def main():
    rclpy.init()

    print_srv_server = PrintService()
    rclpy.spin(print_srv_server)
    rclpy.shutdown()

if __name__== '__main__':
    main()