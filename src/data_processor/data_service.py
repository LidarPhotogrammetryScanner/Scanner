#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class DataProcessorService(Node):
    def __init__(self):
        super().__init__('data_processor_service')
        self.srv = self.create_service(Trigger, '/data/process', self.handle_process)
        self.get_logger().info('Data Processor Service is ready!')

    def handle_process(self, request, response):
        response.success = True
        response.message = "Processed successfully!"
        self.get_logger().info('Processing request...')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DataProcessorService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
