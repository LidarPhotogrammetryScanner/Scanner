#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class Orchestrator(Node):
    def __init__(self):
        super().__init__('orchestrator')
        self.cli = self.create_client(Trigger, '/data/process')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/data/process service not available, waiting...')
        self.get_logger().info('Connected to /data/process')

    def call_data_processor(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Result from data_processor: {future.result().message}')
            print(f'Result from data_processor: {future.result().message}', flush=True)
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    node.call_data_processor()
    rclpy.shutdown()
    print("Orchestrator finished", flush=True)

if __name__ == "__main__":
    main()
