#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from domain.communication.service_caller import ServiceCaller

class Orchestrator(Node):
    def __init__(self):
        super().__init__('orchestrator')
        self.cli = self.create_client(Trigger, '/data/process')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/data/process service not available, waiting...')
        self.get_logger().info('Connected to /data/process')

    # def do_service_call(self):
    #     """Single reusable service call function that returns the result message."""
    #     req = Trigger.Request()
    #     future = self.cli.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #
    #     if future.result() is not None:
    #         msg = future.result().message
    #         self.get_logger().info(f'Result from data_processor: {msg}')
    #         print(f'Result from data_processor: {msg}', flush=True)
    #         return msg
    #     else:
    #         self.get_logger().error('Service call failed')
    #         return None


# def main(args=None):
#     rclpy.init(args=args)
#     node = Orchestrator()
#
#     print("=== CALL 1 ===", flush=True)
#     result_1 = node.do_service_call()
#     print(f"CALL 1 returned: {result_1}", flush=True)
#
#     print("=== CALL 2 ===", flush=True)
#     result_2 = node.do_service_call()
#     print(f"CALL 2 returned: {result_2}", flush=True)
#
#     rclpy.shutdown()
#     print("Orchestrator finished", flush=True)

def main(args=None):
    rclpy.init(args=args)

    caller = ServiceCaller(
        node_name="orchestrator",
        service_type=Trigger,
        service_name="/data/process"
    )

    print("=== CALL 1 ===")
    result1 = caller.call()
    if result1:
        print(f"CALL 1 returned message: {result1.message}")

    print("=== CALL 2 ===")
    result2 = caller.call()
    if result2:
        print(f"CALL 2 returned message: {result2.message}")

    rclpy.shutdown()
    print("Orchestrator finished", flush=True)


if __name__ == "__main__":
    main()
