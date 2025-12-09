import json
import random
import time

import rclpy
from rclpy.node import Node

from config import config
from scanner_pkg.srv import JsonIO   # <-- your service
from domain.dto.Point import Point
import time
import random


class DataProcessorService(Node):
    def __init__(self):
        super().__init__('data_service')

        # Create service using your custom JsonIO
        self.srv = self.create_service(JsonIO, '/data/process', self.handle_process)

        self.get_logger().info('data_service JsonIO service is ready!')

    def handle_process(self, request, response):
        # Parse the step from request JSON
        req_json = json.loads(request.request)
        step = req_json.get("step", None)
        if step is None:
            self.get_logger().error("Missing 'step' in request")
            response.response = json.dumps(False)
            return response
        print("processing data")
        response.response = json.dumps(True)  # set the response
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DataProcessorService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
