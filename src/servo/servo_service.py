import json

import rclpy
from rclpy.node import Node

from config import config
from scanner_pkg.srv import JsonIO


class ServoService(Node):
    def __init__(self):
        super().__init__('servo_service')

        # Create service using your custom JsonIO
        self.srv = self.create_service(JsonIO, '/servo/step', self.handle_servo_step)
        self.srvr = self.create_service(JsonIO, '/servo/reset', self.handle_servo_reset)

        self.get_logger().info('servo JsonIO service is ready!')

    def handle_servo_step(self, request, response):
        # Parse the step from request JSON
        req_json = json.loads(request.request)
        step = req_json.get("step", None)
        if step is None:
            self.get_logger().error("Missing 'step' in request")
            response.response = json.dumps(False)
            return response
        angle = (float(step) / config.STEPS_PER_ROTATION) * 360
        print(f"Moving servo to {angle:.2f} degrees")
        response.response = json.dumps(True)  # set the response
        return response

    def handle_servo_reset(self, request, response):
        # Parse the step from request JSON
        print("resetting servo to 0 degrees")
        response.response = json.dumps(True)  # set the response
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServoService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
