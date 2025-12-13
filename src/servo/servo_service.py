import json
import rclpy
from rclpy.node import Node
from RpiMotorLib import RpiMotorLib
import RPi.GPIO as GPIO

from config import config
from scanner_pkg.srv import JsonIO  # your custom service

# Define stepper motor pins
GPIO_pins = [17, 18, 27, 22]  # IN1, IN2, IN3, IN4

# Initialize motor
motor = RpiMotorLib.BYJMotor("MyMotor", "28BYJ-48")

def step_motor(steps, delay=0.1):
    # Move motor clockwise
    motor.motor_run(GPIO_pins, 0.001, 512, True, False, "half", 0.001)

def reset_motor():
    GPIO.cleanup()

class ServoService(Node):
    def __init__(self):
        super().__init__('servo_service')

        self.srv_step = self.create_service(JsonIO, '/servo/step', self.handle_servo_step)
        self.srv_reset = self.create_service(JsonIO, '/servo/reset', self.handle_servo_reset)
        self.get_logger().info('ServoService is ready!')

    def handle_servo_step(self, request, response):
        req_json = json.loads(request.request)
        step = req_json.get("step", None)
        if step is None:
            self.get_logger().error("Missing 'step' in request")
            response.response = json.dumps(False)
            return response

        step_motor(512, delay=0.001)
        self.get_logger().info(f"Moved stepper {step} steps")
        response.response = json.dumps(True)
        return response

    def handle_servo_reset(self, request, response):
        reset_motor()
        self.get_logger().info("Stepper reset (GPIO cleaned)")
        response.response = json.dumps(True)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServoService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
