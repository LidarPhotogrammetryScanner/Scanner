import json
import rclpy
from rclpy.node import Node
from RpiMotorLib import RpiMotorLib
import RPi.GPIO as GPIO

from config.config import STEPS_PER_ROTATION
from scanner_pkg.srv import JsonIO

# Define stepper motor pins
GPIO_pins = [17, 18, 27, 22]  # IN1, IN2, IN3, IN4

# Initialize motor
motor = RpiMotorLib.BYJMotor("Motor", "28BYJ-48")

def step_motor(steps, delay=0.1):
    # Move motor clockwise
    print("STEPPING STEP MOTOR")
    motor.motor_run(GPIO_pins, 0.001, 512 / STEPS_PER_ROTATION, True, False, "half", 0.001)

def reset_motor():
    GPIO.cleanup()

class StepMotorService(Node):

    def __init__(self):
        super().__init__('step_motor_service')

        self.srv_step = self.create_service(JsonIO, '/step_motor/step', self.handle_step)
        self.get_logger().info('StepMotorService is ready!')

    def handle_step(self, request, response):
        req_json = json.loads(request.request)
        step = req_json.get("step", None)
        if step is None:
            self.get_logger().error("Missing 'step' in request")
            response.response = json.dumps(False)
            return response

        step_motor(step, delay=0.001)
        self.get_logger().info(f"Moved stepper to step {step}")
        response.response = json.dumps(True)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = StepMotorService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
