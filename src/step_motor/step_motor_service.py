import json
import time
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from config.config import *
from scanner_pkg.srv import JsonIO

# ───────── GPIO PINS ─────────

# ───────── DRV8825 CONFIG ─────────
# 1/16 microstepping
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(M0_PIN, GPIO.OUT)
GPIO.setup(M1_PIN, GPIO.OUT)
GPIO.setup(M2_PIN, GPIO.OUT)

GPIO.output(M0_PIN, GPIO.LOW)
GPIO.output(M1_PIN, GPIO.HIGH)
GPIO.output(M2_PIN, GPIO.LOW)

# ───────── MOTION CONFIG ─────────


# ───────── MOTOR DRIVER ─────────
def move_motor():
    GPIO.output(DIR_PIN, GPIO.HIGH)

    for _ in range(MOVE_MICROSTEPS):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(STEP_DELAY)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(STEP_DELAY)


def cleanup():
    GPIO.cleanup()


# ───────── ROS2 SERVICE ─────────
class StepMotorService(Node):

    def __init__(self):
        super().__init__('step_motor_service')
        self.srv_step = self.create_service(JsonIO, '/step_motor/step', self.handle_step)
        self.get_logger().info("DRV8825 NEMA17 Service Ready")


    def handle_step(self, request, response):
        try:
            req_json = json.loads(request.request)
            step = req_json.get("step", None)
        except:
            response.response = json.dumps(False)
            return response

        # Motor movement is NOT based on "step"
        move_motor()

        # "step" is only informational — exactly like before
        self.get_logger().info(f"Moved stepper to step {step}")

        response.response = json.dumps(True)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = StepMotorService()

    try:
        rclpy.spin(node)
    finally:
        cleanup()
        rclpy.shutdown()


if __name__ == "__main__":
    main()