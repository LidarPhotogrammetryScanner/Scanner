import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time

from config.config import STEPS_PER_ROTATION
from scanner_pkg.srv import JsonIO

SERVO_PIN = 26
PWM_FREQUENCY = 50

# # Calibrated PWM duty cycles for your DS3115 continuous servo
# DUTY_STOP = 7.5
# DUTY_CW = 9.0
# DUTY_CCW = 6.0

# Slowest movement
DUTY_STOP = 7.5
DUTY_CW = 7.8   # very slow
DUTY_CCW = 7.2  # very slow

MEASURED_FULL_ROTATION_TIME = 11

# Time per step
STEP_TIME = MEASURED_FULL_ROTATION_TIME / STEPS_PER_ROTATION

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm = GPIO.PWM(SERVO_PIN, PWM_FREQUENCY)
pwm.start(0)
time.sleep(0.5)  # allow servo to initialize


class ServoMotorService(Node):
    def __init__(self):
        super().__init__("servo_motor_service")
        self.srv = self.create_service(JsonIO, "/servo_motor/step", self.handle_step)
        self.get_logger().info(
            f"ServoMotorService ready. {STEPS_PER_ROTATION} steps per rotation."
        )
        self.current_step = 0  # track pseudo-stepper position

    def handle_step(self, request, response):
        direction = getattr(request, "direction", "cw")
        duration = STEP_TIME

        duty = DUTY_CW if direction == "cw" else DUTY_CCW

        self.get_logger().info(
            f"Step {self.current_step + 1}/{STEPS_PER_ROTATION}: spinning {direction.upper()} for {duration:.3f}s"
        )
        pwm.ChangeDutyCycle(duty)
        time.sleep(duration)
        pwm.ChangeDutyCycle(DUTY_STOP)

        # update current step
        if direction == "cw":
            self.current_step = (self.current_step + 1) % STEPS_PER_ROTATION
        else:
            self.current_step = (self.current_step - 1) % STEPS_PER_ROTATION

        self.get_logger().info(
            f"Step complete. Current step position: {self.current_step}"
        )

        response.response = "true"
        return response


def cleanup_servo():
    pwm.ChangeDutyCycle(DUTY_STOP)
    # pwm.stop()
    GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = ServoMotorService()
    try:
        rclpy.spin(node)
    finally:
        # cleanup_servo()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
