import json
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile
from scanner_pkg.srv import JsonIO
from sensor_msgs.msg import LaserScan
from threading import Event

class LidarService(Node):
    def __init__(self):
        super().__init__('lidar_service')
        #
        # # Use a reentrant callback group so service and subscription callbacks can run simultaneously
        # self.cb_group = ReentrantCallbackGroup()
        #
        # # Create service using your custom JsonIO
        # self.srv = self.create_service(
        #     JsonIO,
        #     '/lidar/measure',
        #     self.handle_process,
        #     callback_group=self.cb_group
        # )
        #
        # # Event to wait for scan
        # self.scan_event = Event()
        # self.latest_scan = None
        #
        # # QoS for LiDAR topic (reliable, sensor data)
        # qos = QoSProfile(depth=10)
        # self.scan_sub = self.create_subscription(
        #     LaserScan,
        #     '/scan',            # LiDAR topic
        #     self.scan_callback,
        #     qos,
        #     callback_group=self.cb_group
        # )
        #
        # self.get_logger().info('Lidar JsonIO service is ready!')

    def scan_callback(self, msg: LaserScan):
        # Store the latest scan and set event
        self.latest_scan = msg
        self.scan_event.set()

    def handle_process(self, request, response):
        # try:
        #     request_data = json.loads(request.request)
        #     z_value = float(request_data["step"])
        # except Exception as e:
        #     response.response = json.dumps({
        #         "error": f"Invalid request: {str(e)}"
        #     })
        #     return response
        #
        # self.get_logger().info("Waiting for one LiDAR scan...")
        #
        # scan_msg = None
        # event = Event()
        #
        # def one_shot_callback(msg):
        #     nonlocal scan_msg
        #     scan_msg = msg
        #     event.set()
        #
        # qos = QoSProfile(depth=1)
        #
        # sub = self.create_subscription(
        #     LaserScan,
        #     '/scan',
        #     one_shot_callback,
        #     qos
        # )
        #
        # # Wait for exactly one scan
        # if not event.wait(timeout=5.0):
        #     self.destroy_subscription(sub)
        #     response.response = json.dumps({
        #         "error": "Timeout waiting for LiDAR scan"
        #     })
        #     return response
        #
        # # Immediately stop listening
        # self.destroy_subscription(sub)
        #
        # # Convert scan to JSON
        points = []
        # angle = scan_msg.angle_min
        # for r in scan_msg.ranges:
        #     points.append({
        #         "angle": angle,
        #         "radius": r
        #     })
        #     angle += scan_msg.angle_increment

        response.response = json.dumps(points)
        # self.get_logger().info(f"Returned {len(points)} points at Z={z_value}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = LidarService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
