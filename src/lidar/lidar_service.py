import json
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from domain.dto.Point import DataPoint
from scanner_pkg.srv import JsonIO
from sensor_msgs.msg import LaserScan
from threading import Event

class LidarService(Node):
    def __init__(self):
        super().__init__('lidar_service')

        # Reentrant so callbacks can run simultaneously
        self.cb_group = ReentrantCallbackGroup()

        # Persistent service
        self.srv = self.create_service(
            JsonIO,
            '/lidar/measure',
            self.handle_process,
            callback_group=self.cb_group
        )

        # Persistent subscription to /scan
        self.scan_event = Event()
        self.latest_scan = None
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos,
            callback_group=self.cb_group
        )

        self.get_logger().info('Lidar JsonIO service is ready!')

    def scan_callback(self, msg: LaserScan):
        # Store the latest scan and notify any waiting handler
        self.latest_scan = msg
        self.scan_event.set()

    def handle_process(self, request, response):
        try:
            request_data = json.loads(request.request)
            z_value = float(request_data["step"])
        except Exception as e:
            response.response = json.dumps({"error": f"Invalid request: {str(e)}"})
            return response

        self.get_logger().info(f"Waiting for one LiDAR scan at step {z_value}...")

        # Clear the event
        self.scan_event.clear()
        start_time = time.time()
        timeout = 10.0  # seconds

        # Spin until scan arrives
        while not self.scan_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.01)
            if time.time() - start_time > timeout:
                response.response = json.dumps({"error": "Timeout waiting for LiDAR scan"})
                return response

        # Got scan
        scan_msg = self.latest_scan
        points : DataPoint = []
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            points.append({"angle": angle, "radius": r})  # .nan handling later
            angle += scan_msg.angle_increment

        response.response = json.dumps(points)
        print(points)
        self.get_logger().info(f"Returned {len(points)} points at Z={z_value}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LidarService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
