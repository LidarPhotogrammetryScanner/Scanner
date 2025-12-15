import json
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from domain.dto.data_point import DataPoint
from domain.dto.lidar_response import LidarResponse
from scanner_pkg.srv import JsonIO
from sensor_msgs.msg import LaserScan
from threading import Event
from dataclasses import dataclass
import numpy as np

class LidarService(Node):
    def __init__(self):
        super().__init__('lidar_service')
        self.cb_group = ReentrantCallbackGroup()

        # Service
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
        # Store latest scan and notify waiting handler
        self.latest_scan = msg
        self.scan_event.set()

    def handle_process(self, request, response):
        try:
            request_data = json.loads(request.request)
            step = int(request_data["step"])  # MUST be int (JSON-safe)
        except Exception as e:
            response.response = json.dumps({"error": f"Invalid request: {str(e)}"})
            return response

        self.get_logger().info(f"Waiting for one LiDAR scan at step {step}...")
        self.scan_event.clear()
        start_time = time.time()
        timeout = 10.0

        while not self.scan_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.01)
            if time.time() - start_time > timeout:
                response.response = json.dumps({"error": "Timeout waiting for LiDAR scan"})
                return response

        scan_msg: LaserScan = self.latest_scan
        original_ranges = np.array(scan_msg.ranges)
        original_angles = np.linspace(0, 360, num=len(original_ranges), endpoint=False)

        target_angles = np.arange(0, 360)
        interpolated_ranges = np.interp(target_angles, original_angles, original_ranges)

        points = [
            DataPoint(
                angle=float(a),
                radius=float(r)
            )
            for a, r in zip(target_angles, interpolated_ranges)
        ]

        response_model = LidarResponse(points=points)
        response.response = response_model.json()

        self.get_logger().info(
            f"Returned {len(points)} points at step={step}"
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = LidarService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
