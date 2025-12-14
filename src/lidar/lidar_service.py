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
            step = float(request_data["step"])  # turntable rotation, degrees
        except Exception as e:
            response.response = json.dumps({"error": f"Invalid request: {str(e)}"})
            return response

        self.get_logger().info(f"Waiting for one LiDAR scan at step {step}...")

        # Wait for scan
        self.scan_event.clear()
        start_time = time.time()
        timeout = 10.0

        while not self.scan_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.01)
            if time.time() - start_time > timeout:
                response.response = json.dumps({"error": "Timeout waiting for LiDAR scan"})
                return response

        # Got scan
        scan_msg = self.latest_scan
        original_ranges = np.array(scan_msg.ranges)
        original_angles = np.linspace(0, 360, num=len(original_ranges), endpoint=False)

        # Resample to exactly 360 vertical points
        target_angles = np.arange(0, 360)
        interpolated_ranges = np.interp(target_angles, original_angles, original_ranges)

        # Build DataPoint list (angle/radius only)
        points = [DataPoint(angle=float(a), radius=float(r), step=step) for a, r in zip(target_angles, interpolated_ranges)]

        # Return JSON
        response.response = json.dumps([dp.__dict__ for dp in points])
        self.get_logger().info(f"Returned {len(points)} vertical points at step={step}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LidarService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
