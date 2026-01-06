import csv
import json
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from threading import Event
import numpy as np

# ROS2 message type for /scan
from sensor_msgs.msg import LaserScan as RosLaserScan

# Your DTOs
from domain.dto.laser_scan import LaserScan as LaserScanDTO
from domain.dto.lidar_response import LidarResponse

# Service type
from scanner_pkg.srv import JsonIO


class LidarService(Node):
    def __init__(self):
        super().__init__('lidar_service')

        self.cb_group = ReentrantCallbackGroup()

        self.srv = self.create_service(
            JsonIO,
            '/lidar/measure',
            self.handle_process,
            callback_group=self.cb_group
        )

        self.scan_event = Event()
        self.latest_scan = None

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.scan_sub = self.create_subscription(
            RosLaserScan,
            '/scan',
            self.scan_callback,
            qos,
            callback_group=self.cb_group
        )

        self.get_logger().info('Lidar JsonIO service is ready!')

    def scan_callback(self, msg: RosLaserScan):
        self.latest_scan = msg
        self.scan_event.set()

    def handle_process(self, request, response):
        try:
            request_data = json.loads(request.request)
            step = int(request_data["step"])
        except Exception as e:
            response.response = json.dumps({"error": f"Invalid request: {str(e)}"})
            return response

        self.scan_event.clear()
        start_time = time.time()
        timeout = 10.0

        while not self.scan_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.01)
            if time.time() - start_time > timeout:
                response.response = json.dumps({"error": "Timeout waiting for LiDAR scan"})
                return response

        scan_msg: RosLaserScan = self.latest_scan

        original_ranges = np.array(scan_msg.ranges)

        # assume 360deg sweep
        original_angles = np.linspace(0, 360, num=len(original_ranges), endpoint=False)

        target_angles = np.arange(0, 360)
        interpolated_ranges = np.interp(target_angles, original_angles, original_ranges)

        laser_scans = [
            LaserScanDTO(
                angle=float(a),
                distance=float(r)
            )
            for a, r in zip(target_angles, interpolated_ranges)
        ]

        response_model = LidarResponse(laser_scans=laser_scans)
        response.response = response_model.json()

        return response


def main(args=None):
    rclpy.init(args=args)
    node = LidarService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
