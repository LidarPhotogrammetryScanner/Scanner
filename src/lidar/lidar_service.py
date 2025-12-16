import csv
import json
import os
import time
from domain.dto.laser_scan import LaserScan
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from domain.dto.lidar_response import LidarResponse
from scanner_pkg.srv import JsonIO
from threading import Event
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

    # def handle_process(self, request, response):
    #     try:
    #         request_data = json.loads(request.request)
    #         step = int(request_data["step"])  # MUST be int (JSON-safe)
    #     except Exception as e:
    #         response.response = json.dumps({"error": f"Invalid request: {str(e)}"})
    #         return response

    #     self.get_logger().info(f"Waiting for one LiDAR scan at step {step}...")
    #     self.scan_event.clear()
    #     start_time = time.time()
    #     timeout = 10.0

    #     while not self.scan_event.is_set():
    #         rclpy.spin_once(self, timeout_sec=0.01)
    #         if time.time() - start_time > timeout:
    #             response.response = json.dumps({"error": "Timeout waiting for LiDAR scan"})
    #             return response

    #     scan_msg: LaserScan = self.latest_scan
    #     original_ranges = np.array(scan_msg.ranges)
    #     original_angles = np.linspace(0, 360, num=len(original_ranges), endpoint=False)

    #     target_angles = np.arange(0, 360)
    #     interpolated_ranges = np.interp(target_angles, original_angles, original_ranges)

    #     laser_scans = [
    #         LaserScan(
    #             angle=float(a),
    #             distance=float(r)
    #         )
    #         for a, r in zip(target_angles, interpolated_ranges)
    #     ]

    #     response_model = LidarResponse(laser_scans=laser_scans)
    #     response.response = response_model.json()

    #     self.get_logger().info(
    #         f"Returned {len(laser_scans)} laser scans at step={step}"
    #     )

    #     return response

    def handle_process(self, request, response):
        # request.request is a JSON string
        try:
            request_data = json.loads(request.request)
            step = float(request_data["step"])
        except Exception as e:
            response.response = json.dumps({
                "error": f"Invalid request: {str(e)}"
            })
            return response

        # Read mock data from lidar_scans.csv
        # Get the directory where this script is located
        script_dir = os.path.dirname(os.path.abspath(__file__))
        csv_path = os.path.join(script_dir, 'lidar_scans.csv')
        
        laser_scans = []
        with open(csv_path, 'r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip header row
            for row in reader:
                distance, angle = row
                laser_scans.append(LaserScan(distance=float(distance), angle=float(angle)))


        # Convert points to JSON
        laser_scans_json = [laser_scan.__dict__ for laser_scan in laser_scans]

        # Write JSON output into the response string
        response.response = json.dumps(laser_scans_json)

        self.get_logger().info(f'Processed request: {len(laser_scans)} laser scans at step {step}.')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = LidarService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
