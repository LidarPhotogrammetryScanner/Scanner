import csv
import json
import os
import random

import rclpy
from rclpy.node import Node
from scanner_pkg.srv import JsonIO   # <-- your service
from domain.dto.LaserScan import LaserScan

class LidarService(Node):
    def __init__(self):
        super().__init__('lidar_service')

        # Create service using your custom JsonIO
        self.srv = self.create_service(JsonIO, '/lidar/measure', self.handle_process)

        self.get_logger().info('Lidar JsonIO service is ready!')

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
