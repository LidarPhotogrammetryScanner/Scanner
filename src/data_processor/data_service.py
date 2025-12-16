import json
import math
import random
import time

import rclpy
from rclpy.node import Node

from config import config
from domain.dto.scan_data import ScanData
from domain.helpers.json_utils import from_json
from scanner_pkg.srv import JsonIO   # <-- your service
from domain.dto.data_point import DataPoint
import time
import random


class DataProcessorService(Node):
    def __init__(self):
        super().__init__('data_service')

        self.srv = self.create_service(JsonIO, '/data/process', self.handle_process)

        self.get_logger().info('data_service JsonIO service is ready!')

    def handle_process(self, request, response):
        import math
        import os
        import json

        print("PROCESSING DATA")
        scan_data: ScanData = ScanData.parse_raw(request.request)

        lidar_data = scan_data.lidar_data
        points = []


        total_steps = len(lidar_data)

        for step, data_points in lidar_data.items():
            vertical_angle = 2 * math.pi * step / total_steps
            for dp in data_points:
                if dp.radius is None or dp.radius <= 0.0 or math.isnan(dp.radius):
                    continue
                horiz_angle = math.radians(dp.angle)
                r = dp.radius
                x = r * math.cos(horiz_angle) * math.cos(vertical_angle)
                y = r * math.sin(horiz_angle) * math.cos(vertical_angle)
                z = r * math.sin(vertical_angle)
                points.append((x, y, z))

        header = f"""# .PCD v0.7 - Point Cloud Data file format
    VERSION 0.7
    FIELDS x y z
    SIZE 4 4 4
    TYPE F F F
    COUNT 1 1 1
    WIDTH {len(points)}
    HEIGHT 1
    VIEWPOINT 0 0 0 1 0 0 0
    POINTS {len(points)}
    DATA ascii
    """
        points_text = "\n".join(f"{x:.6f} {y:.6f} {z:.6f}" for x, y, z in points)
        pcd_content = header + points_text

        # Write PCD to Docker-managed volume
        pcd_file_path = "/output/scan.pcd"
        with open(pcd_file_path, "w") as f:
            f.write(pcd_content)

        print(f"PCD saved to {pcd_file_path}")

        response.response = json.dumps(True)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DataProcessorService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
