import json
import math

from config.config import OUTPUT_PATH
from domain.dto.laser_scan import LaserScan
from domain.dto.point import Point
import rclpy
from rclpy.node import Node

from config import config
from domain.dto.scan_data import ScanData
from scanner_pkg.srv import JsonIO


class DataProcessorService(Node):
    def __init__(self):
        super().__init__('data_service')

        self.srv = self.create_service(JsonIO, '/data/process', self.handle_process)

        self.get_logger().info('data_service JsonIO service is ready!')

    def handle_process(self, request, response):
        print("PROCESSING DATA")

        scan_data: ScanData = ScanData.parse_raw(request.request)

        lidar_data = scan_data.lidar_data
        points = []

        for step, laser_scans in lidar_data.items():
            for laser_scan in laser_scans:
                p = laser_data_to_point(laser_scan, step)
                points.append(p)

        pcd_file = points_to_pcd(points)

        with open(OUTPUT_PATH, "w") as f:
            f.write(pcd_file)

        self.get_logger().info(f"PCD file written to {OUTPUT_PATH}")

        response.response = json.dumps(True)
        return response


def laser_data_to_point(laser_scan: LaserScan, step: int) -> Point:
    # ---- SERVO ROTATION (around Z) ----
    servo_angle = 2.0 * math.pi * (float(step) / float(config.STEPS_PER_ROTATION))

    # ---- LIDAR SCAN ANGLE (convert DEGREES → RADIANS) ----
    beam_angle = math.radians(laser_scan.angle)

    r = laser_scan.distance

    # ---- SPHERICAL → CARTESIAN ----
    # beam angle = elevation
    horizontal = r * math.cos(beam_angle)  # radius in XY plane
    z = r * math.sin(beam_angle)

    # rotate around Z according to servo step
    x = horizontal * math.cos(servo_angle)
    y = horizontal * math.sin(servo_angle)

    return Point(x=x, y=y, z=z)


def points_to_pcd(points) -> str:
    n = len(points)

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA ascii\n"
    )

    body = "\n".join(f"{p.x} {p.y} {p.z}" for p in points)
    return header + body + "\n"


def main(args=None):
    rclpy.init(args=args)
    node = DataProcessorService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
