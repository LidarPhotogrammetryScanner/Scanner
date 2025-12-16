import json
import math

from domain.dto.laser_scan import LaserScan
from domain.dto.point import Point
import rclpy
from rclpy.node import Node

from config import config
from domain.dto.scan_data import ScanData
from domain.helpers.json_utils import from_json
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
                point = laser_data_to_point(laser_scan, step)
                points.append(point)
        
        pcd_file = points_to_pcd(points)

        # Write to mounted volume directory accessible from host
        output_path = "/output/scan.pcd"
        with open(output_path, "w") as f:
            f.write(pcd_file)
        
        self.get_logger().info(f"PCD file written to {output_path}")
            
        response.response = json.dumps(True)  # set the response
        return response

def laser_data_to_point(laser_scan: LaserScan, step: int) -> Point:
    ''' Converts a laser scan and servo angle to a 3D point in the world coordinate system. '''

    servo_angle_deg = (float(step) / config.STEPS_PER_ROTATION) * 360
    servo_angle = math.radians(servo_angle_deg)

    horizontal_distance = laser_scan.distance * math.sin(laser_scan.angle)
    vertical_distance = laser_scan.distance * math.cos(laser_scan.angle)

    x = horizontal_distance * math.cos(servo_angle)
    y = horizontal_distance * math.sin(servo_angle)
    z = vertical_distance

    return Point(
        x=x,
        y=y,
        z=z,
    )

def points_to_pcd(points) -> str:
    ''' Converts a list of points to a PCD file. '''
    
    pcd_file = f"""PCD_FILE
VERSION .7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 1
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(points)}
DATA ascii
"""

    for point in points:
        pcd_file += f"\n{point.x} {point.y} {point.z}"

    return pcd_file

def main(args=None):
    rclpy.init(args=args)
    node = DataProcessorService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
