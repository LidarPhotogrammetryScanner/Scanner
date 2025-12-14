from typing import Dict, List
import json
import math

from domain.dto.LaserScan import LaserScan
from domain.dto.Point import Point
import rclpy
from rclpy.node import Node

from config import config
from scanner_pkg.srv import JsonIO

class DataProcessorService(Node):
    def __init__(self):
        super().__init__('data_service')

        # Create service using your custom JsonIO
        self.srv = self.create_service(JsonIO, '/data/process', self.handle_process)

        self.get_logger().info('data_service JsonIO service is ready!')

    def handle_process(self, request, response):
        # Parse the step from request JSON
        req_json = json.loads(request.request)
        lidar_data_dict = req_json.get("lidar_data", None)
        lidar_data = request_json_to_lidar_data(lidar_data_dict)
        if not lidar_data:
            self.get_logger().error("Missing 'lidar_data' in request")
            response.response = json.dumps(False)
            return response
        print("processing data")
        points = []
        for step, laser_scans in lidar_data.items():
            for laser_scan in laser_scans:
                point = laser_data_to_point(laser_scan, step)
                points.append(point)
        
        pcd_file = points_to_pcd(points)

        # Write to mounted volume directory accessible from host
        output_path = "/output/lidar_data.pcd"
        with open(output_path, "w") as f:
            f.write(pcd_file)
        
        self.get_logger().info(f"PCD file written to {output_path}")
            
        response.response = json.dumps(True)  # set the response
        return response

def request_json_to_lidar_data(lidar_data_dict) -> Dict[int, List[LaserScan]]:
    lidar_data = {}
    if lidar_data_dict is not None:
        for step_str, laser_scan_list in lidar_data_dict.items():
            step = int(step_str)
            
            if isinstance(laser_scan_list, str):
                laser_scan_list = json.loads(laser_scan_list)
            
            scans = [
                LaserScan(
                    angle=scan["angle"],
                    distance=scan["distance"]
                ) if isinstance(scan, dict) else scan
                for scan in laser_scan_list
            ]
            lidar_data[step] = scans
            
    return lidar_data

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
