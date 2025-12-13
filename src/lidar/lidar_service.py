import json
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile
from scanner_pkg.srv import JsonIO
from sensor_msgs.msg import LaserScan
from threading import Event

class LidarService(Node):
    def __init__(self):
        super().__init__('lidar_service')

        # Use a reentrant callback group so service and subscription callbacks can run simultaneously
        self.cb_group = ReentrantCallbackGroup()

        # Create service using your custom JsonIO
        self.srv = self.create_service(
            JsonIO,
            '/lidar/measure',
            self.handle_process,
            callback_group=self.cb_group
        )

        # Event to wait for scan
        self.scan_event = Event()
        self.latest_scan = None

        # QoS for LiDAR topic (reliable, sensor data)
        qos = QoSProfile(depth=10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',            # LiDAR topic
            self.scan_callback,
            qos,
            callback_group=self.cb_group
        )

        self.get_logger().info('Lidar JsonIO service is ready!')

    def scan_callback(self, msg: LaserScan):
        # Store the latest scan and set event
        self.latest_scan = msg
        self.scan_event.set()

    def handle_process(self, request, response):
        try:
            request_data = json.loads(request.request)
            z_value = float(request_data["step"])
        except Exception as e:
            response.response = json.dumps({
                "error": f"Invalid request: {str(e)}"
            })
            return response

        # Clear previous event
        self.scan_event.clear()
        self.latest_scan = None

        self.get_logger().info('Waiting for one LiDAR scan...')

        # Wait for a single /scan message (timeout optional)
        if not self.scan_event.wait(timeout=10.0):
            response.response = json.dumps({
                "error": "Timeout waiting for LiDAR scan."
            })
            return response

        scan_msg = self.latest_scan

        # Convert LaserScan to list of points
        points = []
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            points.append({
                "angle": angle,         # in radians
                "radius": r             # in meters
            })
            angle += scan_msg.angle_increment

        response.response = json.dumps(points)
        print(points)
        self.get_logger().info(f'Returned {len(points)} points at Z={z_value} cm.')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = LidarService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
