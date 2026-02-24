# import json
# import random
#
# import rclpy
# from rclpy.node import Node
# from scanner_pkg.srv import JsonIO   # <-- your service
# from domain.dto.Point import DataPoint
#
# class LidarService(Node):
#     def __init__(self):
#         super().__init__('lidar_data_service')
#
#         self.srv = self.create_service(JsonIO, '/lidar/measure', self.handle_process)
#
#         self.get_logger().info('Lidar JsonIO service is ready!')
#
#     def handle_process(self, request, response):
#         # request.request is a JSON string
#         try:
#             request_data = json.loads(request.request)
#             z_value = float(request_data["step"])
#         except Exception as e:
#             response.response = json.dumps({
#                 "error": f"Invalid request: {str(e)}"
#             })
#             return response
#
#         # Generate 360 fake points
#         points = [
#             DataPoint(
#                 angle=random.uniform(-1000.0, 1000.0),
#                 radius=random.uniform(-1000.0, 1000.0)
#             )
#             for _ in range(360)
#         ]
#
#         # Convert points to JSON
#         points_json = [point.__dict__ for point in points]
#
#         # Write JSON output into the response string
#         response.response = json.dumps(points_json)
#
#         self.get_logger().info(f'Processed request: {len(points)} points at Z={z_value} cm.')
#
#         return response
#
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = LidarService()
#     rclpy.spin(node)
#     rclpy.shutdown()
#
#
# if __name__ == "__main__":
#     main()
