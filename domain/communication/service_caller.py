#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class ServiceCaller(Node):
    """
    A fully reusable ROS2 service caller.

    - Works with ANY service type
    - Request fields can be passed dynamically using kwargs
    """

    def __init__(self, node_name: str, service_type, service_name: str):
        super().__init__(node_name)

        self.service_type = service_type
        self.cli = self.create_client(service_type, service_name)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Service '{service_name}' not available, waiting...")

        self.get_logger().info(f"Connected to service: {service_name}")

    def call(self, **request_fields):
        """
        Call the service with any request fields supplied via keyword args.

        Example:
            response = caller.call(data="hello", count=5)

        Returns the service response instance.
        """

        # Create a request object
        req = self.service_type.Request()

        # Apply any provided request fields dynamically
        for key, value in request_fields.items():
            if not hasattr(req, key):
                raise AttributeError(f"Request has no field '{key}'")
            setattr(req, key, value)

        # Call async
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        # Return response or error
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error("Service call failed")
            return None
