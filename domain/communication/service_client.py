#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node

class ServiceClient(Node):
    """
    A fully reusable ROS2 service client.

    - Works with ANY service type
    - Automatically waits for the service to be available
    - Request fields can be passed dynamically using kwargs
    - Service name can be specified per call
    """

    def __init__(self, node_name: str, service_type):
        super().__init__(node_name)
        self.service_type = service_type
        self._service_clients = {}  # store clients per service name

    def _get_client(self, service_name: str):
        if service_name not in self._service_clients:
            client = self.create_client(self.service_type, service_name)
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Service '{service_name}' not available, waiting...")
            self.get_logger().info(f"Connected to service: {service_name}")
            self._service_clients[service_name] = client
        return self._service_clients[service_name]

    def call(self, service_name: str, **request_fields):
        time.sleep(0.2) # prevents the ROS2 executor from getting overloaded when called in rappid succession.
        """
        Call the service with any request fields supplied via keyword args.
        Service name can be passed dynamically.
        """
        client = self._get_client(service_name)
        req = self.service_type.Request()

        # Apply dynamic fields
        for key, value in request_fields.items():
            if not hasattr(req, key):
                raise AttributeError(f"Request has no field '{key}'")
            setattr(req, key, value)

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error(f"Service call to '{service_name}' failed")
            return None
