import json

import rclpy
from domain.communication.service_client import ServiceClient
from domain.dto.Point import Point
from src.orchestrator.scan_data import ScanData
from std_srvs.srv import Trigger
import config.config as config
from scanner_pkg.srv import JsonIO

def main():
    orchestrator = Orchestrator()
    orchestrator.run()


class Orchestrator:

    def __init__(self):
        rclpy.init()

        self.orchestratorClient = ServiceClient(node_name="orchestrator", service_type=Trigger)
        self.servoClient = ServiceClient(node_name="servo", service_type=Trigger)
        self.photogrammetryClient = ServiceClient(node_name="photogrammetry", service_type=Trigger)
        self.lidarClient = ServiceClient(node_name="LidarClient", service_type=JsonIO)
        self._scan_data = ScanData()

    def run(self):
        self._scan_data.lidar_data.setdefault(0, []).append(Point())
        for i in range(config.STEPS_PER_ROTATION):
            self.measure(i)

        # for i in range(20):
        #     print(f"Iteration {i + 1}")
        #
        #     print("=== CALL 1 ===")
        #     result1 = self.orchestratorClient.call('/data/process')
        #     if result1:
        #         print(f"CALL 1 returned message: {result1.message}")
        #
        # print("=== CALL 2 ===")
        # result2 = self.orchestratorClient.call('/data/process')
        # if result2:
        #     print(f"CALL 2 returned message: {result2.message}")
        #
        # rclpy.shutdown()
        # print("Orchestrator finished", flush=True)



    def measure(self, step: int) -> None:
        # # self.reset_servo()
        # payload = {"step": step}
        # response = self.lidarClient.call('/lidar/measure', in_json=json.dumps(payload))
        # # self._scan_data.lidar_data[step] = response.
        # print(response)
        req = {
            "step": 123.4  # whatever
        }

        resp = self.lidarClient.call('/lidar/measure', request=json.dumps(req))

        points = json.loads(resp.response)
        # print(points)

    def reset_servo(self):
        self.servoClient.call('/servo/reset')

if __name__ == "__main__":
    main()
