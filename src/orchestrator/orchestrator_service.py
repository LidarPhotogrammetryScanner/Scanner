import json
import random
import time
from wsgiref.util import request_uri

import rclpy
from domain.communication.service_client import ServiceClient
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
        self.servoClient = ServiceClient(node_name="servo", service_type=JsonIO)
        self.photogrammetryClient = ServiceClient(node_name="photogrammetry", service_type=JsonIO)
        self.lidarClient = ServiceClient(node_name="lidar", service_type=JsonIO)
        self.dataClient = ServiceClient(node_name="data", service_type=JsonIO)
        self._scan_data = ScanData()

    def run(self):

        time.sleep(5)
        # self.reset_servo()

        for i in range(config.STEPS_PER_ROTATION):
            self.measure(i)

        payload = {"step": 0}
        response = self.dataClient.call('/data/process', request=json.dumps(payload))



    def measure(self, step: int) -> None:
        # # self.reset_servo()
        # payload = {"step": step}
        # response = self.lidarClient.call('/lidar/measure', in_json=json.dumps(payload))
        # # self._scan_data.lidar_data[step] = response.
        # print(response)
        time.sleep(random.uniform(5, 5))
        self.step_servo(step)
        time.sleep(random.uniform(5, 5))
        self.lidar_scan(step)
        time.sleep(random.uniform(5, 5))
        self.photogrammetry_measure(step)

    def reset_servo(self) -> None:
        payload = {"step": 0}
        response = self.servoClient.call('/servo/reset', request=json.dumps(payload))

    def step_servo(self, step: int) -> None:
        payload = {"step": step}
        response = self.servoClient.call('/servo/step', request=json.dumps(payload))

    def photogrammetry_measure(self, step: int) -> None:
        payload = {"step": step}
        response = self.servoClient.call('/photogrammetry/measure', request=json.dumps(payload))

    def lidar_scan(self, step: int) -> None:
        payload = { "step": step }
        response = self.lidarClient.call('/lidar/measure', request=json.dumps(payload))
        self._scan_data.lidar_data[step] = response.response

if __name__ == "__main__":
    main()
