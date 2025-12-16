import json
import random
import time

import rclpy
from domain.communication.service_client import ServiceClient
from domain.dto.laser_scan import LaserScan
from domain.dto.scan_data import ScanData
from std_srvs.srv import Trigger
import config.config as config
import domain.helpers.json_utils as json_utils
from scanner_pkg.srv import JsonIO

def main():
    orchestrator = Orchestrator()
    orchestrator.run()


class Orchestrator:

    def __init__(self):
        time.sleep(random.uniform(5, 5))
        rclpy.init()

        self.orchestratorClient = ServiceClient(node_name="orchestrator", service_type=Trigger)
        self.stepMotorClient = ServiceClient(node_name="step_motor", service_type=JsonIO)
        self.photogrammetryClient = ServiceClient(node_name="photogrammetry", service_type=JsonIO)
        self.lidarClient = ServiceClient(node_name="lidar", service_type=JsonIO)
        self.dataClient = ServiceClient(node_name="data", service_type=JsonIO)
        self._scan_data: ScanData = ScanData()

    def run(self):

        time.sleep(3)
        self.lidar_scan(0)
        for i in range(config.STEPS_PER_ROTATION):
            self.measure(i)
            time.sleep(5)

        self.process_data()

    def measure(self, step: int) -> None:
        self.step_step_motor(step)
        self.lidar_scan(step)

    def step_step_motor(self, step: int) -> None:
        payload = {"step": step}
        self.stepMotorClient.call('/step_motor/step', request=json.dumps(payload))

    def photogrammetry_measure(self, step: int) -> None:
        payload = {"step": step}
        print(f"Sending photogrammetry request for step {step}")
        response = self.photogrammetryClient.call('/photogrammetry/measure', request=json.dumps(payload))
        print("Photogrammetry response:", response)

    def lidar_scan(self, step: int) -> None:
        payload = { "step": step }
        response = self.lidarClient.call('/lidar/measure', request=json.dumps(payload))

        lidar_response = json.loads(response.response)
        points = lidar_response["points"]

        self._scan_data.lidar_data[step] = [
            LaserScan.parse_obj(p) for p in points
        ]

    def process_data(self):
        self.dataClient.call('/data/process', request=self._scan_data.json())

if __name__ == "__main__":
    main()
