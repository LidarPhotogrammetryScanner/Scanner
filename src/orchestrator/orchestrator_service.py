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
        print("starting in 60 seconds")
        time.sleep(60)
        rclpy.init()

        self.orchestratorClient = ServiceClient(node_name="orchestrator", service_type=Trigger)
        self.stepMotorClient = ServiceClient(node_name="step_motor", service_type=JsonIO)
        self.servoMotorClient = ServiceClient(node_name="servo_motor", service_type=JsonIO)
        # self.photogrammetryClient = ServiceClient(node_name="photogrammetry", service_type=JsonIO)
        self.lidarClient = ServiceClient(node_name="lidar", service_type=JsonIO)
        self.dataClient = ServiceClient(node_name="data", service_type=JsonIO)

        self._scan_data: ScanData = ScanData()

    def run(self):

        self.lidar_scan(0)

        for i in range(config.STEPS_PER_ROTATION):
            self.measure(i)

        self.process_data()

    def measure(self, step: int) -> None:
        print("NOW PREPARE STEP {}".format(step + 1))
        if step != 0:
            self.step_step_motor(step)
        self.lidar_scan(step)
        print("DONE WITH STEP {}".format(step + 1))

    def step_step_motor(self, step: int) -> None:
        payload = {"step": step}
        self.stepMotorClient.call('/step_motor/step', request=json.dumps(payload))
        # self.servoMotorClient.call('/servo_motor/step', request=json.dumps(payload))

    def photogrammetry_measure(self, step: int) -> None:
        payload = {"step": step}
        # response = self.photogrammetryClient.call('/photogrammetry/measure', request=json.dumps(payload))
        # print("Photogrammetry response:", response)

    def lidar_scan(self, step: int) -> None:
        payload = {"step": step}
        response = self.lidarClient.call('/lidar/measure', request=json.dumps(payload))

        lidar_response = json.loads(response.response)

        scans = lidar_response["laser_scans"]

        self._scan_data.lidar_data[step] = [
            LaserScan.parse_obj(p) for p in scans
        ]

    def process_data(self):
        self.step_step_motor(0)
        self.dataClient.call('/data/process', request=self._scan_data.json())


if __name__ == "__main__":
    main()
