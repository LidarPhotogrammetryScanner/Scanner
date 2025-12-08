import rclpy
from domain.communication.service_client import ServiceClient
from domain.dto.Point import Point
from src.orchestrator.scan_data import ScanData
from std_srvs.srv import Trigger
import config.config as config

def main():
    orchestrator = Orchestrator()
    orchestrator.run()


class Orchestrator:

    def __init__(self):
        rclpy.init()

        self.orchestratorClient = ServiceClient(node_name="orchestrator", service_type=Trigger)
        self.servoClient = ServiceClient(node_name="servo", service_type=Trigger)
        self.photogrammetryClient = ServiceClient(node_name="photogrammetry", service_type=Trigger)
        self.lidarClient = ServiceClient(node_name="LidarClient", service_type=Trigger)
        self.DataProcessingClient = ServiceClient(node_name="data", service_type=Trigger)
        self._scan_data = ScanData()

    def run(self):
        self._scan_data.lidar_data.setdefault(0, []).append(Point())
        for i in range(config.STEPS_PER_ROTATION):
            self.measure(i)

        for i in range(20):
            print(f"Iteration {i + 1}")

            print("=== CALL 1 ===")
            result1 = self.orchestratorClient.call('/data/process')
            if result1:
                print(f"CALL 1 returned message: {result1.message}")

        print("=== CALL 2 ===")
        result2 = self.orchestratorClient.call('/data/process')
        if result2:
            print(f"CALL 2 returned message: {result2.message}")

        rclpy.shutdown()
        print("Orchestrator finished", flush=True)



    def measure(self, i: int) -> None:

        return

if __name__ == "__main__":
    main()
