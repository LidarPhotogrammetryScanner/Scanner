from typing import Dict, List
from dataclasses import dataclass, field


from domain.dto.LaserScan import LaserScan


@dataclass
class ScanData:
    lidar_data: Dict[int, List[LaserScan]] = field(default_factory=dict) # laser scan per step
