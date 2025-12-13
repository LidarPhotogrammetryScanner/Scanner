from typing import Dict, List
from dataclasses import dataclass, field


from domain.dto.Point import DataPoint


@dataclass
class ScanData:
    lidar_data: Dict[float, List[DataPoint]] = field(default_factory=dict) #lidar point per step
