from typing import Dict, List
from dataclasses import dataclass, field


from domain.dto.Point import Point


@dataclass
class ScanData:
    lidar_data: Dict[int, List[Point]] = field(default_factory=dict)
