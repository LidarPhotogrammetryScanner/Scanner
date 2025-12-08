from typing import Dict, List
from dataclasses import dataclass, field


from domain.dto.Point import Point


@dataclass
class ScanData:
    current_step: int = 0
    lidar_data: Dict[int, List[Point]] = field(default_factory=dict)
