import json
from typing import Dict, List
from pydantic import BaseModel
from domain.dto.laser_scan import LaserScan

class ScanData(BaseModel):
    lidar_data: Dict[int, List[LaserScan]] = {}