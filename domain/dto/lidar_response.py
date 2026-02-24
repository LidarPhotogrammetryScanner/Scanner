from pydantic import BaseModel
from typing import List

from domain.dto.laser_scan import LaserScan


class LidarResponse(BaseModel):
    laser_scans: List[LaserScan]