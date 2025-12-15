import json
from typing import Dict, List
from pydantic import BaseModel
from domain.dto.data_point import DataPoint

class ScanData(BaseModel):
    lidar_data: Dict[int, List[DataPoint]] = {}