from typing import Dict, List
from pydantic import BaseModel

class LaserScan(BaseModel):
    angle: float = 0.0 # Angle in radians
    distance: float = 0.0 # Distance in meters
