from typing import Dict, List
from pydantic import BaseModel

class DataPoint(BaseModel):
    angle: float = 0.0 #angle in radians
    radius: float = 0.0 #angle in meters
