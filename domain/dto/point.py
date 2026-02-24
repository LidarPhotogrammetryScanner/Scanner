from typing import Dict, List
from pydantic import BaseModel

class Point(BaseModel):
    x: float = 0.0 # X coordinate in meters
    y: float = 0.0 # Y coordinate in meters
    z: float = 0.0 # Z coordinate in meters
