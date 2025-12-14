from dataclasses import dataclass

@dataclass
class DataPoint:
    angle: float = 0.0 #angle in radians
    radius: float = 0.0 #angle in meters
    step: float = 0
