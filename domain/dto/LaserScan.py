from dataclasses import dataclass

@dataclass
class LaserScan:
    ''' Represents the data of a single laser scan. '''
    angle: float = 0.0 # Angle in radians
    distance: float = 0.0 # Distance in meters
