from dataclasses import dataclass

@dataclass
class Point:
    ''' Represents a single point in the world coordinate system. '''
    x: float = 0.0 # X coordinate
    y: float = 0.0 # Y coordinate
    z: float = 0.0 # Z coordinate
