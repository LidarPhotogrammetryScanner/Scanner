from pydantic import BaseModel
from typing import List

from domain.dto.data_point import DataPoint


class LidarResponse(BaseModel):
    points: List[DataPoint]