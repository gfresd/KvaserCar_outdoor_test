
from typing import Tuple, Optional

class ComMsg:
    def __init__(self, id, time_stamp, arrival_time, front, vel, length=None):
        """

        :param time_stamp: original obps time_stamp
        :param arrival_time: time at reception of this message
        :param front:
        :param back:
        :param vel:
        """
        self.id = id
        self.time_stamp: int = time_stamp
        self.arrival_time: int = arrival_time
        self.front: Tuple[float, float] = front
        self.velocity: float = vel
        self.length: Optional[float] = length