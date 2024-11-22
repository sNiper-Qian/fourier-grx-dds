from enum import Enum
from dataclasses import dataclass
import numpy as np

class BaseControlGroup(tuple, Enum):
    """Base class for control group enumerations."""

    @property
    def slice(self):
        """Returns a slice object representing the range of indices for the control group."""
        return slice(self.value[0], self.value[0] + self.value[1])

    @property
    def num_joints(self):
        """Returns the number of joints in the control group."""
        return self.value[1]

    @classmethod
    def from_string(cls, s: str):
        """Creates an enum member from a string representation."""
        return getattr(cls, s.upper())

class GR1ControlGroup(BaseControlGroup):
    """GR1 Control group enumeration. Each group is a tuple of (start, num_joints). Available groups are: ALL, LEFT_LEG, RIGHT_LEG, WAIST, HEAD, LEFT_ARM, RIGHT_ARM, LOWER, UPPER, UPPER_EXTENDED."""

    ALL = (0, 32)
    LEFT_LEG = (0, 6)
    RIGHT_LEG = (6, 6)
    WAIST = (12, 3)
    HEAD = (15, 3)
    LEFT_ARM = (18, 7)
    RIGHT_ARM = (25, 7)
    LOWER = (0, 18)
    UPPER = (18, 14)
    UPPER_EXTENDED = (12, 20)

class GR2ControlGroup(BaseControlGroup):
    """GR2 Control group enumeration. Each group is a tuple of (start, num_joints). Available groups are: ALL, LEFT_LEG, RIGHT_LEG, WAIST, HEAD, LEFT_ARM, RIGHT_ARM, LOWER, UPPER, UPPER_EXTENDED."""
    # TODO: Implement GR2 control groups
    pass

@dataclass
class Trajectory:
    start: np.ndarray
    end: np.ndarray
    duration: float

    def at(self, t: float):
        return self.start + (self.end - self.start) * t / self.duration

    def finished(self, t: float):
        return t >= self.duration