import numpy as np


class Position:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

    def _array(self):
        return np.array([self.x, self.y, self.z])


class Quaternion:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 0

    def _array(self):
        return np.array([self.x, self.y, self.z, self.w])


class Pose:
    def __init__(self):
        self.position = Position()
        self.orientation = Quaternion()


class PoseStamped:
    def __init__(self):
        self.header = None
        self.pose = Pose()
