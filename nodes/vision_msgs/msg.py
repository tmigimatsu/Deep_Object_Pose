from geometry_msgs.msg import Pose, Position, PoseStamped


class Bbox:
    def __init__(self):
        self.center = Pose()
        self.size = Position()


class Detection3D:
    def __init__(self):
        self.results = []
        self.bbox = Bbox()


class Detection3DArray:
    def __init__(self):
        self.header = None
        self.detections = []


class ObjectHypothesisWithPose:
    def __init__(self):
        self.id = None
        self.score = None
        self.pose = PoseStamped()
