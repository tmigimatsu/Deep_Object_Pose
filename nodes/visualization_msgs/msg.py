from geometry_msgs.msg import Pose, Position


class Rgba:
    def __init__(self):
        self.r = 0
        self.g = 0
        self.b = 0
        self.a = 0


class Marker:
    ADD = "add"
    CUBE = "cube"
    TEXT_VIEW_FACING = "text_view_facing"
    MESH_RESOURCE = "mesh_resource"
    DELETE = "delete"

    def __init__(self):
        self.header = None
        self.action = None
        self.pose = Pose()
        self.color = Rgba()
        self.ns = None
        self.id = None
        self.type = None
        self.scale = Position()


class MarkerArray:
    def __init__(self):
        self.markers = []
