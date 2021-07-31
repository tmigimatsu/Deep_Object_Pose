class Image:
    def __init__(self):
        self.header = None
        self.bgr = None


class CameraInfo:
    def __init__(self):
        self.header = None
        self.P = []  # Rectified camera matrix [12].
        self.K = []  # Camera matrix [9].
        self.D = []  # Distortion coefficients [5].
