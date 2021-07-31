import numpy as np

from sensor_msgs.msg import Image


class CvBridge:
    def __init__(self):
        pass

    def imgmsg_to_cv2(self, img: Image, encoding: str) -> np.ndarray:
        if encoding == "rgb8":
            return img.bgr[:, :, ::-1].copy()
        elif encoding == "bgr8":
            return img.bgr
        raise ArgumentError(f"Unknown encoding: {encoding}")

    def cv2_to_imgmsg(self, mat: np.ndarray, encoding: str) -> Image:
        img = Image()
        if encoding == "rgb8":
            img.bgr = mat[:, :, ::-1]
        elif encoding == "bgr8":
            img.bgr = mat
        else:
            raise ArgumentError(f"Unknown encoding: {encoding}")
        return img
