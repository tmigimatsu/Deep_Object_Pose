import threading

import ctrlutils
import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image, CameraInfo


class Subscriber:
    def __init__(self, key, value_type):
        pass


class TimeSynchronizer:
    def __init__(self, keys, queue_size):
        self._run = False
        self._thread = None

        self._redis = ctrlutils.RedisClient(
            port=rospy.ARGS.redis_port, password=rospy.ARGS.redis_pass
        )
        redis_pipe = self._redis.pipeline()
        redis_pipe.get("rgbd::camera_0::color::intrinsic")
        redis_pipe.get("rgbd::camera_0::color::distortion")
        b_intrinsic, b_distortion = redis_pipe.execute()

        intrinsic = ctrlutils.redis.decode_matlab(b_intrinsic)
        distortion = ctrlutils.redis.decode_matlab(b_distortion)

        self._camera_info = CameraInfo()
        self._camera_info.P = np.concatenate((intrinsic, np.zeros((3, 1))), axis=1)
        self._camera_info.K = intrinsic
        self._camera_info.D = distortion

    def start(self):
        self._run = True
        self._thread.start()

    def stop(self):
        self._run = False

    def join(self):
        self._thread.join()

    def _redis_thread(self, callback):
        timer = ctrlutils.Timer(30)
        while self._run:
            timer.sleep()

            img = Image()
            bgr = self._redis.get_image("rgbd::camera_0::color")
            img.bgr = cv2.undistort(bgr, self._camera_info.K, self._camera_info.D)

            callback(img, self._camera_info)

    def registerCallback(self, callback):
        self._thread = threading.Thread(target=self._redis_thread, args=(callback,))
        rospy.THREADS.append(self)
