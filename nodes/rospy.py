import argparse
import pathlib
import yaml
import signal
import sys

import ctrlutils
import cv2

from geometry_msgs.msg import *
from sensor_msgs.msg import *
from vision_msgs.msg import *
from visualization_msgs.msg import *

# HACK: Add dope source to python path for main script.
sys.path.insert(0, str((pathlib.Path(__file__).parent.parent / "src").resolve()))

ARGS = None
CONFIG = None
REDIS = None
REDIS_PIPE = None
THREADS = []


def init_node(name):
    global ARGS
    global CONFIG
    global REDIS
    global REDIS_PIPE

    # Parse args.
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--redis_port", type=int, default=6379)
    parser.add_argument("-a", "--redis_pass", default="")
    ARGS = parser.parse_args()

    # Load config.
    with open("../config/config_pose.yaml", "r") as f:
        CONFIG = yaml.load(f)

    # Start Redis.
    REDIS = ctrlutils.RedisClient(port=ARGS.redis_port, password=ARGS.redis_pass)
    REDIS_PIPE = REDIS.pipeline()


class ROSInterruptException(Exception):
    pass


def signal_handler(sig, frame):
    global THREADS
    print(f"Stopping {len(THREADS)} threads...")
    for thread in THREADS:
        thread.stop()
    for thread in THREADS:
        thread.join()
    raise ROSInterruptException()


def spin():
    global THREADS
    for thread in THREADS:
        thread.start()
    signal.signal(signal.SIGINT, signal_handler)


def get_param(param, default=None):
    key = param[1:]
    if key in CONFIG:
        return CONFIG[key]
    return default


class Publisher:
    def __init__(self, key, value_type, queue_size=10):
        self._key = key

    def publish(self, value):
        global REDIS
        global REDIS_PIPE

        if isinstance(value, PoseStamped):
            REDIS_PIPE.set(
                self._key + "::pos",
                ctrlutils.redis.encode_matlab(value.pose.position._array()),
            )
            REDIS_PIPE.set(
                self._key + "::ori",
                ctrlutils.redis.encode_matlab(value.pose.orientation._array()),
            )
            REDIS_PIPE.execute()
        elif isinstance(value, Image):
            cv2.imshow(self._key, value.bgr)
            cv2.waitKey(1)
            REDIS.set_image(self._key, value.bgr)
        elif isinstance(value, CameraInfo):
            pass
        elif isinstance(value, Detection3DArray):
            for i, detection in enumerate(value.detections):
                REDIS_PIPE.set(
                    self._key + f"::{i}::pos",
                    ctrlutils.redis.encode_matlab(value.center.pose.position._array()),
                )
                REDIS_PIPE.set(
                    self._key + f"::{i}::ori",
                    ctrlutils.redis.encode_matlab(
                        value.center.pose.orientation._array()
                    ),
                )
                REDIS_PIPE.set(
                    self._key + f"::{i}::size",
                    ctrlutils.redis.encode_matlab(value.size._array()),
                )
            REDIS_PIPE.execute()
        elif isinstance(value, MarkerArray):
            pass
        elif isinstance(value, str):
            REDIS.set(self._key, value)
        else:
            raise ValueError(f"Unsupported value type: {type(value)}")

    def get_num_connections(self):
        return 1
