import argparse
import pathlib
import yaml
import re
import signal
import sys

import ctrlutils
import cv2
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import *
from sensor_msgs.msg import *
from vision_msgs.msg import *
from visualization_msgs.msg import *
import redis_gl

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
    parser.add_argument("-rh", "--redis_host", default="127.0.0.1")
    parser.add_argument("-p", "--redis_port", type=int, default=6379)
    parser.add_argument("-a", "--redis_pass", default="")
    ARGS = parser.parse_args()

    # Load config.
    with open("../config/config_pose.yaml", "r") as f:
        CONFIG = yaml.load(f)

    # Start Redis.
    REDIS = ctrlutils.RedisClient(
        host=ARGS.redis_host, port=ARGS.redis_port, password=ARGS.redis_pass
    )
    REDIS_PIPE = REDIS.pipeline()

    # Set up redis-gl.
    model_keys = redis_gl.ModelKeys("dope")
    redis_gl.register_model_keys(REDIS, model_keys)
    models = get_param("~weights").keys()
    meshes = get_param("~meshes")
    for model in models:
        geometry = redis_gl.Graphics.Geometry(
            redis_gl.Graphics.Geometry.Type.MESH, mesh=meshes[model]
        )
        material = redis_gl.Graphics.Material(
            "textured", texture=meshes[model][:-3] + "mtl"
        )
        obj = redis_gl.ObjectModel(
            model,
            graphics=[redis_gl.Graphics(model, geometry=geometry, material=material)],
            key_pos=f"dope::{model}::pos",
            key_ori=f"dope::{model}::ori",
        )
        redis_gl.register_object(REDIS, model_keys, obj)
    path_resources = pathlib.Path(__file__).parent.parent / "resources"
    redis_gl.register_resource_path(REDIS, str(path_resources))


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
        global REDIS
        global REDIS_PIPE
        m = re.match(r"dope/(pose|dimension)_(.*)", key)
        if m is not None:
            name_obj = m[2]
            self._key = f"dope::{name_obj}::"
            if m[1] == "pose":
                REDIS_PIPE.get("rgbd::camera_0::pos")
                REDIS_PIPE.get("rgbd::camera_0::ori")
                b_pos, b_ori = REDIS_PIPE.execute()
                pos = ctrlutils.redis.decode_matlab(b_pos)
                ori = ctrlutils.redis.decode_matlab(b_ori)
                self._T_camera_to_world = np.eye(4)
                self._T_camera_to_world[:3, :3] = R.from_quat(ori).as_matrix()
                self._T_camera_to_world[:3, 3] = pos
        else:
            self._key = key

    def publish(self, value):
        global REDIS
        global REDIS_PIPE

        if isinstance(value, PoseStamped):
            quat_camera = value.pose.orientation._array()
            T_camera = np.eye(4)
            T_camera[:3, :3] = R.from_quat(quat_camera).as_matrix()
            T_camera[:3, 3] = value.pose.position._array()
            T_world = self._T_camera_to_world.dot(T_camera)
            pos_world = T_world[:3, 3]
            quat_world = R.from_matrix(T_world[:3, :3]).as_quat()

            REDIS_PIPE.set(
                self._key + "pos",
                ctrlutils.redis.encode_matlab(pos_world),
            )
            REDIS_PIPE.set(
                self._key + "ori",
                ctrlutils.redis.encode_matlab(quat_world),
            )
            REDIS_PIPE.execute()
        elif isinstance(value, Image):
            cv2.imshow(self._key, value.bgr)
            cv2.waitKey(1)
            # REDIS.set_image(self._key, value.bgr)
        elif isinstance(value, CameraInfo):
            pass
        elif isinstance(value, Detection3DArray):
            pass
            # for i, detection in enumerate(value.detections):
            #     REDIS_PIPE.set(
            #         self._key + f"::{i}::pos",
            #         ctrlutils.redis.encode_matlab(
            #             detection.bbox.center.position._array()
            #         ),
            #     )
            #     REDIS_PIPE.set(
            #         self._key + f"::{i}::ori",
            #         ctrlutils.redis.encode_matlab(
            #             detection.bbox.center.orientation._array()
            #         ),
            #     )
            #     REDIS_PIPE.set(
            #         self._key + f"::{i}::size",
            #         ctrlutils.redis.encode_matlab(detection.bbox.size._array()),
            #     )
            #     print(detection.results)
            # REDIS_PIPE.execute()
        elif isinstance(value, MarkerArray):
            pass
        elif isinstance(value, str):
            if self._key[-2:] == "::":
                dim = np.array([float(d) for d in value[1:-1].split(", ")]) / 100
                REDIS.set_matrix(self._key + "size", dim)
            else:
                REDIS.set(self._key, value)
        else:
            raise ValueError(f"Unsupported value type: {type(value)}")

    def get_num_connections(self):
        return 1
