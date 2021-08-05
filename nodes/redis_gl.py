import enum
import json
from typing import Any, Dict, List

import ctrlutils
import numpy as np
from scipy.spatial.transform import Rotation as R


class webapp:
    KEY_PREFIX = "webapp::"
    KEY_RESOURCES_PREFIX = KEY_PREFIX + "resources::"


kName = "simulator"
KEY_PREFIX = webapp.KEY_PREFIX + kName + "::"
KEY_ARGS = KEY_PREFIX + "args"
KEY_RESOURCES = webapp.KEY_RESOURCES_PREFIX + kName


class ModelKeys:
    def __init__(self, key_namespace: str):
        self.key_namespace = key_namespace
        self.key_robots_prefix = key_namespace + "::model::robot::"
        self.key_objects_prefix = key_namespace + "::model::object::"
        self.key_trajectories_prefix = key_namespace + "::model::trajectory::"
        self.key_cameras_prefix = key_namespace + "::model::camera::"

    def to_json(self) -> Dict[str, Any]:
        return {
            "key_robots_prefix": self.key_robots_prefix,
            "key_objects_prefix": self.key_objects_prefix,
            "key_trajectories_prefix": self.key_trajectories_prefix,
            "key_cameras_prefix": self.key_cameras_prefix,
        }


class Graphics:
    class Geometry:
        class Type(enum.Enum):
            UNDEFINED = 0
            BOX = 1
            CAPSULE = 2
            CYLINDER = 3
            SPHERE = 4
            MESH = 5

        def __init__(
            self,
            geometry_type: Type = Type.UNDEFINED,
            scale: np.ndarray = np.ones(3),
            radius: float = 0.0,
            length: float = 0,
            mesh: str = "",
        ):
            self.type = geometry_type
            self.scale = scale
            self.radius = radius
            self.length = length
            self.mesh = mesh

        def to_json(self) -> Dict[str, Any]:
            json_dict = {
                "type": self.type.name.lower(),
            }
            if self.type == self.Type.BOX:
                json_dict["scale"] = self.scale
            elif self.type in (self.Type.CAPSULE, self.Type.CYLINDER):
                json_dict["radius"] = self.radius
                json_dict["length"] = self.length
            elif self.type == self.Type.SPHERE:
                json_dict["radius"] = self.radius
            elif self.type == self.Type.MESH:
                json_dict["mesh"] = self.mesh
                json_dict["scale"] = self.scale.tolist()
            return json_dict

    class Material:
        def __init__(
            self, name: str = "", rgba: np.ndarray = np.ones(4), texture: str = ""
        ):
            self.name = name
            self.rgba = rgba
            self.texture = texture

        def to_json(self) -> Dict[str, Any]:
            return {
                "name": self.name,
                "rgba": self.rgba.tolist(),
                "texture": self.texture,
            }

    def __init__(
        self,
        name: str = "",
        T_to_parent: np.ndarray = np.eye(4),
        geometry: Geometry = Geometry(),
        material: Material = Material(),
    ):
        self.name = name
        quat = R.from_matrix(T_to_parent[:3, :3]).as_quat()
        self.T_to_parent = {
            "pos": T_to_parent[:3, 3].tolist(),
            "ori": {
                "x": quat[0],
                "y": quat[1],
                "z": quat[2],
                "w": quat[3],
            }
        }
        self.geometry = geometry
        self.material = material

    def to_json(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "T_to_parent": self.T_to_parent,
            "geometry": self.geometry.to_json(),
            "material": self.material.to_json(),
        }


class ObjectModel:
    def __init__(self, name: str, graphics: List[Graphics], key_pos: str, key_ori: str):
        self.name = name
        self.graphics = graphics
        self.key_pos = key_pos
        self.key_ori = key_ori

    def to_json(self) -> Dict[str, Any]:
        return {
            "graphics": [graphics.to_json() for graphics in self.graphics],
            "key_pos": self.key_pos,
            "key_ori": self.key_ori,
        }


def register_model_keys(redis: ctrlutils.RedisClient, model_keys: ModelKeys):
    redis.set(
        KEY_ARGS + "::" + model_keys.key_namespace, json.dumps(model_keys.to_json())
    )


def unregister_model_keys(redis: ctrlutils.RedisClient, model_keys: ModelKeys):
    redis.delete(KEY_ARGS + "::" + model_keys.key_namespace)


def register_resource_path(redis: ctrlutils.RedisClient, path: str):
    redis.sadd(KEY_RESOURCES, path)


def unregister_resource_path(redis: ctrlutils.RedisClient, path: str):
    redis.srem(KEY_RESOURCES, path)


def register_object(
    redis: ctrlutils.RedisClient, model_keys: ModelKeys, obj: ObjectModel
):
    redis.set(model_keys.key_objects_prefix + obj.name, json.dumps(obj.to_json()))
