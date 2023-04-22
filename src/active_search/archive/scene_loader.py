import numpy as np
import pybullet as p
import os

from iio import load_yaml
from scipy.spatial.transform import Rotation


pkg_root = "/home/tom/dev_ws/thesis_ws/src/active_grasp"
urdfs_dir = os.path.join(pkg_root,"assests")
scene_yaml = os.path.join(pkg_root,"cfg/hw/scene01.yaml")

class Scene:
    def __init__(self):
        self.support_urdf = os.path.join(urdfs_dir,"plane/model.urdf")
        self.support_uid = -1
        self.object_uids = []

    def clear(self):
        self.remove_support()
        self.remove_all_objects()

    def generate(self, rng):
        raise NotImplementedError

    def add_support(self, pos):
        self.support_uid = p.loadURDF(str(self.support_urdf), pos, globalScaling=0.3)

    def remove_support(self):
        p.removeBody(self.support_uid)

    def add_object(self, urdf, ori, pos, scale=1.0):
        uid = p.loadURDF(str(urdf), pos, ori.as_quat(), globalScaling=scale)
        self.object_uids.append(uid)
        return uid

    def remove_object(self, uid):
        p.removeBody(uid)
        self.object_uids.remove(uid)

    def remove_all_objects(self):
        for uid in list(self.object_uids):
            self.remove_object(uid)

class YamlScene(Scene):
    def __init__(self, config_name):
        super().__init__()
        #self.config_path = pkg_root / "cfg/sim" / config_name #i feel like this should be the path to the scene number
        self.config_path = scene_yaml
        

    def load_config(self):
        self.scene = load_yaml(self.config_path)
        self.center = np.asarray(self.scene["center"])
        self.length = 0.3
        self.origin = self.center - np.r_[0.5 * self.length, 0.5 * self.length, 0.0]

    def generate(self, rng):
        self.load_config()
        self.add_support(self.center)
        for object in self.scene["objects"]:
            urdf = urdfs_dir / object["object_id"] / "model.urdf"
            ori = Rotation.from_euler("xyz", object["rpy"], degrees=True)
            pos = self.center + np.asarray(object["xyz"])
            scale = object.get("scale", 1)
            if randomize := object.get("randomize", False):
                angle = rng.uniform(-randomize["rot"], randomize["rot"])
                ori = Rotation.from_euler("z", angle, degrees=True) * ori
                b = np.asarray(randomize["pos"])
                pos += rng.uniform(-b, b)
            self.add_object(urdf, ori, pos, scale)
        for _ in range(60):
            p.stepSimulation()
        return self.scene["q"]