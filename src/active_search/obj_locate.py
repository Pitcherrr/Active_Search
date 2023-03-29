import numpy as np
import pybullet as p
import os

from active_grasp.bbox import AABBox


class Target_Object:
    def __init__(self, sim, uid):
        self.sim = sim
        self.uid = uid

    def get_target_bbox(self):
        aabb_min, aabb_max = p.getAABB(self.uid)
        return AABBox(aabb_min, aabb_max)
    


def test(sim):

    pkg_root = "/home/tom/dev_ws/thesis_ws/src/active_grasp"
    urdfs_dir = os.path.join(pkg_root,"assets/test")

    item = p.loadURDF(os.path.join(urdfs_dir,"Paprika_800_tex.urdf"), [1,-1,1])

    obj = object(sim, item)

    bbox = obj.get_target_bbox()

    print(bbox.min)
    print(bbox.max)




