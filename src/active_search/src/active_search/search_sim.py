from pathlib import Path
import os
import pybullet as p
import pybullet_data
import rospkg
import time

from active_grasp.bbox import AABBox
from robot_helpers.bullet import *
# from .bullet_utils import *
from robot_helpers.io import load_yaml
from robot_helpers.model import KDLModel
from robot_helpers.spatial import Rotation, Transform
from vgn.perception import UniformTSDFVolume
from vgn.utils import find_urdfs, view_on_sphere
from vgn.detection import VGN, select_local_maxima
from .locate import *

# import vgn.visualizer as vis

rospack = rospkg.RosPack()
pkg_root = Path(rospack.get_path("active_search"))
urdfs_dir = pkg_root / "assets"


class Simulation:
    """Robot is placed s.t. world and base frames are the same"""

    def __init__(self, gui, scene_id, vgn_path):
        self.configure_physics_engine(gui, 60, 4)
        self.configure_visualizer()
        self.seed()
        self.load_robot()
        self.load_vgn(Path(vgn_path))
        self.scene = get_scene(scene_id)
        self.object_uids = self.scene.object_uids
        print("urdfs:", urdfs_dir)

    def configure_physics_engine(self, gui, rate, sub_step_count):
        os.environ["CUDA_VISIBLE_DEVICES"] = "0"
        self.rate = rate
        self.dt = 1.0 / self.rate
        p.connect(p.GUI if gui else p.DIRECT)
        # p.connect(p.GUI if gui else p.DIRECT, options= "--opengl2 --gpu")
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setPhysicsEngineParameter(fixedTimeStep=self.dt, numSubSteps=sub_step_count)
        p.setGravity(0.0, 0.0, -9.81)

    def configure_visualizer(self):
        # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(1.2, -120, -30, [0.4, 0.0, 0.2])

    def seed(self, seed=None):
        self.rng = np.random.default_rng(seed) if seed else np.random

    def load_robot(self):
        panda_urdf_path = urdfs_dir / "franka/panda_arm_hand.urdf"
        plane = p.loadURDF("plane.urdf")
        self.arm = BtPandaArm(panda_urdf_path)
        #self.arm = BtPandaArm(panda_urdf_path)
        self.gripper = BtPandaGripper(self.arm)
        self.model = KDLModel.from_urdf_file(
            panda_urdf_path, self.arm.base_frame, self.arm.ee_frame
        )
        #self.camera = BtCamera(320, 240, 0.96, 0.01, 1.0, self.arm.uid, 11) #depth is meant to be 1
        self.camera = BtCamera(320, 240, 0.96, 0.01, 1.0, self.arm.uid, 11) #depth is meant to be 1

    def load_vgn(self, model_path):
        self.vgn = VGN(model_path)

    def reset(self):
        self.set_arm_configuration([0.0, -1.39, 0.0, -2.36, 0.0, 1.57, 0.79])
        q = self.scene.generate(self.rng)
        self.set_arm_configuration(q)
        self.scene.clear()
        self.scene.generate(self.rng)
        self.object_uids = self.scene.object_uids

        print("Cam position", p.getLinkState(self.camera.body_uid, self.camera.link_id))

        origin = Transform.from_translation(self.scene.origin)

        print(self.object_uids)

        target_uid = np.random.choice(self.object_uids)
        print(target_uid)
        target = get_target_bb(self, target_uid)
        tsdf = get_tsdf(self, reset_tsdf=True)
        target_points = get_poi_torch(tsdf, target)
        target = AABBox(target.min_bound, target.max_bound)
        min_bound_t = (Transform.from_translation(target.min)*origin).translation
        max_bound_t = (Transform.from_translation(target.max)*origin).translation

        target.min = min_bound_t - np.array([0,0,0.1])
        target.max = max_bound_t - np.array([0,0,0.1])
        
        bbs = []
        #temporary-----------------------
        for i in self.object_uids:
            if i != target_uid:
                bb = get_target_bb(self, i)
                bb = AABBox(bb.min_bound, bb.max_bound)
  
                min_bound_t = (Transform.from_translation(bb.min)*origin).translation
                max_bound_t = (Transform.from_translation(bb.max)*origin).translation

                bb.min = min_bound_t - np.array([0,0,0.1])
                bb.max = max_bound_t - np.array([0,0,0.1])

                bbs.append(bb)

        bbs.append(target)
        #--------------------------------
        # return target_points
        return bbs
        #self.set_arm_configuration(q)

    def set_arm_configuration(self, q):
        for i, q_i in enumerate(q):
            p.resetJointState(self.arm.uid, i, q_i, 0)
        p.resetJointState(self.arm.uid, 9, 0.04, 0)
        p.resetJointState(self.arm.uid, 10, 0.04, 0)
        self.gripper.set_desired_width(0.4)

    def get_target_bbox(self, uid):
        aabb_min, aabb_max = p.getAABB(uid)
        return AABBox(aabb_min, aabb_max)
    
    def get_grasp_uid(self):

        print(self.object_uids)

        for obj in self.object_uids:

            contacts = p.getContactPoints(self.gripper.uid, obj)

            print(contacts)

            if contacts == ():
                continue
            elif contacts[0][2] == obj: #first contact point is in contact with obj B
                print(obj)
                return obj

            # if len(contacts) > 0:
            #     # If there is at least one contact, the robot arm is touching an object
            #     # Get the UID of the object that the robot arm is touching
            #     object_uid = contacts[0][2] if contacts[0][1] == self.gripper.uid else contacts[0][1]
            #     print("Robot arm is touching object with UID:", object_uid)
            #     return object_uid

            # else:
            #     # If there are no contacts, the robot arm is not touching any objects
            #     print("Robot arm is not touching any object, obj")
            #     return None

    def step(self):
        p.stepSimulation()


class Scene:
    def __init__(self):
        self.support_urdf = urdfs_dir / "plane/model.urdf"
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
        uid = p.loadURDF(str(urdf), pos, ori.as_quat(), flags = p.URDF_USE_MATERIAL_COLORS_FROM_MTL, globalScaling=scale)
        self.object_uids.append(uid)
        return uid

    def remove_object(self, uid):
        p.removeBody(uid)
        self.object_uids.remove(uid)

    def remove_object_ret_bb(self, uid):
        print(uid)

        if uid is None:
            return AABBox([0,0,0],[0,0,0])
      
        bb = p.getAABB(uid)
        origin = Transform.from_translation(self.origin)
        bb_min = (Transform.from_translation(bb[0])*origin.inv()).translation
        bb_max = (Transform.from_translation(bb[1])*origin.inv()).translation
        p.removeBody(uid)
        self.object_uids.remove(uid)
        return AABBox(bb_min, bb_max)

    def remove_all_objects(self):
        for uid in list(self.object_uids):
            self.remove_object(uid)


class YamlScene(Scene):
    def __init__(self, config_name):
        super().__init__()
        self.config_path = pkg_root / "cfg/sim" / config_name

    def load_config(self):
        self.scene = load_yaml(self.config_path)
        self.center = np.asarray(self.scene["center"])
        self.length = 0.3
        self.origin = self.center - np.r_[0.5 * self.length, 0.5 * self.length, 0.0]
        self.alt_origin = self.center - np.r_[0.5 * self.length, 0.5 * self.length, 0.1]
        print(pybullet_data.getDataPath())

    def generate(self, rng):
        self.load_config()
        self.add_support(self.center)
        i = 0
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
            uid = self.add_object(urdf, ori, pos, scale)
            # Simulate the mustard bottle going into the mug
            # dont need anymore as urdf was just broken 
            # if i == 1:
            #     for _ in range(4000):
            #         p.stepSimulation()

            #         # Move the mustard bottle downwards
            #         pos, orn = p.getBasePositionAndOrientation(2)
            #         p.resetBasePositionAndOrientation(2, [pos[0], pos[1], pos[2] - 0.0005], orn)

            #         # Check if the mustard bottle has entered the mug
            #         if pos[2] < 0.02:
            #             break
            # i+=1
            self.object_uids.append(uid)
        #this is the initial position of the robots camera link
        cam = (0.167987435473768, -0.00028228516747251644, 0.7347501344586954)

        bottle = self.object_uids[0]

        bb = p.getAABB(bottle)

        mid_bb = tuple(np.asarray(bb[0])+(np.asarray(bb[1])-np.asarray(bb[0]))/2)
 
        print("Ray result", p.rayTest(cam, mid_bb))


        for _ in range(60):
            p.stepSimulation()
        return self.scene["q"]


class RandomScene(Scene):
    def __init__(self):
        super().__init__()
        self.center = np.r_[0.5, 0.0, 0.2]
        print(self.center)
        self.length = 0.3#not sure why this is 0.3, turns out this was the issue the whole time, scene length is a protion of the scene you want
        self.origin = self.center - np.r_[0.5 * self.length, 0.5 * self.length, 0.0]
        self.alt_origin = self.center - np.r_[0.5 * self.length, 0.5 * self.length, 0.1]
        self.object_urdfs = find_urdfs(urdfs_dir / "test")
        #print(self.object_urdfs)

    def generate(self, rng, object_count=8, attempts=10):
        self.add_support(self.center) #this the table that things sit on 0.3mx0.3m
        urdfs = rng.choice(self.object_urdfs, object_count) #this going to select a random amount of objects from the set
        for urdf in urdfs:
            scale = rng.uniform(0.8, 1.0)
            uid = self.add_object(urdf, Rotation.identity(), np.zeros(3), scale)
            lower, upper = p.getAABB(uid) #get the bounding box 
            z_offset = 0.5 * (upper[2] - lower[2]) + 0.002 #some bounding box offest
            state_id = p.saveState()
            for _ in range(attempts):
                # Try to place and check for collisions
                ori = Rotation.from_euler("z", rng.uniform(0, 2 * np.pi)) #random rotation of object 
                pos = np.r_[rng.uniform(0.2, 0.8, 2) * self.length, z_offset] #random position for object 
                p.resetBasePositionAndOrientation(uid, self.origin + pos, ori.as_quat()) #move object to this location
                p.stepSimulation() #step sim to run phyisics engine 
                if not p.getContactPoints(uid): #check collisions 
                    break
                else:
                    p.restoreState(stateId=state_id)
                p.restoreState(stateId=state_id)
            else:
                # No placement found, remove the object
                self.remove_object(uid)
        q = [0.0, -1.39, 0.0, -2.36, 0.0, 1.57, 0.79]
        q += rng.uniform(-0.08, 0.08, 7)
        return q
    

class RandomOccludedScene(Scene):
    def __init__(self):
        super().__init__()
        self.center = np.r_[0.5, 0.0, 0.2]
        print(self.center)
        self.length = 0.3#not sure why this is 0.3, turns out this was the issue the whole time, scene length is a protion of the scene you want
        self.origin = self.center - np.r_[0.5 * self.length, 0.5 * self.length, 0.0]
        self.alt_origin = self.center - np.r_[0.5 * self.length, 0.5 * self.length, 0.0]
        self.object_urdfs = find_urdfs(urdfs_dir / "test")
        self.occluding_objs = find_urdfs(urdfs_dir / "occluding_objs")
        #print(self.object_urdfs)

    def generate(self, rng, object_count=5, attempts=10):
        self.add_support(self.center) #this the table that things sit on 0.3mx0.3m
        urdfs = rng.choice(self.object_urdfs, object_count) #this going to select a random amount of objects from the set
        occluding = rng.choice(self.occluding_objs)
        for urdf in urdfs:
            scale = rng.uniform(0.4, 0.8)
            uid = self.add_object(urdf, Rotation.identity(), np.zeros(3), scale)
            lower, upper = p.getAABB(uid) #get the bounding box 
            z_offset = 0.5 * (upper[2] - lower[2]) + 0.002 #some bounding box offest
            state_id = p.saveState()
            for _ in range(attempts):
                # Try to place and check for collisions
                ori = Rotation.from_euler("z", rng.uniform(0, 2 * np.pi)) #random rotation of object 
                pos = np.r_[rng.uniform(0.2, 0.8, 2) * self.length, z_offset] #random position for object 
                p.resetBasePositionAndOrientation(uid, self.origin + pos, ori.as_quat()) #move object to this location
                p.stepSimulation() #step sim to run phyisics engine 
                if not p.getContactPoints(uid): #check collisions 
                    break
                else:
                    p.restoreState(stateId=state_id)
                p.restoreState(stateId=state_id)
            else:
                # No placement found, remove the object
                self.remove_object(uid)

        #add the occluding object to the scene 
        
        target = rng.choice(self.object_uids)

        bb = p.getAABB(target)

        mid_bb = tuple(np.asarray(bb[0])+(np.asarray(bb[1])-np.asarray(bb[0]))/2)

        ori = Rotation.from_euler("xyz", [0, 180, 0], degrees=True)
        
        self.add_object(occluding, ori, np.asarray(mid_bb)+ [0,0,0.2], 1.2)
        
        q = [0.0, -1.39, 0.0, -2.36, 0.0, 1.57, 0.79]
        q += rng.uniform(-0.08, 0.08, 7)
        return q


def get_scene(scene_id):
    if scene_id.endswith(".yaml"):
        return YamlScene(scene_id)
    elif scene_id == "random":
        return RandomScene()

    else:
        raise ValueError("Unknown scene {}.".format(scene_id))
