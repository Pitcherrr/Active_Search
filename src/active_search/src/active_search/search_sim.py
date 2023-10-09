from pathlib import Path
import os
import pybullet as p
import pybullet_data
import rospkg
import time
import yaml

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

    def __init__(self, gui, scene_id):
        self.configure_physics_engine(gui, 60, 4)
        self.configure_visualizer()
        self.seed()
        self.load_robot()
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
        # self.rng = np.random.default_rng(seed) if seed else np.random
        self.rng = np.random


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

    def reset(self):
        self.set_arm_configuration([0.0, -1.39, 0.0, -2.36, 0.0, 1.57, 0.79])
        self.scene.clear()

        q = self.scene.generate(self.rng)
        self.set_arm_configuration(q)
        # self.scene.generate(self.rng)
        self.object_uids = self.scene.object_uids

        print("Cam position", p.getLinkState(self.camera.body_uid, self.camera.link_id))

        origin = Transform.from_translation(self.scene.origin)

        print(self.object_uids)

        target_uid = np.random.choice(self.object_uids)
        print(target_uid)
        target = get_target_bb(self, target_uid)
        tsdf = get_tsdf(self, reset_tsdf=True)
        # target_points = get_poi_torch(tsdf, target)
        target = AABBox(target.min_bound, target.max_bound)
        min_bound_t = (Transform.from_translation(target.min)*origin).translation
        max_bound_t = (Transform.from_translation(target.max)*origin).translation

        target.min = min_bound_t - np.array([0,0,0.1])
        target.max = max_bound_t - np.array([0,0,0.1])
        
        bbs = []
        #temporary-----------------------
        # for i in self.object_uids:
        #     if i != target_uid:
        #         bb = get_target_bb(self, i)
        #         bb = AABBox(bb.min_bound, bb.max_bound)
  
        #         min_bound_t = (Transform.from_translation(bb.min)*origin).translation
        #         max_bound_t = (Transform.from_translation(bb.max)*origin).translation

        #         bb.min = min_bound_t - np.array([0,0,0.1])
        #         bb.max = max_bound_t - np.array([0,0,0.1])

        #         bbs.append(bb)

        # bbs.append(target)
        #--------------------------------
        # return target_points
        target_bb = AABBox(self.scene.target_bb[0], self.scene.target_bb[1])
        return target_bb
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
        self.complete = False
        self.yaml_dir = pkg_root / "cfg/sim"

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

        if uid == self.target:
            self.complete_sim()
            return AABBox([0,0,0],[0,0,0])

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

    def complete_sim(self):
        print("############ Sim Complete ############")
        self.complete = True

    
    def save_scene_to_yaml(self, scene_data, output_file):
        output_path = self.yaml_dir / output_file

        with open(output_path, 'w') as yaml_file:
            yaml.dump(scene_data, yaml_file)



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

        print("Center", self.center)

    def generate(self, rng):
        self.complete = False
        self.load_config()
        self.add_support(self.center)
        i = 0
        for object in self.scene["objects"]:
            # urdf = urdfs_dir / object["object_id"] / "model.urdf"
            urdf = object["object_id"]
            ori = Rotation.from_euler("xyz", object["rpy"], degrees=True)
            pos = np.asarray(object["xyz"])
            scale = object.get("scale", 1)
            # if randomize := object.get("randomize", False):
            #     angle = rng.uniform(-randomize["rot"], randomize["rot"])
            #     ori = Rotation.from_euler("z", angle, degrees=True) * ori
            #     b = np.asarray(randomize["pos"])
            #     pos += rng.uniform(-b, b)
            uid = self.add_object(urdf, ori, pos, scale)

            # self.target = rng.choice(self.object_uids)

            # p.changeVisualShape(self.target, -1, rgbaColor=[1, 0, 0, 1])

            # self.target_bb = p.getAABB(self.target)
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

        self.target = self.object_uids[0]

        p.changeVisualShape(self.target, -1, rgbaColor=[1, 0, 0, 1])

        self.target_bb = p.getAABB(self.target)

        #this is the initial position of the robots camera link
        # cam = (0.167987435473768, -0.00028228516747251644, 0.7347501344586954)

        # bottle = self.object_uids[0]

        # bb = p.getAABB(bottle)

        # mid_bb = tuple(np.asarray(bb[0])+(np.asarray(bb[1])-np.asarray(bb[0]))/2)
 
        # print("Ray result", p.rayTest(cam, mid_bb))


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
        self.complete = False
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
    

class ActiveSearchScene(Scene):
    def __init__(self):
        super().__init__()
        self.center = np.r_[0.5, 0.0, 0.2]
        print(self.center)
        self.length = 0.3#not sure why this is 0.3, turns out this was the issue the whole time, scene length is a protion of the scene you want
        self.origin = self.center - np.r_[0.5 * self.length, 0.5 * self.length, 0.0]
        self.alt_origin = self.center - np.r_[0.5 * self.length, 0.5 * self.length, 0.0]
        self.object_urdfs = find_urdfs(urdfs_dir / "test")
        self.occluding_objs = find_urdfs(urdfs_dir / "occluding_objs/mug")
        self.scene_id = "random"
        #print(self.object_urdfs)

    
    def generate(self, rng):
        if self.scene_id.endswith(".yaml"):
            return self.generate_yaml_scene(self.scene_id)
        elif self.scene_id == "random":
            return self.generate_random_occluded(rng)
            # return YamlScene("test.yaml")
    
    def generate_random_occluded(self, rng, attempts=10):
        print("generating scene")

        self.complete = False
        self.add_support(self.center) #this the table that things sit on 0.3mx0.3m

        object_count = np.random.randint(1,8)
        urdfs = rng.choice(self.object_urdfs, object_count) #this going to select a random amount of objects from the set

        q = [0.0, -1.39, 0.0, -2.36, 0.0, 1.57, 0.79]
        q += rng.uniform(-0.08, 0.08, 7)

        scene_data = {
            "center": self.center.tolist(),
            "q": q.tolist(),  # Generate random robot configuration
            "objects": []
        }

        target = rng.choice(urdfs)
        scale = rng.uniform(0.4, 0.6)
        self.target = self.add_object(target, Rotation.identity(), np.zeros(3), scale)
        lower, upper = p.getAABB(self.target) #get the bounding box 
        z_offset = 0.5 * (upper[2] - lower[2]) + 0.002 #some bounding box offest
        ori = Rotation.from_euler("z", rng.uniform(0, 2 * np.pi)) #random rotation of object 
        pos = self.origin + np.r_[rng.uniform(0.4, 0.6, 2) * self.length, z_offset] #random position for object 
        p.resetBasePositionAndOrientation(self.target, pos, ori.as_quat()) #move object to this location
        p.changeVisualShape(self.target, -1, rgbaColor=[1, 0, 0, 1])
        self.target_bb = p.getAABB(self.target)
        mid_bb = tuple(np.asarray(self.target_bb[0])+(np.asarray(self.target_bb[1])-np.asarray(self.target_bb[0]))/2)


        object_data = {
            "object_id": str(target),
            "rpy": ori.as_euler('xyz', degrees=True).tolist(),  # You may need to adjust the orientation as needed
            "xyz": pos.tolist(),  # You may need to adjust the position as needed
            "scale": scale,
        }
        # objects.append(object_data)
        scene_data["objects"].append(object_data)


        scene_type = rng.choice(["fully", "infront"])
        # scene_type = "fully"

        if scene_type == "fully":
            done = False
            while not done:
                occluding = rng.choice(self.occluding_objs)
                
                print("occluding obj", occluding)

                rot_occ = np.random.uniform(0, 180)
                ori_occ = Rotation.from_euler("xyz", [90, 180, rot_occ], degrees=True)
                pos_occ = np.asarray(mid_bb) + [0,0,0.2]
                scale = 0.02 


                p.resetBasePositionAndOrientation(self.target, pos, ori.as_quat()) #move object to this location

                occluding_uid = self.add_object(occluding, ori_occ, pos_occ, scale)

                # print(p.getContactPoints(self.target))

                for _ in range(60):
                    p.stepSimulation() #step sim to run phyisics engine
                    # time.sleep(0.1)
                time.sleep(1)

                occ_low, occ_upp = p.getAABB(occluding_uid)
                target_low, target_upp = p.getAABB(self.target)

                print(occ_low, target_upp)
                if not bb_inside(occ_low, occ_upp, target_low, target_upp):
                    print("target was not in occluding object")
                    self.remove_object(occluding_uid)
                else:
                    done = True
                    object_data = {
                        "object_id": str(occluding),
                        "rpy": ori_occ.as_euler('xyz', degrees=True).tolist(),  # You may need to adjust the orientation as needed
                        "xyz": pos_occ.tolist(),  # You may need to adjust the position as needed
                        "scale": scale,
                    }
                    scene_data["objects"].append(object_data) 



        elif scene_type == "infront":
            occluding = rng.choice(self.object_urdfs)

            ori = Rotation.from_euler("xyz", [90, 270, 0], degrees=True)
            self.add_object(occluding, ori, np.asarray(mid_bb) + [0.1, 0, 0], 0.8)

        for urdf in urdfs:
            scale = rng.uniform(0.4, 0.8)
            uid = self.add_object(urdf, Rotation.identity(), np.zeros(3), scale)
            lower, upper = p.getAABB(uid) #get the bounding box 
            z_offset = 0.5 * (upper[2] - lower[2]) + 0.002 #some bounding box offest
            state_id = p.saveState()
            object_placed = True
            for _ in range(attempts):
                # Try to place and check for collisions
                ori = Rotation.from_euler("z", rng.uniform(0, 2 * np.pi)) #random rotation of object 
                pos = self.origin + np.r_[rng.uniform(0.2, 0.8, 2) * self.length, z_offset] #random position for object 
                p.resetBasePositionAndOrientation(uid, pos, ori.as_quat()) #move object to this location
                p.stepSimulation() #step sim to run phyisics engine 
                if not p.getContactPoints(uid): #check collisions 
                    break
                else:
                    p.restoreState(stateId=state_id)
                p.restoreState(stateId=state_id)
            else:
                # No placement found, remove the object
                self.remove_object(uid)
                object_placed = False

            if object_placed:
                object_data = {
                    "object_id": str(urdf),
                    "rpy": ori.as_euler('xyz', degrees=True).tolist(),  # You may need to adjust the orientation as needed
                    "xyz": pos.tolist(),  # You may need to adjust the position as needed
                    "scale": scale,
                }
                # objects.append(object_data)
                scene_data["objects"].append(object_data)

        for _ in range(10):
            p.stepSimulation() #step sim to run phyisics engine 


        self.target_bb = p.getAABB(self.target)

        # self.save_scene_to_yaml(scene_data, "test_3.yaml")
        return q
    

    def generate_yaml_scene(self, scene_id):
        
        self.config_path = pkg_root / "cfg/sim" / scene_id
        self.scene = load_yaml(self.config_path)

        self.center = np.asarray(self.scene["center"])
        self.origin = self.center - np.r_[0.5 * self.length, 0.5 * self.length, 0.0]
        self.alt_origin = self.center - np.r_[0.5 * self.length, 0.5 * self.length, 0.1]

        self.complete = False
        # self.load_config()
        self.add_support(self.center)
        i = 0
        for object in self.scene["objects"]:
            # urdf = urdfs_dir / object["object_id"] / "model.urdf"
            urdf = object["object_id"]
            ori = Rotation.from_euler("xyz", object["rpy"], degrees=True)
            pos = np.asarray(object["xyz"])
            scale = object.get("scale", 1)
            uid = self.add_object(urdf, ori, pos, scale)

            self.object_uids.append(uid)

        self.target = self.object_uids[0]

        p.changeVisualShape(self.target, -1, rgbaColor=[1, 0, 0, 1])

        self.target_bb = p.getAABB(self.target)

        for _ in range(60):
            p.stepSimulation()
        return self.scene["q"]

    



def get_scene(scene_id):
    if scene_id.endswith(".yaml"):
        return YamlScene(scene_id)
    elif scene_id == "random":
        return ActiveSearchScene()
        # return YamlScene("test.yaml")

    else:
        raise ValueError("Unknown scene {}.".format(scene_id))
    
def bb_inside(bb1_low, bb1_high, bb2_low, bb2_high):
    if (bb1_low[0] <= bb2_low[0] and
        bb1_low[1] <= bb2_low[1] and
        bb1_high[0] >= bb2_high[0] and
        bb1_high[1] >= bb2_high[1] and
        bb1_low[2] <= bb2_high[2]):
        return True
    else:
        return False

