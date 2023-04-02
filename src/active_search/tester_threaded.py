import pybullet
import pybullet_data
import math
import time
import numpy as np
import threading 
import open3d as o3d
import cv2
import os
from queue import Queue
from scipy.spatial.transform import Rotation


from robot_helpers.bullet import *
from robot_helpers.model import *
from robot_helpers.spatial import Transform
from search_sim import Simulation
from vgn.perception import UniformTSDFVolume
from dynamic_perception import SceneTSDFVolume
from vgn.utils import view_on_sphere

class Environment:
    def __init__(self, gui, scene_id, vgn_path):
        self.gui = gui
        self.scene_id = scene_id
        self.vgn_path = vgn_path

    def load_engine(self):
        self.sim = Simulation(self.gui, self.scene_id, self.vgn_path)
        self.scene_origin = Transform.from_translation(self.sim.scene.alt_origin)
        self.sim_state = Queue()
        self.sim.reset()

    
    def get_tsdf(self):

        tsdf = SceneTSDFVolume(self.sim.scene.length, 200)
        
        cam_data = self.sim.camera.get_image()
        image = cam_data[0]
        depth_img = cam_data[1]

        tsdf.integrate(depth_img, self.sim.camera.intrinsic, (self.sim.camera.pose.inv()*self.scene_origin).as_matrix()) 
        
        self.tsdf = tsdf.o3dvol

        self.get_poi()

        tsdf_mesh = tsdf.o3dvol.extract_point_cloud()
        print(tsdf_mesh)


        self.sim_state.put([tsdf_mesh, image])


    def get_target(self):
        print(self.sim.object_uids)
        self.target_uid = np.random.choice(self.sim.object_uids)
  

    def get_target_bb(self):
        target_bb =  o3d.geometry.AxisAlignedBoundingBox()
        # target_bb =  o3d.geometry.OrientedBoundingBox()      
        min_bound, max_bound = pybullet.getAABB(self.target_uid)
        
        origin = Transform.from_translation(self.sim.scene.origin)

        min_bound_t = (Transform.from_translation(min_bound)*origin.inv()).translation
        max_bound_t = (Transform.from_translation(max_bound)*origin.inv()).translation

        target_bb.min_bound = min_bound_t + np.array([0,0,0.1])
        target_bb.max_bound = max_bound_t + np.array([0,0,0.1])

        target_bb.scale(0.8, target_bb.get_center())

        self.target_bb = target_bb


    def get_poi(self):
        # Assume we have the bounding box in world coordinates and the TSDF volume as a numpy array
        world_to_grid = np.linalg.inv(self.tsdf.get_intrinsics().intrinsic_matrix)
        world_to_voxel = world_to_grid @ self.tsdf.extrinsic

        # Get the bounding box in voxel grid coordinates
        # bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(np.asarray(self.target_bb.get_box_points()))
        bbox_min = self.target_bb.get_min_bound()
        bbox_max = self.target_bb.get_max_bound()
        bbox_center = (bbox_min + bbox_max) / 2
        bbox_center_voxel = world_to_voxel @ np.append(bbox_center, 1)
        bbox_size_voxel = np.abs(bbox_max - bbox_min) / np.array(self.tsdf.get_voxel_size())
        bbox_voxel = o3d.geometry.AxisAlignedBoundingBox(bbox_center_voxel[:3], bbox_size_voxel)

        # Shift the bounding box by a distance behind the TSDF grid
        bbox_voxel.translate([0, 0, -10])

        # Find all the voxels that intersect with the shifted bounding box
        voxel_indices = np.transpose(np.nonzero(self.tsdf.get_voxels()))
        voxel_coords = np.hstack((voxel_indices, np.ones((voxel_indices.shape[0], 1))))
        voxel_coords_world = world_to_voxel @ voxel_coords.T
        voxel_coords_bbox = bbox_voxel.get_transform().inverse() @ voxel_coords_world
        voxels_in_bbox = np.logical_and(np.all(voxel_coords_bbox >= -0.5, axis=0), np.all(voxel_coords_bbox <= 0.5, axis=0))

        # Mark the corresponding points in the TSDF volume
        points_in_bbox = voxel_coords_world[:3, voxels_in_bbox].T
        for p in points_in_bbox:
            self.tsdf.set_voxel(p, 1.0)


    def center_view(self, vis):
        vis.reset_view_point(True)


    def kill_o3d(self, vis):
        vis.destroy_window()
        self.o3d_window_active = False


    def open3d_window(self, reset_bb: bool = True):
        """Need to run this in a loop on another thread, 
        may have to parse in camera data from main thread running pybullet"""
        
        self.paused = False
        self.o3d_window_active = True
        tsdf_exists = False

        o3d.core.Device("cuda:0")

        vis = o3d.visualization.VisualizerWithKeyCallback()

        vis.create_window(window_name = "Depth Camera")

        #some key call backs for controlling the sim 
        vis.register_key_callback(ord("C"), self.center_view)
        vis.register_key_callback(ord("X"), self.kill_o3d)

        while self.sim_state.empty():
             continue
        
        if self.sim_state.qsize() > 1:
            print("wtfffffffffffff")
        
        state = self.sim_state.get()
        tsdf_mesh_init, image = state

        target_bb = o3d.geometry.OrientedBoundingBox.create_from_axis_aligned_bounding_box(self.target_bb) 
        target_bb.color = [0, 1, 0] 

        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05)
        origin_sphere = mesh = o3d.geometry.TriangleMesh.create_sphere(0.05)
        origin_sphere.transform(Transform.from_translation(self.sim.scene.origin).as_matrix())

        vis.add_geometry(tsdf_mesh_init, reset_bounding_box = True)
        vis.add_geometry(target_bb, reset_bounding_box = reset_bb)
        vis.add_geometry(frame, reset_bounding_box = reset_bb)

        vis.update_renderer()

        vis.remove_geometry(tsdf_mesh_init, reset_bounding_box = reset_bb)
        while self.o3d_window_active:
            # vis.poll_events()
            # vis.update_renderer()
            if not self.sim_state.empty():


                if tsdf_exists:
                    vis.remove_geometry(tsdf_mesh, reset_bounding_box = reset_bb)
                    vis.remove_geometry(bb, reset_bounding_box = reset_bb)

                state = self.sim_state.get()

                tsdf_mesh, image = state

                tsdf_exists = True
                
                aligned_bb = tsdf_mesh.get_axis_aligned_bounding_box()
                bb = o3d.geometry.OrientedBoundingBox.create_from_axis_aligned_bounding_box(aligned_bb) 
                bb.color = [1, 0, 0] 

                target_bb = o3d.geometry.OrientedBoundingBox.create_from_axis_aligned_bounding_box(self.target_bb) 
                target_bb.color = [0, 1, 0] 
                #vis.add_geometry(target_bb, reset_bounding_box = reset_bb)

                vis.add_geometry(tsdf_mesh, reset_bounding_box = reset_bb)
                vis.add_geometry(bb, reset_bounding_box = reset_bb)

                vis.poll_events()
                vis.update_renderer()
                #vis.remove_geometry(tsdf_mesh, reset_bounding_box = reset_bb)
                
                #vis.remove_geometry(target_bb, reset_bounding_box = reset_bb)


    def full_scene(self, reset_bb: bool = True):
        o3d.core.Device("cuda:0")
    
        #vis = o3d.visualization.Visualizer()
        while self.sim_state.empty():
             continue
        state = self.sim_state.get()
        tsdf_mesh_init, image = state
        o3d.visualization.draw_geometries([tsdf_mesh_init], "Full Scene", 1280, 720)


    def live_feed(self):

        while self.sim_state.empty():
             continue
        
        state = self.sim_state.get()
        tsdf_mesh_init, image = state

        cv2.namedWindow("RGB Camera", cv2.WINDOW_NORMAL)
        
        while True:
            state = self.sim_state.get()
            tsdf_mesh, image = state

            cv2.imshow("RGB Camera", image)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break

            #image.fill(0)

        cv2.destroyAllWindows()


    def run(self):

        # Create two threads, one for each window
        # self.thread_live_feed = threading.Thread(target=self.live_feed)
        # self.thread_live_feed.start()
        self.thread_open3d = threading.Thread(target=self.open3d_window, args= (False,))
        #self.thread_open3d = threading.Thread(target=self.full_scene, args= (False,))
        self.thread_open3d.start()


        [j1_init, j2_init, j3_init, j4_init, j5_init, j6_init, j7_init] = [j[0] for j in pybullet.getJointStates(self.sim.arm.uid, range(7))]

        #set up user inputs 
        j1_comm = pybullet.addUserDebugParameter("J1", -math.pi, math.pi, 0)
        j2_comm = pybullet.addUserDebugParameter("J2", -math.pi, math.pi, 0)
        j3_comm = pybullet.addUserDebugParameter("J3", -math.pi, math.pi, 0)
        j4_comm = pybullet.addUserDebugParameter("J4", -math.pi, math.pi, 0)    
        j5_comm = pybullet.addUserDebugParameter("J5", -math.pi, math.pi, 0)
        j6_comm = pybullet.addUserDebugParameter("J6", -math.pi, math.pi, 0)
        j7_comm = pybullet.addUserDebugParameter("J7", -math.pi, math.pi, 0)
        grip_comm = pybullet.addUserDebugParameter("Grip", 0,0.1,0)
        # cam_rot_comm = pybullet.addUserDebugParameter("Rotate", 1,0,1)
        cam_rot_comm = pybullet.addUserDebugParameter("Rotate", 0, 2*math.pi, 0)

        frame_buff = 0  
        frame_count = 0

        while True:

            j1 = j1_init + pybullet.readUserDebugParameter(j1_comm)
            j2 = j2_init + pybullet.readUserDebugParameter(j2_comm)
            j3 = j3_init + pybullet.readUserDebugParameter(j3_comm)
            j4 = j4_init + pybullet.readUserDebugParameter(j4_comm)
            j5 = j5_init + pybullet.readUserDebugParameter(j5_comm)
            j6 = j6_init + pybullet.readUserDebugParameter(j6_comm)
            j7 = j7_init + pybullet.readUserDebugParameter(j7_comm)
            cam_rot = pybullet.readUserDebugParameter(cam_rot_comm)
            grip_width = pybullet.readUserDebugParameter(grip_comm)


            robot_pos = [j1, j2, j3, j4, j5, j6, j7]
            self.sim.gripper.set_desired_width(grip_width)
            self.sim.camera.rot = cam_rot


            if frame_buff == 10:
                self.get_tsdf()
                frame_buff = 0

            pybullet.setJointMotorControlArray(self.sim.arm.uid, range(7), pybullet.POSITION_CONTROL, targetPositions=robot_pos)
            self.sim.step()

            frame_buff += 1 
            frame_count += 1


def main():
    gui = True
    scene_id = "random"
    vgn_path = "/home/tom/dev_ws/thesis_ws/src/vgn/assets/models/vgn_conv.pth" #was changed 

    env = Environment(gui, scene_id, vgn_path)
    env.load_engine()
    env.get_target()
    env.get_target_bb()
    env.run()


if __name__ == "__main__":
     main()
