import pybullet
import pybullet_data
import math
import time
import numpy as np
import threading 
import open3d as o3d
import cv2
import os
import cProfile
import pstats
import torch
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
        self.sim_state = Queue(maxsize=1)
        self.sim.reset()

    
    def get_tsdf(self):

        tsdf = SceneTSDFVolume(self.sim.scene.length, 50)
        
        cam_data = self.sim.camera.get_image()
        image = cam_data[0]
        depth_img = cam_data[1]

        tsdf.integrate(depth_img, self.sim.camera.intrinsic, (self.sim.camera.pose.inv()*self.scene_origin).as_matrix()) 
        
        self.tsdf = tsdf

        self.targets = self.get_poi_torch()
        self.sim_state.put([self.tsdf, image])


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

        #target_bb = target_bb.get_minimal_oriented_bounding_box()

        print(np.asarray(target_bb.get_box_points()))

        self.target_bb = target_bb

    
    def get_poi_torch(self):

        resolution = 50
        voxel_size = self.sim.scene.length/resolution

        volume = self.tsdf.o3dvol.extract_volume_tsdf()
        vol_array = np.asarray(volume)

        vol_mat = vol_array[:,0].reshape(resolution, resolution, resolution)

        #bb_voxel = np.floor(self.target_bb.get_extent()/voxel_size)
        bb_voxel = [10,10,10]

        # occ_mat = np.zeros_like(vol_mat)
        # tsdf_check = np.zeros_like(vol_mat)
        # print("bb_mat", bb_mat.shape)
        # print("bb_voxel", bb_voxel)
        #print("vol_mat", vol_mat.shape)

        bb_voxel = torch.tensor(bb_voxel)

        vol_mat = torch.from_numpy(vol_mat).to(torch.device("cuda"))
        # occ_mat = torch.from_numpy(occ_mat).to(torch.device("cuda"))
        # tsdf_check = torch.from_numpy(tsdf_check).to(torch.device("cuda"))
        occ_mat = torch.zeros_like(vol_mat, device="cuda")
        tsdf_check = occ_mat
        max_tsdf_slices = occ_mat

        tsdf_slices = vol_mat.unfold(0, int(bb_voxel[0]), 1).unfold(1, int(bb_voxel[1]), 1).unfold(2, int(bb_voxel[2]), 1)
        max_tsdf_slices = tsdf_slices.amax(dim=(3, 4, 5))
        #print(tsdf_slices.shape)
        
        tsdf_check[0:resolution-bb_voxel[0]+1,0:resolution-bb_voxel[1]+1,0:resolution-bb_voxel[2]+1] = max_tsdf_slices <= 0

        #print("tsdf_check",tsdf_check.shape)
 
        occ_mat[0:resolution, 0:resolution, 0:resolution] = tsdf_check.squeeze().to(dtype=torch.uint8)
        # occ_mat[0:bb_voxel[0], 0:bb_voxel[1], 0:bb_voxel[2]] = tsdf_check.squeeze().to(dtype=torch.uint8)

        occ_mat_result = occ_mat.cpu().numpy()

        coordinate_mat = np.argwhere(occ_mat_result > 0)

        poi_mat = np.zeros_like(coordinate_mat)

        # print(coordinate_mat.shape)
        #print(voxel_size)
        #strange offset that is based on the size of the vovel bb
        # poi_mat = coordinate_mat*voxel_size+[(bb_voxel[0]/2)*voxel_size,(bb_voxel[2]/2)*voxel_size,bb_voxel[2]*voxel_size]
        poi_mat = coordinate_mat*voxel_size+[0.009,0.009,bb_voxel[2]*voxel_size]
        # poi_mat = coordinate_mat*voxel_size+[0.009, 0.009, 0.10004142]
   
        self.occ_mat = occ_mat_result
        self.poi_mat = poi_mat

        return occ_mat_result


    def center_view(self, vis):
        vis.reset_view_point(True)


    def kill_o3d(self, vis):
        vis.destroy_window()
        self.o3d_window_active = False
        exit()


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
        
        state = self.sim_state.get()
        tsdf_init, image = state

        tsdf_mesh_init = tsdf_init.o3dvol.extract_point_cloud()

        target_bb = o3d.geometry.OrientedBoundingBox.create_from_axis_aligned_bounding_box(self.target_bb) 
        target_bb.color = [0, 1, 0] 

        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05)
        origin_sphere = mesh = o3d.geometry.TriangleMesh.create_sphere(0.05)
        origin_sphere.transform(Transform.from_translation(self.sim.scene.origin).as_matrix())

        vis.add_geometry(tsdf_mesh_init, reset_bounding_box = True)
        # vis.add_geometry(target_bb, reset_bounding_box = reset_bb)
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
                    vis.remove_geometry(target_pc, reset_bounding_box = reset_bb)

                state = self.sim_state.get()

                tsdf_mesh, image = state

                tsdf_mesh = tsdf_mesh.o3dvol.extract_point_cloud()
                # print(tsdf_mesh)

                tsdf_exists = True

                bb = tsdf_mesh.get_axis_aligned_bounding_box()
                # bb = o3d.geometry.OrientedBoundingBox.create_from_axis_aligned_bounding_box(aligned_bb) 
                bb.color = [1, 0, 0] 

                # print(bb.get_extent())

                target_bb = o3d.geometry.OrientedBoundingBox.create_from_axis_aligned_bounding_box(self.target_bb) 
                target_bb.color = [0, 1, 0] 
                #vis.add_geometry(target_bb, reset_bounding_box = reset_bb)

                vis.add_geometry(tsdf_mesh, reset_bounding_box = reset_bb)
                vis.add_geometry(bb, reset_bounding_box = reset_bb)
                
                if np.amax(bb.get_extent()) > 0:
                    points = o3d.utility.Vector3dVector(self.poi_mat)
                    target_pc = o3d.geometry.PointCloud()
                    target_pc.points = points
                    # target_pc = target_pc.crop(bb)
                    target_pc.paint_uniform_color([0,0,0])

                vis.add_geometry(target_pc, reset_bounding_box = reset_bb)
                #vis.add_geometry(self.targets, reset_bounding_box = reset_bb)

                vis.poll_events()
                vis.update_renderer()


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

        while self.o3d_window_active:

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


def main():
    gui = True
    scene_id = "random"
    vgn_path = "/home/tom/dev_ws/thesis_ws/src/vgn/assets/models/vgn_conv.pth" #was changed 

    env = Environment(gui, scene_id, vgn_path)
    env.load_engine()
    env.get_target()
    env.get_target_bb()
    check_gpu()
    env.run()

def check_gpu():
    print('Cuda Available : {}'.format(torch.cuda.is_available())) 
    if not torch.cuda.is_available():
        raise Exception("You must have a cuda device")
    print('GPU - {0}'.format(torch.cuda.get_device_name())) 


if __name__ == "__main__":
    #  with cProfile.Profile() as profile:
    #     main()
    #     results = pstats.Stats(profile)
    #     results.sort_stats('nfl')
    #     results.print_stats("get_poi")
    main()
