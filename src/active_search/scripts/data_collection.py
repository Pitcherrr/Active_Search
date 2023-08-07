import pybullet
import numpy as np
import threading 
import open3d as o3d
import cv2
import rospkg
import os
import torch
from queue import Queue
from trac_ik_python.trac_ik import IK
from pathlib import Path

from active_search.bullet_utils import *
from robot_helpers.model import *
from robot_helpers.spatial import Transform
from active_search.search_sim import Simulation
from active_search.dynamic_perception import SceneTSDFVolume
# from vgn.perception import UniformTSDFVolume
from vgn.detection import VGN, select_local_maxima, to_voxel_coordinates

class Environment:
    def __init__(self, gui, scene_id, vgn_path):
        self.gui = gui
        self.scene_id = scene_id
        self.vgn_path = vgn_path

    def load_engine(self):
        self.sim = Simulation(self.gui, self.scene_id, self.vgn_path)
        self.load_environment()

    def load_environment(self):
        self.sim.reset()
        self.scene_origin = Transform.from_translation(self.sim.scene.alt_origin)
        self.sim_state = Queue(maxsize=1)
        # self.tsdf = SceneTSDFVolume(self.sim.scene.length, 40)
        self.init_tsdf()
        self.reset_tsdf = False
        self.save_scene = False

    def init_tsdf(self):
        self.tsdf = SceneTSDFVolume(self.sim.scene.length, 40)
        rospack = rospkg.RosPack()
        pkg_root = Path(rospack.get_path("active_search"))
        directory_path =  str(pkg_root)+"/training_test/"
        files = os.listdir(directory_path)
        matching_files = [file for file in files if file.startswith("p")]

        if matching_files:
            def extract_number(filename):
                return int(filename[1:-9])  # Extract the numeric part, excluding the 'p' prefix and '.pcd' extension

            sorted_files = sorted(matching_files, key=extract_number, reverse=True)
            latest_file = sorted_files[0]
            print("Latest file:", latest_file)
            self.point_index = extract_number(latest_file) + 1
        else:
            self.point_index = 0

    def get_tsdf(self):
        cam_data = self.sim.camera.get_image()
        image = cam_data[0]
        depth_img = cam_data[1]

        if self.remove_rand_obj:
            object_bb = self.get_object_bbox(self.sim.object_uids)
            rand_bb = np.random.choice(object_bb)
            print(self.sim.scene.object_uids)
            print(object_bb.index(rand_bb))
            self.sim.scene.remove_object(self.sim.scene.object_uids[object_bb.index(rand_bb)])
            object_bb.remove(rand_bb)
            min_bound = np.floor(np.asarray(rand_bb.min_bound) / self.tsdf.voxel_size) - [4,4,4]
            min_bound = np.clip(min_bound,0,np.inf).astype(int)
            max_bound = np.ceil(np.asarray(rand_bb.max_bound) / self.tsdf.voxel_size) + [4,4,4]
            max_bound = np.clip(max_bound,0,np.inf).astype(int)
            print(min_bound, max_bound)
            tsdf_vec = np.asarray(self.tsdf.o3dvol.extract_volume_tsdf())
            print(tsdf_vec)
            tsdf_grid = np.reshape(tsdf_vec, [40,40,40,2])
            print(tsdf_grid.shape)
            tsdf_grid[min_bound[0]:max_bound[0], min_bound[1]:max_bound[1], min_bound[2]:max_bound[2]] = [0,0]
            tsdf_vec = o3d.utility.Vector2dVector(np.reshape(tsdf_grid, [40*40*40,2]))
            # self.tsdf.o3dvol.inject_volume_tsdf(tsdf_vec)
            self.tsdf.o3dvol.inject_volume_tsdf(tsdf_vec)
            self.remove_rand_obj = False

        self.tsdf.integrate(depth_img, self.sim.camera.intrinsic, (self.sim.camera.pose.inv()*self.scene_origin).as_matrix()) 

        self.get_poi_torch()

        self.sim_state.put([self.tsdf, image])

        if self.save_scene:
            self.save_tsdfs()
    

    def save_tsdfs(self):

        rospack = rospkg.RosPack()
        pkg_root = Path(rospack.get_path("active_search"))
        file_dir_occ = str(pkg_root)+"/training_test/p"+str(self.point_index)+"_occu.pcd"
        file_dir_tsdf = str(pkg_root)+"/training_test/p"+str(self.point_index)+"_tsdf.pcd"
        print(file_dir_occ)
        print(file_dir_tsdf)
        coord_o3d = o3d.utility.Vector3dVector(self.coordinate_mat)
        poi_mat_o3d = o3d.geometry.PointCloud(points = coord_o3d)
        write_occ = o3d.io.write_point_cloud(file_dir_occ, poi_mat_o3d, write_ascii=False, compressed=False, print_progress=False)
        write_tsdf = o3d.io.write_point_cloud(file_dir_tsdf, self.tsdf.get_map_cloud(), write_ascii=False, compressed=False, print_progress=False)
        print(write_occ, write_tsdf)
        #only increment if we have created a set
        if write_occ and write_tsdf:
            self.point_index += 1

    
    def get_target(self):
        self.target_uid = np.random.choice(self.sim.object_uids)
        
    def get_target_bb(self):
        target_bb =  o3d.geometry.AxisAlignedBoundingBox()
 
        min_bound, max_bound = pybullet.getAABB(self.target_uid)
        
        origin = Transform.from_translation(self.sim.scene.origin)

        min_bound_t = (Transform.from_translation(min_bound)*origin.inv()).translation
        max_bound_t = (Transform.from_translation(max_bound)*origin.inv()).translation

        target_bb.min_bound = min_bound_t + np.array([0,0,0.1])
        target_bb.max_bound = max_bound_t + np.array([0,0,0.1])

        target_bb.scale(0.8, target_bb.get_center())

        self.target_bb = target_bb

    
    def get_poi_torch(self):

        resolution = self.tsdf.resolution
        voxel_size = self.tsdf.voxel_size

        volume = self.tsdf.o3dvol.extract_volume_tsdf()
        vol_array = np.asarray(volume)

        vol_mat = vol_array[:,0].reshape(resolution, resolution, resolution)

        #bb_voxel = np.floor(self.target_bb.get_extent()/voxel_size)
        bb_voxel = [5,5,5]

        vol_mat = torch.from_numpy(vol_mat).to(torch.device("cuda"))

        occ_mat = torch.zeros_like(vol_mat, device="cuda")
        tsdf_check = occ_mat
        max_tsdf_slices = occ_mat

        tsdf_slices = vol_mat.unfold(0, int(bb_voxel[0]), 1).unfold(1, int(bb_voxel[1]), 1).unfold(2, int(bb_voxel[2]), 1)
        # max_tsdf_slices[0:resolution-bb_voxel[0]+1,0:resolution-bb_voxel[1]+1,0:resolution-bb_voxel[2]+1]  = tsdf_slices.amax(dim=(3, 4, 5))
        max_tsdf_slices = tsdf_slices.amax(dim=(3, 4, 5))
        # print(max_tsdf_slices.shape)

        tsdf_check[0:resolution-bb_voxel[0]+1,0:resolution-bb_voxel[1]+1,0:resolution-bb_voxel[2]+1] = max_tsdf_slices <= 0.5

 
        occ_mat[0:resolution, 0:resolution, 0:resolution] = tsdf_check.squeeze().to(dtype=torch.uint8)

        # print(occ_mat.shape)

        #as each occluded voxel currently represents a location of the center of the target object we are pooling half the object size around its center point to fill in the missing voxels. 

        pooling = torch.nn.MaxPool3d(kernel_size=(3,3,3),stride=(1,1,1))

        # occ_mat = pooling(occ_mat)

        occ_mat = pooling(occ_mat.unsqueeze(0).unsqueeze(0)).squeeze()

        occ_mat_result = occ_mat.cpu().numpy()

        coordinate_mat = np.argwhere(occ_mat_result > 0)

        poi_mat = np.zeros_like(coordinate_mat)

        poi_mat = coordinate_mat*voxel_size+[0.009+round(bb_voxel[0]/2)*voxel_size,0.009+round(bb_voxel[1]/2)*voxel_size,round(bb_voxel[2]/2)*voxel_size]

        # self.coord_set = coordinate_mat_set
        self.coordinate_mat = coordinate_mat
        self.occ_mat = occ_mat_result 
        self.poi_mat = poi_mat

        return occ_mat_result
    
    def get_object_bbox(self, uids):
        object_bbs = []

        for uid in uids:
            bb =  o3d.geometry.AxisAlignedBoundingBox()
            min_bound, max_bound = pybullet.getAABB(uid)
            origin = Transform.from_translation(self.sim.scene.origin)
            min_bound_t = (Transform.from_translation(min_bound)*origin.inv()).translation
            max_bound_t = (Transform.from_translation(max_bound)*origin.inv()).translation
            bb.min_bound = min_bound_t + np.array([0,0,0.0])
            bb.max_bound = max_bound_t + np.array([0,0,0.0])
            object_bbs.append(bb)
            # target_bb.scale(0.8, target_bb.get_center())
        return object_bbs
    
    def get_object_bbox_bullet(self, uids):
        object_bbs = []
        for uid in uids:
            bb = self.sim.get_target_bbox(uid)
            object_bbs.append(bb)
        return object_bbs

    def center_view(self, vis):
        vis.reset_view_point(True)

    def remove_obj_tsdf(self, vis):
        self.remove_rand_obj = True

    def kill_o3d(self, vis):
        vis.destroy_window()
        self.o3d_window_active = False
        print("Killing Open3d")
        exit()

    def open3d_window(self, reset_bb: bool = True):        
        self.paused = False
        self.o3d_window_active = True
        self.remove_rand_obj = False
        tsdf_exists = False

        o3d.core.Device("cuda:0")

        vis = o3d.visualization.VisualizerWithKeyCallback()

        vis.create_window(window_name = "Depth Camera")

        #some key call backs for controlling the sim 
        vis.register_key_callback(ord("C"), self.center_view)
        vis.register_key_callback(ord("X"), self.kill_o3d)
        vis.register_key_callback(ord("R"), self.remove_obj_tsdf)

        while self.sim_state.empty():
             continue
        
        state = self.sim_state.get()
        tsdf_init, image = state

        tsdf_mesh_init = tsdf_init.get_map_cloud()

        
        target_bb = o3d.geometry.OrientedBoundingBox.create_from_axis_aligned_bounding_box(self.target_bb) 
        target_bb.color = [0, 1, 0] 

        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05)
        origin_sphere = o3d.geometry.TriangleMesh.create_sphere(0.05)
        origin_sphere.transform(Transform.from_translation(self.sim.scene.origin).as_matrix())

        object_bb = self.get_object_bbox(self.sim.object_uids)
        for objects in object_bb:
            objects.color = [0, 0, 1] 
            vis.add_geometry(objects)
        
        total_space = o3d.geometry.AxisAlignedBoundingBox()
        total_space.min_bound = [0,0,0]
        total_space.max_bound = [0.3,0.3,0.3]
        total_space.color = [0,0,0]

        vis.add_geometry(total_space)
        vis.add_geometry(tsdf_mesh_init, reset_bounding_box = True)
        # vis.add_geometry(target_bb, reset_bounding_box = reset_bb)
        vis.add_geometry(frame, reset_bounding_box = reset_bb)
        vis.update_renderer()
        vis.remove_geometry(tsdf_mesh_init, reset_bounding_box = reset_bb)

        while self.o3d_window_active:
            vis.poll_events()
            vis.update_renderer()
            if not self.sim_state.empty():

                if tsdf_exists:
                    vis.remove_geometry(tsdf_mesh, reset_bounding_box = reset_bb)
                    vis.remove_geometry(bb, reset_bounding_box = reset_bb)
                    vis.remove_geometry(target_pc, reset_bounding_box = reset_bb)
                
                # if self.remove_rand_obj:
                #     state = self.sim_state.get()
                #     tsdf, image = state
                #     rand_bb = np.random.choice(object_bb)
                #     print(self.sim.scene.object_uids)
                #     print(object_bb.index(rand_bb))
                #     self.sim.scene.remove_object(self.sim.scene.object_uids[object_bb.index(rand_bb)])
                #     object_bb.remove(rand_bb)
                #     min_bound = np.floor(np.asarray(rand_bb.min_bound) / self.tsdf.voxel_size) - [4,4,4]
                #     min_bound = np.clip(min_bound,0,np.inf).astype(int)
                #     max_bound = np.ceil(np.asarray(rand_bb.max_bound) / self.tsdf.voxel_size) + [4,4,4]
                #     max_bound = np.clip(max_bound,0,np.inf).astype(int)
                #     # tsdf_grid = tsdf.get_grid()
                #     print(min_bound, max_bound)
                #     tsdf_vec = np.asarray(tsdf.o3dvol.extract_volume_tsdf())
                #     tsdf_grid = np.reshape(tsdf_vec, [40,40,40,2])
                #     print(tsdf_grid.shape)
                #     tsdf_grid[min_bound[0]:max_bound[0], min_bound[1]:max_bound[1], min_bound[2]:max_bound[2]] = 0
                #     tsdf_vec = o3d.utility.Vector2dVector(np.reshape(tsdf_grid, [40*40*40,2]))
                #     # self.tsdf.o3dvol.inject_volume_tsdf(tsdf_vec)
                #     tsdf.o3dvol.inject_volume_tsdf(tsdf_vec)
                #     self.tsdf = tsdf
                #     self.remove_rand_obj = False
                # else:
                state = self.sim_state.get()
                tsdf, image = state

                tsdf_mesh = tsdf.get_map_cloud()

                tsdf_exists = True

                bb = tsdf_mesh.get_axis_aligned_bounding_box()
                bb.color = [1, 0, 0] 

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
                    target_pc.paint_uniform_color([0,1,0])

                vis.add_geometry(target_pc, reset_bounding_box = reset_bb)
                #vis.add_geometry(self.targets, reset_bounding_box = reset_bb)

                vis.poll_events()
                vis.update_renderer()


    def run(self):

        # Create two threads, one for each window
        # self.thread_live_feed = threading.Thread(target=self.live_feed)
        # self.thread_live_feed.start()
        self.thread_open3d = threading.Thread(target=self.open3d_window, args=(False,))
        #self.thread_open3d = threading.Thread(target=self.full_scene, args= (False,))
        self.thread_open3d.start()

        [j1_init, j2_init, j3_init, j4_init, j5_init, j6_init, j7_init] = [j[0] for j in pybullet.getJointStates(self.sim.arm.uid, range(7))]

        frame_buff = 0  
        tsdf_buff = 0
        done = False

        # while self.o3d_window_active:
        while not done:
            print(frame_buff)

            if tsdf_buff == 10:
                self.get_tsdf()
                tsdf_buff = 0

            if frame_buff == 50:
                self.get_tsdf()
                print("In buffer")
                if len(self.sim.object_uids) > 0:
                    self.remove_rand_obj = True
                else:
                    self.load_environment()
                    print("finished loading")
                frame_buff = 0

            # pybullet.setJointMotorControlArray(self.sim.arm.uid, range(7), pybullet.POSITION_CONTROL, targetPositions=robot_pos)
            self.sim.step()

            frame_buff += 1 
            tsdf_buff += 1

    def init_ik_solver(self):
        self.q0 = self.sim.arm.configurations["ready"]
        self.cam_ik_solver = IK(self.sim.arm.base_frame, "camera_depth_optical_frame")
        self.ee_ik_solver = IK(self.sim.arm.base_frame, "panda_link8")


def solve_ik(q0, pose, solver):
    x, y, z = pose.translation
    qx, qy, qz, qw = pose.rotation.as_quat()
    return solver.get_ik(q0, x, y, z, qx, qy, qz, qw)


def main():
    gui = True
    scene_id = "random"
    # scene_id = "as_test_scene.yaml"
    vgn_path = "src/vgn/assets/models/vgn_conv.pth" #was changed 

    env = Environment(gui, scene_id, vgn_path)
    env.load_engine()
    # env.init_ik_solver()
    env.get_target()
    env.get_target_bb()
    check_gpu()

    for _ in range(100):
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
