import pybullet
import numpy as np
import threading 
import open3d as o3d
import cv2
import cProfile
import pstats
import torch
import torch.nn.functional as F
from queue import Queue
from trac_ik_python.trac_ik import IK

from active_search.bullet_utils import *
from robot_helpers.model import *
from robot_helpers.spatial import Transform
from active_search.search_sim import Simulation
from active_search.dynamic_perception import SceneTSDFVolume
from vgn.detection import VGN, select_local_maxima, to_voxel_coordinates

class Environment:
    def __init__(self, gui, scene_id, vgn_path):
        self.gui = gui
        self.scene_id = scene_id
        self.vgn_path = vgn_path

    def load_engine(self):
        self.sim = Simulation(self.gui, self.scene_id, self.vgn_path)
        self.sim.reset()
        self.scene_origin = Transform.from_translation(self.sim.scene.alt_origin)
        self.sim_state = Queue(maxsize=1)
        self.tsdf = SceneTSDFVolume(self.sim.scene.length, 40)
        self.reset_tsdf = False

    def get_tsdf(self):
            
        if self.reset_tsdf:
            self.tsdf = SceneTSDFVolume(self.sim.scene.length, 40)
        
        cam_data = self.sim.camera.get_image()
        image = cam_data[0]
        depth_img = cam_data[1]

        self.tsdf.integrate(depth_img, self.sim.camera.intrinsic, (self.sim.camera.pose.inv()*self.scene_origin).as_matrix()) 

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

        # print(np.asarray(target_bb.get_box_points()))

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



        occ_mat_result = occ_mat.cpu().numpy()

        # ones = np.ones((5, 5, 5), dtype=int)
        # indices = np.argwhere(occ_mat_result == 1)
        # i, j, k = np.ogrid[:5, :5, :5]
        # box_indices = np.array([i, j, k], dtype=object)
        # box_indices = (indices[:, np.newaxis, np.newaxis, np.newaxis, :] + box_indices)
        # # Convert the box_indices array to an integer array
        # box_indices = box_indices.reshape(-1, 3).astype(int)

        # # Use array slicing and broadcasting to set all elements in each box to 1
        # occ_mat_result[tuple(box_indices.T)] = ones.ravel()


        coordinate_mat = np.argwhere(occ_mat_result > 0)
        # coordinate_mat_set = set(coordinate_mat)

        poi_mat = np.zeros_like(coordinate_mat)

        poi_mat = coordinate_mat*voxel_size+[0.009+round(bb_voxel[0]/2)*voxel_size,0.009+round(bb_voxel[1]/2)*voxel_size,round(bb_voxel[2]/2)*voxel_size]

        # self.coord_set = coordinate_mat_set
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
            bb.min_bound = min_bound_t + np.array([0,0,0.1])
            bb.max_bound = max_bound_t + np.array([0,0,0.1])
            object_bbs.append(bb)
            # target_bb.scale(0.8, target_bb.get_center())
        return object_bbs
    
    def get_object_bbox_bullet(self, uids):
        object_bbs = []
        for uid in uids:
            bb = self.sim.get_target_bbox(uid)
            object_bbs.append(bb)
        return object_bbs


    # def load_vgn(self, model_path):
    #     self.vgn = VGN(model_path)

    def check_for_grasps(self, bbox):
        origin = Transform.from_translation(self.sim.scene.origin)
        origin.translation[2] -= 0.05

        voxel_size, tsdf_grid = self.tsdf.voxel_size, self.tsdf.get_grid()

        # Then check whether VGN can find any grasps on the target
        out = self.sim.vgn.predict(tsdf_grid)
        grasps, qualities = select_local_maxima(voxel_size, out, threshold=0.8)

        for grasp in grasps:
            pose = origin * grasp.pose
            tip = pose.rotation.apply([0, 0, 0.05]) + pose.translation
            if bbox.is_inside(tip):
                return (True, grasp)
        return (False,)
    

    def center_view(self, vis):
        vis.reset_view_point(True)

    def remove_obj_tsdf(self, vis):
        self.remove_rand_obj = True

    def kill_o3d(self, vis):
        vis.destroy_window()
        self.o3d_window_active = False
        print("Killing Open3d")
        exit()

    def set_grasp(self, vis):
        self.do_grasp_check = True
        print("Checking Grasps...")


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
        vis.register_key_callback(ord("G"), self.set_grasp)
        vis.register_key_callback(ord("R"), self.remove_obj_tsdf)

        while self.sim_state.empty():
             continue
        
        state = self.sim_state.get()
        tsdf_init, image = state

        # tsdf_mesh_init = tsdf_init.o3dvol.extract_point_cloud()
        tsdf_mesh_init = tsdf_init.o3dvol.extract_triangle_mesh()

        tsdf_mesh_init.compute_triangle_normals()
        tsdf_mesh_init.compute_vertex_normals()

        target_bb = o3d.geometry.OrientedBoundingBox.create_from_axis_aligned_bounding_box(self.target_bb) 
        target_bb.color = [0, 1, 0] 

        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05)
        origin_sphere = mesh = o3d.geometry.TriangleMesh.create_sphere(0.05)
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
                
                if self.remove_rand_obj:
                    state = self.sim_state.get()
                    tsdf, image = state
                    rand_bb = np.random.choice(object_bb)
                    print(self.sim.scene.object_uids)
                    print(object_bb.index(rand_bb))
                    self.sim.scene.remove_object(self.sim.scene.object_uids[object_bb.index(rand_bb)])
                    object_bb.remove(rand_bb)
                    min_bound = np.floor(np.asarray(rand_bb.min_bound) / self.tsdf.voxel_size).astype(int)
                    max_bound = np.ceil(np.asarray(rand_bb.max_bound) / self.tsdf.voxel_size).astype(int)
                    # tsdf_grid = tsdf.get_grid()
                    print(min_bound, max_bound)
                    tsdf_vec = np.asarray(tsdf.o3dvol.extract_volume_tsdf())
                    tsdf_grid = np.reshape(tsdf_vec, [40,40,40,2])
                    print(tsdf_grid.shape)
                    tsdf_grid[min_bound[0]:max_bound[0], min_bound[1]:max_bound[1], min_bound[2]:max_bound[2]] = 0
                    tsdf_vec = o3d.utility.Vector2dVector(np.reshape(tsdf_grid, [40*40*40,2]))
                    tsdf = tsdf.o3dvol.inject_volume_tsdf(tsdf_vec)
                    self.remove_rand_obj = False
                else:
                    state = self.sim_state.get()
                    tsdf, image = state

                state = self.sim_state.get()
                tsdf, image = state
                tsdf_mesh = tsdf.o3dvol.extract_triangle_mesh()
                tsdf_mesh.compute_triangle_normals()
                tsdf_mesh.compute_vertex_normals()

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
                    target_pc = target_pc.crop(bb)
                    target_pc.paint_uniform_color([0,1,0])

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

        self.do_grasp_check = False

        # Create two threads, one for each window
        # self.thread_live_feed = threading.Thread(target=self.live_feed)
        # self.thread_live_feed.start()
        self.thread_open3d = threading.Thread(target=self.open3d_window, args= (False,))
        #self.thread_open3d = threading.Thread(target=self.full_scene, args= (False,))
        self.thread_open3d.start()


        [j1_init, j2_init, j3_init, j4_init, j5_init, j6_init, j7_init] = [j[0] for j in pybullet.getJointStates(self.sim.arm.uid, range(7))]

        #set up user inputs 
        j1_comm = pybullet.addUserDebugParameter("J1", -np.pi, np.pi, 0)
        j2_comm = pybullet.addUserDebugParameter("J2", -np.pi, np.pi, 0)
        j3_comm = pybullet.addUserDebugParameter("J3", -np.pi, np.pi, 0)
        j4_comm = pybullet.addUserDebugParameter("J4", -np.pi, np.pi, 0)    
        j5_comm = pybullet.addUserDebugParameter("J5", -np.pi, np.pi, 0)
        j6_comm = pybullet.addUserDebugParameter("J6", -np.pi, np.pi, 0)
        j7_comm = pybullet.addUserDebugParameter("J7", -np.pi, np.pi, 0)
        grip_comm = pybullet.addUserDebugParameter("Grip", 0,0.1,0)
        # cam_rot_comm = pybullet.addUserDebugParameter("Rotate", 1,0,1)
        cam_rot_comm = pybullet.addUserDebugParameter("Rotate", 0, 2*np.pi, 0)

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

            if self.do_grasp_check:
                objs = self.get_object_bbox_bullet(self.sim.object_uids)
                for obj in objs:
                    grasp = self.check_for_grasps(obj)
                    print(f"Grasp detected on object {objs.index(obj)}: {grasp[0]}")
                    if grasp[0]:
                        grasp_coord = to_voxel_coordinates(self.tsdf.voxel_size, grasp[1])
                        print(grasp_coord.pose.translation)
                self.do_grasp_check = False


            if frame_buff == 10:
                self.get_tsdf()
                frame_buff = 0

            pybullet.setJointMotorControlArray(self.sim.arm.uid, range(7), pybullet.POSITION_CONTROL, targetPositions=robot_pos)
            self.sim.step()

            frame_buff += 1 

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
    # scene_id = "random"
    scene_id = "as_test_scene.yaml"
    vgn_path = "src/vgn/assets/models/vgn_conv.pth" #was changed 

    env = Environment(gui, scene_id, vgn_path)
    env.load_engine()
    # env.init_ik_solver()
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
