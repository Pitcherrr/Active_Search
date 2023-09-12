import numpy as np
from sensor_msgs.msg import CameraInfo
from pathlib import Path
import rospy
from trac_ik_python.trac_ik import IK
import torch
import open3d as o3d
import rospkg
import os

from robot_helpers.ros import tf
from robot_helpers.ros.conversions import *
from vgn.detection import *
from vgn.perception import UniformTSDFVolume
from robot_helpers.spatial import Transform
# from active_search.dynamic_perception import SceneTSDFVolume

from active_grasp.timer import Timer
from active_grasp.rviz import Visualizer
from active_grasp.bbox import AABBox


def solve_ik(q0, pose, solver):
    x, y, z = pose.translation
    qx, qy, qz, qw = pose.rotation.as_quat()
    return solver.get_ik(q0, x, y, z, qx, qy, qz, qw)


class Policy:
    def __init__(self):
        self.load_parameters()
        self.init_ik_solver()
        self.init_visualizer()
        self.init_tsdf()

    def load_parameters(self):
        self.base_frame = rospy.get_param("~base_frame_id")
        self.T_grasp_ee = Transform.from_list(rospy.get_param("~ee_grasp_offset")).inv()
        self.cam_frame = rospy.get_param("~camera/frame_id")
        self.task_frame = "task"
        info_topic = rospy.get_param("~camera/info_topic")
        msg = rospy.wait_for_message(info_topic, CameraInfo, rospy.Duration(2.0))
        self.intrinsic = from_camera_info_msg(msg)
        self.qual_thresh = rospy.get_param("vgn/qual_threshold")
        self.target_bb = AABBox([0,0,0],[0,0,0])

    def init_ik_solver(self):
        self.q0 = [0.0, -0.79, 0.0, -2.356, 0.0, 1.57, 0.79]
        self.cam_ik_solver = IK(self.base_frame, self.cam_frame)
        self.ee_ik_solver = IK(self.base_frame, "panda_link8")

    def init_tsdf(self):
        self.tsdf = UniformTSDFVolume(0.3, 40)
        rospack = rospkg.RosPack()
        pkg_root = Path(rospack.get_path("active_search"))
        directory_path =  str(pkg_root)+"/training/"
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


    def solve_cam_ik(self, q0, view):
        return solve_ik(q0, view, self.cam_ik_solver)

    def solve_ee_ik(self, q0, pose):
        return solve_ik(q0, pose, self.ee_ik_solver)

    def init_visualizer(self):
        self.vis = Visualizer()

    def activate(self, bbox, view_sphere):
        self.vis.clear()

        self.bbox = bbox
        self.view_sphere = view_sphere

        self.init_task_frame()
        # self.calibrate_task_frame()
        self.vis.bbox(self.base_frame, self.bbox)
        
        self.vgn = VGN(Path(rospy.get_param("vgn/model")))

        self.views = []
        self.best_grasp = None
        self.x_d = None
        self.done = False
        self.info = {}

    def init_data(self):
        self.views = []
        self.best_grasp = None
        self.x_d = None
        self.done = False
        self.info = {}
        self.qual_hist = np.zeros((self.T,) + (40,) * 3, np.float32)

    #I think this is contraining the frame in which the robot can operate
    def calibrate_task_frame(self):
        xyz = np.r_[self.bbox.center[:2] - 0.15, self.bbox.min[2] - 0.05]
        self.T_base_task = Transform.from_translation(xyz)
        self.T_task_base = self.T_base_task.inv()
        tf.broadcast(self.T_base_task, self.base_frame, self.task_frame)
        rospy.sleep(1.0)  # Wait for tf tree to be updated
        self.vis.roi(self.task_frame, 0.3)

    def init_task_frame(self):
        center = np.r_[0.5, 0.0, 0.2]
        length = 0.3
        xyz = center - np.r_[0.5 * length, 0.5 * length, 0.0]
        self.T_base_task = Transform.from_translation(xyz)
        self.T_task_base = self.T_base_task.inv()
        tf.broadcast(self.T_base_task, self.base_frame, self.task_frame)
        rospy.sleep(1.0)  # Wait for tf tree to be updated
        self.vis.roi(self.task_frame, 0.3)


    def update(self, img, x, q):
        raise NotImplementedError

    def filter_grasps(self, out, q):
        grasps, qualities = select_local_maxima(
            self.tsdf.voxel_size,
            out,
            self.qual_thresh,
        )
        filtered_grasps, filtered_qualities = [], []

        bbox_min = self.bbox.min + [0, 0, 3*self.tsdf.voxel_size]
        bbox_max = self.bbox.max
        bbox = AABBox(bbox_min, bbox_max)


        # target_min = np.clip(self.target_bb.min * 0.8, 0, np.inf)
        # target_max = np.clip(self.target_bb.max * 1.2, 0, np.inf)
        target_min = self.target_bb.min
        target_max = self.target_bb.max
        target = AABBox(target_min, target_max)

        self.vis.bbox(self.base_frame, target)
        # print("grasps", grasps, qualities)
        for grasp, quality in zip(grasps, qualities):
            pose = self.T_base_task * grasp.pose
            tip = pose.rotation.apply([0, 0, 0.05]) + pose.translation
            #need to add some padding to botting of bbox as grasps appear there sometimes
            # print(bbox.min)
            if bbox.is_inside(tip) and quality > 0.9:
                grasp.pose = pose
                q_grasp = self.solve_ee_ik(q, pose * self.T_grasp_ee)                
                if q_grasp is not None:
                    filtered_grasps.append(grasp)
                    filtered_qualities.append(quality)
            elif target.is_inside(tip) and quality > 0.9:
                grasp.pose = pose
                q_grasp = self.solve_ee_ik(q, pose * self.T_grasp_ee)
                if q_grasp is not None:
                    print("Found grasp on target")
                    self.done = True
                    filtered_grasps = [grasp]
                    filtered_qualities = [quality]
                    return filtered_grasps, filtered_qualities

        return filtered_grasps, filtered_qualities

    def deactivate(self):
        self.vis.clear_ig_views()


def select_best_grasp(grasps, qualities):
    i = np.argmax(qualities)
    return grasps[i], qualities[i]


class MultiViewPolicy(Policy):
    def __init__(self):
        super().__init__()
        self.T = rospy.get_param("policy/window_size")

    def activate(self, bbox, view_sphere):
        super().activate(bbox, view_sphere)
        self.qual_hist = np.zeros((self.T,) + (40,) * 3, np.float32)

    def integrate(self, img, x, q):
        self.views.append(x)
        self.vis.path(self.base_frame, self.intrinsic, self.views)

        with Timer("tsdf_integration"):
            for _ in range(5):
                self.tsdf.integrate(img, self.intrinsic, x.inv() * self.T_base_task)

        self.get_poi_torch()

        scene_cloud = self.tsdf.get_scene_cloud()
        self.vis.scene_cloud(self.task_frame, np.asarray(scene_cloud.points))

        map_cloud = self.tsdf.get_map_cloud()
        self.vis.map_cloud(
            self.task_frame,
            np.asarray(map_cloud.points),
            np.expand_dims(np.asarray(map_cloud.colors)[:, 0], 1),
        )


    def get_grasps(self, q):
        # print("################### predicting grasps ################################")
        with Timer("grasp_prediction"):
            tsdf_grid = self.tsdf.get_grid()
            out = self.vgn.predict(tsdf_grid)
        self.vis.quality(self.task_frame, self.tsdf.voxel_size, out.qual, 0.9)

        t = (len(self.views) - 1) % self.T
        self.qual_hist[t, ...] = out.qual

        with Timer("grasp_selection"):
            self.grasps, self.qualities = self.filter_grasps(out, q)
            print(len(self.grasps),"grasps above 0.9")

        if len(self.grasps) > 0:
            self.best_grasp, quality = select_best_grasp(self.grasps, self.qualities)
            self.vis.grasps(self.base_frame, self.grasps, self.qualities)
        else:
            self.best_grasp = None
            self.vis.clear_grasp()


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

        self.coordinate_mat = np.argwhere(occ_mat_result > 0)

        coordinate_mat_set = set(map(tuple, self.coordinate_mat)) 
        # print(coordinate_mat_set)

        poi_mat = np.zeros_like(self.coordinate_mat)

        poi_mat = self.coordinate_mat*voxel_size+[0.009+round(bb_voxel[0]/2)*voxel_size,0.009+round(bb_voxel[1]/2)*voxel_size,round(bb_voxel[2]/2)*voxel_size]
        
        self.vis.target_locations(self.base_frame, poi_mat+[0.35,-0.15,0.2])

        # print(coordinate_mat)
        store_pc = False

        if store_pc:
            rospack = rospkg.RosPack()
            pkg_root = Path(rospack.get_path("active_search"))
            file_dir_occ = str(pkg_root)+"/training/p"+str(self.point_index)+"_occu.pcd"
            file_dir_tsdf = str(pkg_root)+"/training/p"+str(self.point_index)+"_tsdf.pcd"
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

        self.coord_set = coordinate_mat_set
        self.occ_mat = occ_mat_result 
        self.poi_mat = poi_mat


    def tsdf_cut(self, bb):
        min_bound = np.floor(np.asarray(bb.min) / self.tsdf.voxel_size) - self.tsdf.sdf_trunc/self.tsdf.voxel_size #+ [0,0,6]
        min_bound = np.clip(min_bound,0,np.inf).astype(int)
        max_bound = np.ceil(np.asarray(bb.max) / self.tsdf.voxel_size) + self.tsdf.sdf_trunc/self.tsdf.voxel_size #- [0,0,6}
        max_bound = np.clip(max_bound,0,np.inf).astype(int)

        x_mask = np.logical_and(self.coordinate_mat[:,0] >= min_bound[0], self.coordinate_mat[:,0] <= max_bound[0])
        y_mask = np.logical_and(self.coordinate_mat[:,1] >= min_bound[1], self.coordinate_mat[:,1] <= max_bound[1])
        z_mask = np.logical_and(self.coordinate_mat[:,2] >= min_bound[2], self.coordinate_mat[:,2] <= max_bound[2])

        points_within_box = np.logical_and(np.logical_and(x_mask, y_mask), z_mask)

        bb_vis = bb
        bb_vis.min += [0.35,-0.15,0.2]
        bb_vis.max += [0.35,-0.15,0.2]
        self.vis.bbox(self.base_frame, bb_vis)
        tsdf_vec = np.asarray(self.tsdf.o3dvol.extract_volume_tsdf())
        tsdf_grid = np.reshape(tsdf_vec, [40,40,40,2])
        # print(min_bound, max_bound)
        tsdf_grid[min_bound[0]:max_bound[0], min_bound[1]:max_bound[1], 0:max_bound[2]] = 0
        tsdf_vec = o3d.utility.Vector2dVector(np.reshape(tsdf_grid, [40*40*40,2]))
        self.tsdf = UniformTSDFVolume(0.3, 40)
        self.tsdf.o3dvol.inject_volume_tsdf(tsdf_vec)
        #update rviz
        scene_cloud = self.tsdf.get_scene_cloud()
        self.vis.scene_cloud(self.task_frame, np.asarray(scene_cloud.points))

        # return np.prod(max_bound - min_bound)
        return np.sum(points_within_box)




def compute_error(x_d, x):
    linear = x_d.translation - x.translation
    angular = (x_d.rotation * x.rotation.inv()).as_rotvec()
    return linear, angular


registry = {}


def register(id, cls):
    global registry
    registry[id] = cls


def make(id, *args, **kwargs):
    if id in registry:
        return registry[id](*args, **kwargs)
    else:
        raise ValueError("{} policy does not exist.".format(id))
