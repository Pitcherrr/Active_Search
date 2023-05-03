import open3d as o3d
import numpy as np
import threading
import rospy
import copy
import ros_numpy
import vgn.srv

from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from queue import Queue
from vgn.utils import from_cloud_msg, map_cloud_to_grid

from robot_helpers.spatial import Transform


class Open3d_viz():
    def __init__(self):
        self.init_tsdf_stream()
        
    def init_tsdf_stream(self):
        print("init stream")
        # rospy.wait_for_message('/map_cloud', PointCloud2)
        # rospy.Subscriber('/map_cloud', PointCloud2, self.sensor_cb)
        rospy.wait_for_service("get_map_cloud")
        # rospy.Service("get_map_cloud", vgn.srv.GetMapCloud, self.get_tsdf)
        tsdf = rospy.ServiceProxy('get_map_cloud', vgn.srv.GetMapCloud)
        self.get_tsdf(tsdf())

    def update(self):
        tsdf = rospy.ServiceProxy('get_map_cloud', vgn.srv.GetMapCloud)
        self.get_tsdf(tsdf())

    def get_tsdf(self, req):
        # print('sensor callback')
        # self.tsdf_msg = req
        # req = copy.deepcopy(self.tsdf_msg)
        self.points, self.distances = from_cloud_msg(req.map_cloud)
        # self.tsdf = map_cloud_to_grid(0.0075, points, distances)

    def center_view(self, vis):
        vis.reset_view_point(True)


    def kill_o3d(self, vis):
        vis.destroy_window()
        self.o3d_window_active = False
        print("Killing Open3d")
        exit()

    def to_pc(self):
        # tsdf_vol = o3d.pipelines.integration.UniformTSDFVolume(
        #     length=0.3,
        #     resolution=40,
        #     sdf_trunc=4*0.0075,
        #     color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor,
        # )
        # tsdf = np.reshape(tsdf,(40*40*40, 2))
        # print(tsdf.shape)
        # tsdf_vec = o3d.utility.Vector2dVector(tsdf)
        # tsdf_vol.inject_volume_tsdf(tsdf_vec)

        point_vec = o3d.utility.Vector3dVector(self.points)
        pc = o3d.geometry.PointCloud(point_vec)

        # v_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pc, 0.0075)

        # distances

        return pc


    def open3d_window(self, reset_bb: bool = True):        
        self.paused = False
        self.o3d_window_active = True
        tsdf_exists = False

        o3d.core.Device("cuda:0")

        vis = o3d.visualization.VisualizerWithKeyCallback()

        vis.create_window(window_name = "Depth Camera")

        #some key call backs for controlling the sim 
        vis.register_key_callback(ord("C"), self.center_view)
        vis.register_key_callback(ord("X"), self.kill_o3d)

        tsdf_init = self.to_pc()

        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05)

        vis.add_geometry(tsdf_init, reset_bounding_box = True)
        vis.add_geometry(frame, reset_bounding_box = reset_bb)
        vis.update_renderer()
        vis.remove_geometry(tsdf_init, reset_bounding_box = reset_bb)

        while self.o3d_window_active:
            if tsdf_exists:
                vis.remove_geometry(tsdf_mesh, reset_bounding_box = reset_bb)
                vis.remove_geometry(bb, reset_bounding_box = reset_bb)

            tsdf_mesh = self.to_pc()

            tsdf_exists = True

            bb = tsdf_mesh.get_axis_aligned_bounding_box()
            bb.color = [1, 0, 0] 

            vis.add_geometry(tsdf_mesh, reset_bounding_box = reset_bb)
            vis.add_geometry(bb, reset_bounding_box = reset_bb)

            vis.poll_events()
            vis.update_renderer()

            self.update()

    def run(self):
        self.thread_open3d = threading.Thread(target=self.open3d_window, args= (False,))
        self.thread_open3d.start()
        # self.open3d_window()