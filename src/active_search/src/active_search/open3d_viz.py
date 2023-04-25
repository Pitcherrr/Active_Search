import open3d as o3d
import numpy as np
import threading
import rospy
import copy
import ros_numpy

from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from queue import Queue
from vgn.utils import from_cloud_msg, map_cloud_to_grid

from robot_helpers.spatial import Transform


class Open3d_viz():
    def __init__(self):
        print("init steam --------------------")
        self.init_tsdf_steam()
        
    def init_tsdf_steam(self):
        rospy.wait_for_message('/map_cloud', PointCloud2)
        rospy.Subscriber('/map_cloud', PointCloud2, self.sensor_cb)

    def sensor_cb(self, msg):
        self.tsdf_msg = msg

    def center_view(self, vis):
        vis.reset_view_point(True)


    def kill_o3d(self, vis):
        vis.destroy_window()
        self.o3d_window_active = False
        print("Killing Open3d")
        exit()

    def update(self):
        print("trying to update")
        msg = copy.deepcopy(self.tsdf_msg)
        print(self.tsdf_msg)
        points, distances = from_cloud_msg(msg)
        self.tsdf = map_cloud_to_grid(self.voxel_size, points, distances)


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

        self.update()
        tsdf_init = self.tsdf

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

        vis.add_geometry(tsdf_mesh_init, reset_bounding_box = True)
        # vis.add_geometry(target_bb, reset_bounding_box = reset_bb)
        vis.add_geometry(frame, reset_bounding_box = reset_bb)
        vis.update_renderer()
        vis.remove_geometry(tsdf_mesh_init, reset_bounding_box = reset_bb)

        while self.o3d_window_active:
            if tsdf_exists:
                vis.remove_geometry(tsdf_mesh, reset_bounding_box = reset_bb)
                vis.remove_geometry(bb, reset_bounding_box = reset_bb)
                vis.remove_geometry(target_pc, reset_bounding_box = reset_bb)

            state = self.sim_state.get()

            tsdf_mesh, image = state
            tsdf_mesh = tsdf_init.o3dvol.extract_triangle_mesh()
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

            self.update()

    def run(self):
        self.thread_open3d = threading.Thread(target=self.open3d_window, args= (False,))
        self.thread_open3d.start()