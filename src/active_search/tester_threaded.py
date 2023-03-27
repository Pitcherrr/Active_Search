import pybullet
import pybullet_data
import math
import time
import numpy as np
import threading 
import open3d as o3d
import cv2
from queue import Queue
from scipy.spatial.transform import Rotation


from robot_helpers.bullet import *
from robot_helpers.model import *
from robot_helpers.spatial import Transform
from search_sim import Simulation
#from active_grasp.simulation import Simulation
from vgn.perception import UniformTSDFVolume
from dynamic_perception import DyUniTSDFVolume
from vgn.utils import view_on_sphere

class Environment:
    def __init__(self, gui, scene_id, vgn_path):
        self.gui = gui
        self.scene_id = scene_id
        self.vgn_path = vgn_path

    def load_engine(self):
        self.sim = Simulation(self.gui, self.scene_id, self.vgn_path)
        self.sim_state = Queue()
        self.sim.reset()
        #self.sim.camera = BtCamera(320, 240, 0.96, 0.01, 1.0, self.sim.arm.uid, 11)


    def get_tsdf(self):
        view_loop = False

        origin = Transform.from_translation(self.sim.scene.origin)
        #origin.translation[2] -= 0.05
        center = Transform.from_translation(self.sim.scene.center)

        tsdf = UniformTSDFVolume(self.sim.scene.length, 40)
        r = 2.0 * self.sim.scene.length
        theta = np.pi / 4.0
        phis = np.linspace(0.0, 2.0 * np.pi, 5)

        if view_loop:
            for view in [view_on_sphere(center, r, theta, phi) for phi in phis]:
                depth_img = self.sim.camera.get_image(view)[1]
                tsdf.integrate(depth_img, self.sim.camera.intrinsic, view.inv() * origin)
            voxel_size, tsdf_grid = tsdf.voxel_size, tsdf.get_grid()

        if not view_loop:
            view = [view_on_sphere(center, r, theta, phi) for phi in phis][0]
            cam_data = self.sim.camera.get_image(view)
            image = cam_data[0]
            depth_img = cam_data[1]
            tsdf.integrate(depth_img, self.sim.camera.intrinsic, view.inv() * origin)

            #print(view.inv())
            #print(origin)
            voxel_size, tsdf_grid = tsdf.voxel_size, tsdf.get_grid()

        tsdf_mesh = tsdf.o3dvol.extract_triangle_mesh()

        # flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
        # tsdf_mesh.transform(flip_transform)

        self.sim_state.put([tsdf_mesh, image])

    
    def get_tsdf_2(self):

        tsdf = DyUniTSDFVolume(self.sim.scene.length, 50)

        origin = Transform.from_translation(self.sim.scene.origin)

        # r = pybullet.getLinkState(self.body_uid, self.link_id, computeForwardKinematics=1)
        # cam_pose = Transform(Rotation.from_quat(r[5]), r[4])
        # camera_rot = Transform(Rotation.from_euler('z', 90, degrees=True))
        #print(tsdf.get_grid)
        
        cam_data = self.sim.camera.get_image()
        image = cam_data[0]
        depth_img = cam_data[1]
        intrinsic = self.sim.camera.intrinsic
        tsdf.integrate(depth_img, self.sim.camera.intrinsic, np.identity(4)) 
        #voxel_size, tsdf_grid = tsdf.voxel_size, tsdf.get_grid()

        #tsdf_mesh = tsdf.o3dvol.extract_triangle_mesh()
        tsdf_mesh = tsdf.o3dvol.extract_point_cloud()
        print(tsdf_mesh)
        #tsdf_mesh.scale(0.25, center = tsdf_mesh.get_center())

        self.sim_state.put([tsdf_mesh, image])

    def center_view(self, vis):
        vis.reset_view_point(True)
        ctr = vis.get_view_control()
        ctr.rotate(x = 0, y = 1000)
        # ctr.set_lookat(self.bbox_center)
        # ctr.convert_from_pinhole_camera_parameters(self.camera_params)

        #vis.get_view_control().change_field_of_view(step = 1)

    def kill_o3d(self, vis):
        vis.destroy_window()
        self.o3d_window_active = False


    def open3d_window(self, reset_bb: bool = True):

        """Need to run this in a loop on another thread, 
        may have to parse in camera data from main thread running pybullet"""

        self.o3d_window_active = True

        o3d.core.Device("cuda:0")

        vis = o3d.visualization.VisualizerWithKeyCallback()
    
        #vis = o3d.visualization.Visualizer()

        vis.create_window(window_name = "Depth Camera")

        vis.register_key_callback(ord("C"), self.center_view)
        vis.register_key_callback(ord("X"), self.kill_o3d)

        lines = [
                [0, 1],
                [0, 3],
                [0, 4],
                [1, 2],
                [1, 5],
                [2, 3],
                [2, 6],
                [3, 7],
                [4, 5],
                [4, 7],
                [5, 6],
                [6, 7],
            ]

        while self.sim_state.empty():
             continue
        
        state = self.sim_state.get()
        tsdf_mesh_init, image = state
        #tsdf_mesh_init.compute_vertex_normals()
        #tsdf_mesh_init.compute_triangle_normals()
        vis.add_geometry(tsdf_mesh_init, reset_bounding_box = True)
        vis.update_renderer()

        vis.get_view_control().rotate(x = 0, y = 1000)
        
        vis.remove_geometry(tsdf_mesh_init, reset_bounding_box = reset_bb)
        while self.o3d_window_active:
            if not self.sim_state.empty():

                state = self.sim_state.get()

                tsdf_mesh, image = state

                # Assuming that 'mesh' is the mesh object you want to visualize the bounding box for
                bounding_box = tsdf_mesh.get_axis_aligned_bounding_box()

                # Get the eight corners of the bounding box
                vertices = np.asarray(bounding_box.get_box_points())

                # Rearrange the vertices so that they are in the correct order


                colors = [[1, 0, 0] for _ in range(len(lines))]
                line_set = o3d.geometry.LineSet(
                    points=o3d.utility.Vector3dVector(vertices),
                    lines=o3d.utility.Vector2iVector(lines),
                )
                line_set.colors = o3d.utility.Vector3dVector(colors)

                #print("Image",image)
                #print(tsdf_mesh)

                #tsdf_mesh.compute_vertex_normals()
                #tsdf_mesh.compute_point_cloud_distance(target = tsdf_mesh_init)
                #tsdf_mesh.compute_nearest_neighbor_distance()

                vis.add_geometry(tsdf_mesh, reset_bounding_box = reset_bb)
                vis.add_geometry(line_set, reset_bounding_box = reset_bb)
                #vis.get_render_option().point_color_option
                vis.poll_events()
                vis.update_renderer()
                vis.remove_geometry(tsdf_mesh, reset_bounding_box = reset_bb)
                vis.remove_geometry(line_set, reset_bounding_box = reset_bb)
                    

    def open3d_window_2(self, reset_bb: bool = True):

        """Need to run this in a loop on another thread, 
        may have to parse in camera data from main thread running pybullet"""

        self.o3d_window_active = True

        o3d.core.Device("cuda:0")

        camera_stuff = False 

        if camera_stuff == True:
            while self.sim_state.empty():
                continue
            state = self.sim_state.get()
            tsdf_mesh_init, image = state

            intrinsic = self.sim.camera.intrinsic.to_o3d()

            camera_pose = np.eye(4)
            camera_pose[:3, 3] = np.array([0, 0, -1]) # Move camera 1 unit away from scene
            camera_pose[:3, :3] = np.eye(3) # Set camera orientation

            bbox_center = o3d.geometry.AxisAlignedBoundingBox.get_center(tsdf_mesh_init.get_axis_aligned_bounding_box())
            camera_pose[:3, 3] = bbox_center + np.array([0, 0, 2]) # Move camera 2 units away from center of scene
            
            camera_params = o3d.camera.PinholeCameraParameters()
            camera_params.intrinsic = intrinsic
            camera_params.extrinsic = camera_pose
            
            self.bbox_center = bbox_center
            self.camera_params = camera_params


        vis = o3d.visualization.VisualizerWithKeyCallback()

        vis.create_window(window_name = "Depth Camera")

        vis.register_key_callback(ord("C"), self.center_view)
        vis.register_key_callback(ord("X"), self.kill_o3d)

        while self.sim_state.empty():
             continue
        
        state = self.sim_state.get()
        tsdf_mesh_init, image = state
        tsdf_mesh_init.compute_vertex_normals()
        tsdf_mesh_init.compute_triangle_normals()
        vis.add_geometry(tsdf_mesh_init, reset_bounding_box = False)
        vis.update_renderer()
        
        vis.remove_geometry(tsdf_mesh_init, reset_bounding_box = False)
        while True:
            if not self.sim_state.empty():

                state = self.sim_state.get()

                tsdf_mesh, image = state

                print("Image",image)

                print(tsdf_mesh)

                tsdf_mesh.compute_vertex_normals()
                tsdf_mesh.compute_triangle_normals()

                vis.add_geometry(tsdf_mesh, reset_bounding_box = False)
                vis.poll_events()
                vis.update_renderer()
                vis.remove_geometry(tsdf_mesh, reset_bounding_box = False)

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
        cam_rot_comm = pybullet.addUserDebugParameter("Rotate", 1,0,1)

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

            if frame_buff == 20:
                #self.sim.camera.get_image()
                self.get_tsdf_2()
                frame_buff = 0

            pybullet.setJointMotorControlArray(self.sim.arm.uid, range(7), pybullet.POSITION_CONTROL, targetPositions=robot_pos)
            self.sim.step()

            frame_buff += 1 
            frame_count += 1



def thread_handler(env):

    """Getting issues where thread handler is not adding user control to pybullet thread, open3d thread seems to be working"""
    # Create two threads, one for each window
    thread_open3d = threading.Thread(target=env.open3d_window)
    thread_open3d.start()
    thread_open3d.join()


def main():
    gui = True
    scene_id = "random"
    vgn_path = "/home/tom/dev_ws/thesis_ws/src/vgn/assets/models/vgn_conv.pth" #was changed 

    env = Environment(gui, scene_id, vgn_path)
    env.load_engine()
    env.run()


       

if __name__ == "__main__":
     main()
