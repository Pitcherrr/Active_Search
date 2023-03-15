import pybullet
import pybullet_data
import math
import numpy as np
import threading 
import open3d as o3d

from robot_helpers.bullet import *
from queue import Queue
from robot_helpers.model import *
from search_sim import Simulation
#from active_grasp.simulation import Simulation
from vgn.perception import UniformTSDFVolume
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
        self.sim.camera = BtCamera(320, 240, 0.96, 0.01, 1.0, self.sim.arm.uid, 11)


    def get_tsdf(self):
        view_loop = False

        origin = Transform.from_translation(self.sim.scene.origin)
        origin.translation[2] -= 0.05
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
            voxel_size, tsdf_grid = tsdf.voxel_size, tsdf.get_grid()

        tsdf_mesh = tsdf.o3dvol.extract_triangle_mesh()

        self.sim_state.put([tsdf_mesh, image])

    
    def get_tsdf_2(self):
        view_loop = False

        origin = Transform.from_translation(self.sim.scene.origin)
        origin.translation[2] -= 0.05
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
            cam_data = self.sim.camera.get_image()
            image = cam_data[0]
            depth_img = cam_data[1]
            tsdf.integrate(depth_img, self.sim.camera.intrinsic, view.inv() * origin)
            voxel_size, tsdf_grid = tsdf.voxel_size, tsdf.get_grid()

        tsdf_mesh = tsdf.o3dvol.extract_triangle_mesh()

        self.sim_state.put([tsdf_mesh, image])

    def open3d_window(self):

        """Need to run this in a loop on another thread, 
        may have to parse in camera data from main thread running pybullet"""

        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name = "Depth Camera")

        while self.sim_state.empty():
             continue
        
        state = self.sim_state.get()
        tsdf_mesh_init, image = state
        tsdf_mesh_init.compute_vertex_normals()
        tsdf_mesh_init.compute_triangle_normals()
        vis.add_geometry(tsdf_mesh_init, reset_bounding_box = True)
        vis.update_renderer()
        
        vis.remove_geometry(tsdf_mesh_init, reset_bounding_box = False)
        while True:
                if not self.sim_state.empty():

                    state = self.sim_state.get()

                    tsdf_mesh, image = state

                    tsdf_mesh.compute_vertex_normals()
                    tsdf_mesh.compute_triangle_normals()

                    vis.add_geometry(tsdf_mesh, reset_bounding_box = False)
                    vis.poll_events()
                    vis.update_renderer()
                    vis.remove_geometry(tsdf_mesh, reset_bounding_box = False)
                    

    def run(self):

            # Create two threads, one for each window
            self.thread_open3d = threading.Thread(target=self.open3d_window)
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
                grip_width = pybullet.readUserDebugParameter(grip_comm)


                robot_pos = [j1, j2, j3, j4, j5, j6, j7]
                self.sim.gripper.set_desired_width(grip_width)

                if frame_buff == 100:
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
