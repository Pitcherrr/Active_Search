import pybullet
import pybullet_data
import math
import numpy as np
import threading 
import open3d as o3d

from bullet import *
from robot_helpers.model import *
from active_grasp.simulation import Simulation
from vgn.perception import UniformTSDFVolume
from vgn.utils import view_on_sphere

class Environment:
    def __init__(self, gui, scene_id, vgn_path):
        self.gui = gui
        self.scene_id = scene_id
        self.vgn_path = vgn_path

    def load_engine(self):
        self.sim = Simulation(self.gui, self.scene_id, self.vgn_path)

        self.sim.reset()


    def get_tsdf(self):
        origin = Transform.from_translation(self.sim.scene.origin)
        origin.translation[2] -= 0.05
        center = Transform.from_translation(self.sim.scene.center)

        tsdf = UniformTSDFVolume(self.sim.scene.length, 40)
        r = 2.0 * self.sim.scene.length
        theta = np.pi / 4.0
        phis = np.linspace(0.0, 2.0 * np.pi, 5)
        for view in [view_on_sphere(center, r, theta, phi) for phi in phis]:
            depth_img = self.sim.camera.get_image(view)[1]
            tsdf.integrate(depth_img, self.sim.camera.intrinsic, view.inv() * origin)
        voxel_size, tsdf_grid = tsdf.voxel_size, tsdf.get_grid()

        tsdf_mesh = tsdf.o3dvol.extract_triangle_mesh()

        return tsdf_mesh


def pybullet_window(sim):

        [j1_init, j2_init, j3_init, j4_init, j5_init, j6_init, j7_init] = [j[0] for j in pybullet.getJointStates(sim.arm.uid, range(7))]
        #set up user inputs 
        j1_comm = pybullet.addUserDebugParameter("J1", -math.pi, math.pi, 0)
        j2_comm = pybullet.addUserDebugParameter("J2", -math.pi, math.pi, 0)
        j3_comm = pybullet.addUserDebugParameter("J3", -math.pi, math.pi, 0)
        j4_comm = pybullet.addUserDebugParameter("J4", -math.pi, math.pi, 0)    
        j5_comm = pybullet.addUserDebugParameter("J5", -math.pi, math.pi, 0)
        j6_comm = pybullet.addUserDebugParameter("J6", -math.pi, math.pi, 0)
        j7_comm = pybullet.addUserDebugParameter("J7", -math.pi, math.pi, 0)
        grip_comm = pybullet.addUserDebugParameter("Grip", 0,0.1,0)

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
            sim.gripper.set_desired_width(grip_width)

            if frame_buff == 20:
                sim.camera.get_image()
                frame_buff = 0

            pybullet.setJointMotorControlArray(sim.arm.uid, range(7), pybullet.POSITION_CONTROL, targetPositions=robot_pos)
            sim.step()

            frame_buff += 1 
            frame_count += 1

def open3d_window(sim, tsdf_mesh):

    """Need to run this in a loop on another thread, 
    may have to parse in camera data from main thread running pybullet"""

    while True:

        image = sim.camera.get_image()[0]

        texture = o3d.geometry.Image(image)

        tsdf_mesh.textures = [texture]
        tsdf_mesh.compute_vertex_normals()
        tsdf_mesh.compute_triangle_normals()

        vis = o3d.visualization.Visualizer()

        # Add the TSDF volume to the visualizer
        vis.create_window()
        vis.add_geometry(tsdf_mesh)

        vis.run()

def thread_handler(sim,tsdf_mesh):

    # Create two threads, one for each window
    thread_open3d = threading.Thread(target=open3d_window, args=(sim, tsdf_mesh))
    thread_pybullet = threading.Thread(target=pybullet, args=(sim))
    thread_open3d.start()
    thread_pybullet.start()

    thread_open3d.join()
    thread_pybullet.join()  

def main():
        gui = True
        scene_id = "random"
        vgn_path = "/home/tom/dev_ws/thesis_ws/src/vgn/assets/models/vgn_conv.pth" #was changed 

        env = Environment(gui, scene_id, vgn_path)
        env.load_engine()
        mesh = env.get_tsdf()

        thread_handler(env.sim, mesh)

if __name__ == "__main__":
     main()
