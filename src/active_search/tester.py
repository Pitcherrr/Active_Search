
#import active_grasp.src.active_grasp.simulation
import pybullet
import pybullet_data
import math
import numpy as np
import time

from robot_helpers.bullet import *
from robot_helpers.model import *

def main():
    #Connect to the physics environment
    cid = pybullet.connect(pybullet.SHARED_MEMORY)
    if (cid < 0):
        pybullet.connect(pybullet.GUI)

    pybullet.resetSimulation()
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

    pybullet.setGravity(0, 0, -9.81)
    pybullet.setTimeStep(0.0001)
    pybullet.setRealTimeSimulation(0)

    arm = BtPandaArm()

    gripper = BtPandaGripper(arm)

    camera = BtCamera(320, 240, 0.96, 0.01, 1.0, arm.uid, 11)

    #Get intial joint positions
    [j1_init, j2_init, j3_init, j4_init, j5_init, j6_init, j7_init] = [j[0] for j in pybullet.getJointStates(arm.uid, range(7))]

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
    camera.get_image()

    start_time = time.time()
    update_interval = 1  # update FPS once per second
    last_update_time = start_time

    while True:
    
        #start_time = time.time() # start time of the loop

        j1 = j1_init + pybullet.readUserDebugParameter(j1_comm)
        j2 = j2_init + pybullet.readUserDebugParameter(j2_comm)
        j3 = j3_init + pybullet.readUserDebugParameter(j3_comm)
        j4 = j4_init + pybullet.readUserDebugParameter(j4_comm)
        j5 = j5_init + pybullet.readUserDebugParameter(j5_comm)
        j6 = j6_init + pybullet.readUserDebugParameter(j6_comm)
        j7 = j7_init + pybullet.readUserDebugParameter(j7_comm)
        grip_width = pybullet.readUserDebugParameter(grip_comm)

        #text_id = p.addUserDebugText("FPS: 0", [0.9, 0.1, 0.1], textColorRGB=[1, 0, 0], textSize=2)

        robot_pos = [j1, j2, j3, j4, j5, j6, j7]
        gripper.set_desired_width(grip_width)

        if frame_buff == 20:
            camera.get_image()
            #pybullet.addUserDebugText("FPS: {}".format(fps), [0.9, 0.1, 0.1], textColorRGB=[1, 0, 0], textSize=2, replaceItemUniqueId=text_id)
            frame_buff = 0

        current_time = time.time()
        if current_time - last_update_time >= update_interval:
            # calculate the FPS and update the text
            fps = int(frame_count / (current_time - last_update_time))
            print("FPS:",fps)

        pybullet.setJointMotorControlArray(arm.uid, range(7), pybullet.POSITION_CONTROL, targetPositions=robot_pos)
        pybullet.stepSimulation()

        #fps = int(1.0 / (time.time() - start_time)) # FPS = 1 / time to process loop

        frame_buff += 1 
        frame_count += 1


if __name__ == "__main__":
    main()
