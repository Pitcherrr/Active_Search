#!/usr/bin/env python3

import sys
# import os
import argparse
from datetime import datetime
import pandas as pd
from pathlib import Path
import rospy
from tqdm import tqdm
from std_srvs.srv import SetBool, Empty, Trigger, TriggerRequest, TriggerResponse
from std_msgs.msg import String

sys.path.append('/home/tom/dev_ws/active_search/src/active_grasp/src/')
sys.path.append('/usr/local/lib/python2.7/dist-packages')
# print(sys.path)

from active_grasp.bbox import AABBox
from active_search.rl_controller import *
from active_search.open3d_viz import *
from active_search.search_policy import make, registry
from active_grasp.srv import Seed
from active_search.srv import ServiceStr, ServiceStrRequest
from robot_helpers.ros import tf



def main():
    rospy.init_node("grasp_controller")

    change_sim = rospy.ServiceProxy("change_sim", ServiceStr)

    msg = ServiceStrRequest()
    msg.input_str = "random"
    res = change_sim(msg)
    print("Change sim response", res.output_str)

    tf.init()

    parser = create_parser()
    args = parser.parse_args()

    policy = make(args.policy)
    print("Policy", policy)
    controller = GraspController(policy)
    logger = Logger(args)

    seed_simulation(args.seed)
    rospy.sleep(1.0)  # Prevents a rare race condiion

    for n in tqdm(range(400), disable=args.wait_for_input):
        controller.reset()
        controller.policy.activate(AABBox([0,0,0], [0.3,0.3,0.3]), None)
        if args.wait_for_input:
            controller.gripper.move(0.08)
            controller.switch_to_joint_trajectory_control()
            controller.moveit.goto("ready", velocity_scaling=0.4)
            i = input("Run policy? [y/n] ")
            if i != "y":
                exit()
            rospy.loginfo("Running policy ...")
        # info = controller.run()
        # logger.log_run(info)
        # print("n % 15:", n % 15)
        # if (n+1) % 15 == 0:
        user = input("Press a key to start")
        run_hw(controller, logger)

def enumerate_test_scenes(controller, logger):
    rospy.loginfo("############### Testing Policy ######################")
    change_sim = rospy.ServiceProxy("change_sim", ServiceStr)
    test_cases = ["test.yaml", "test_2.yaml"]

    for case in test_cases:
        msg = ServiceStrRequest()
        msg.input_str = case
        res = change_sim(msg)
        print("Change sim response", res.output_str)

        # seed_simulation(args.seed)
        rospy.sleep(1.0)  # Prevents a rare race condiion

        controller.reset()
        controller.policy.activate(AABBox([0,0,0], [0.3,0.3,0.3]), None)

        controller.gripper.move(0.08)
        controller.switch_to_joint_trajectory_control()
        controller.moveit.goto("ready", velocity_scaling=0.4)
     
        rospy.loginfo("Running policy ...")
        info = controller.run_policy(case)

        # info = controller.run_policy()

        logger.log_run(info)
        # info = controller.run() 

    msg = ServiceStrRequest()
    msg.input_str = "random"
    res = change_sim(msg)
    print("Change sim response", res.output_str)

def run_hw(controller, logger):

    controller.reset()
    controller.policy.activate(AABBox([0,0,0], [0.3,0.3,0.3]), None)

    controller.gripper.move(0.08)
    # controller.switch_to_joint_trajectory_control()
    # controller.moveit.goto("ready", velocity_scaling=0.4)

    _ = input("Stop here")
    
    rospy.loginfo("Running policy ...")
    # info = controller.run_policy("test1.yaml")
    info = controller.run_vgn()
    # info = controller.run_baseline()

    logger.log_run(info) 


def create_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("policy", type=str, choices=registry.keys())
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--wait-for-input", action="store_true")
    parser.add_argument("--logdir", type=Path, default="logs")
    parser.add_argument("--seed", type=int, default=1)
    return parser


class Logger:
    def __init__(self, args):
        args.logdir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%y%m%d-%H%M%S")
        name = "{}_policy={},seed={}.csv".format(
            stamp,
            args.policy,
            args.seed,
        )
        self.path = args.logdir / name
        print("log path is:", self.path)

    def log_run(self, info):
        df = pd.DataFrame.from_records([info])
        df.to_csv(self.path, mode="a", header=not self.path.exists(), index=False)


def seed_simulation(seed):
    rospy.ServiceProxy("seed", Seed)(seed)
    rospy.sleep(1.0)


if __name__ == "__main__":
    main()
