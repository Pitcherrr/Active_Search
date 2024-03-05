#!/usr/bin/env python3

import argparse
from datetime import datetime
import pandas as pd
from pathlib import Path
import rospy
from tqdm import tqdm
# from std_srvs.srv import SetBool, Empty, Trigger, TriggerRequest, TriggerResponse
# from std_msgs.msg import String
import glob
import os

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

    print("Resetting controller")
    controller.reset()
    print("Activating policy")
    controller.policy.activate(AABBox([0,0,0], [0.3,0.3,0.3]), None)

    for n in tqdm(range(10000), disable=args.wait_for_input):
        if args.wait_for_input:
            controller.gripper.move(0.08)
            controller.switch_to_joint_trajectory_control()
            controller.moveit.goto("ready", velocity_scaling=0.4)
            i = input("Run policy? [y/n] ")
            if i != "y":
                exit()
            rospy.loginfo("Running policy ...")
        info = controller.run()
        logger.log_run(info)
        # test the policy every 15 games
        if (n+1) % 20 == 0:
            controller.policy.view_nn.save_model()
            controller.policy.grasp_nn.save_model()
            enumerate_test_scenes(controller)

def enumerate_test_scenes(controller):
    print("############### Testing Policy ######################")
    change_sim = rospy.ServiceProxy("change_sim", ServiceStr)
    test_cases = ["test_1.yaml", "test_2.yaml", "test_3.yaml", "test_4.yaml"]
    # test_cases = get_all_test_files()
    print("Running test cases", test_cases)

    for case in test_cases:
        msg = ServiceStrRequest()
        msg.input_str = case
        res = change_sim(msg)
        print("Change sim response", res.output_str)

        # seed_simulation(args.seed)
        rospy.sleep(1.0)  # Prevents a rare race condiion

        controller.reset()
        controller.policy.activate(AABBox([0,0,0], [0.3,0.3,0.3]), None)

        # reset_tsdf = rospy.ServiceProxy('reset_map', Empty)
        # tsdf_response = reset_tsdf() # Call the service with argument True
        # print(tsdf_response)

        controller.gripper.move(0.08)
        controller.switch_to_joint_trajectory_control()
        controller.moveit.goto("ready", velocity_scaling=0.4)
     
        rospy.loginfo("Running policy ...")
        info = controller.run_policy(case)
        # info = controller.run_baseline()

    msg = ServiceStrRequest()
    msg.input_str = "random"
    res = change_sim(msg)
    print("Change sim response", res.output_str)


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

    def log_run(self, info):
        df = pd.DataFrame.from_records([info])
        df.to_csv(self.path, mode="a", header=not self.path.exists(), index=False)


def seed_simulation(seed):
    print("Seeding sim")
    rospy.ServiceProxy("seed", Seed)(seed)
    rospy.sleep(1.0)
    

def get_all_test_files() -> list:
    directory_path = "/home/pitcher/dev_ws/thesis_ws/active_search/src/active_search/cfg/sim"
    pattern = "test_*.yaml"

    # Get a list of matching files in the directory
    matching_files = glob.glob(os.path.join(directory_path, pattern))

    if matching_files:
        # Sort the files by the numeric part of their names
        sorted_files = sorted(matching_files, key=lambda x: int(os.path.splitext(os.path.basename(x))[0].split('_')[-1]))
        
        # Extract the numeric part from each file name
        file_numbers = [int(os.path.splitext(os.path.basename(file))[0].split('_')[-1]) for file in sorted_files]

        # Generate the list of test file names
        test_files = ["test_" + str(number) + ".yaml" for number in file_numbers]

        return test_files
    else:
        print("No matching files found.")
        return []
    

if __name__ == "__main__":
    main()
