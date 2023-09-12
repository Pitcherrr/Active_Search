from controller_manager_msgs.srv import *
import copy
import cv_bridge
from geometry_msgs.msg import Twist
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from random import sample
import geometry_msgs.msg
import trimesh
import threading
import time
from torch.utils.tensorboard import SummaryWriter


from active_grasp.bbox import from_bbox_msg, AABBox
from active_grasp.timer import Timer
from active_grasp.srv import Reset, ResetRequest
from robot_helpers.ros import tf
from robot_helpers.ros.conversions import *
from robot_helpers.ros.panda import PandaArmClient, PandaGripperClient
from robot_helpers.ros.moveit import MoveItClient, create_collision_object_from_mesh
from robot_helpers.spatial import Rotation, Transform
from vgn.utils import look_at, cartesian_to_spherical, spherical_to_cartesian
from vgn.detection import select_local_maxima
import torch

from .models import Autoencoder


class GraspController:
    def __init__(self, policy):
        self.policy = policy
        print(self.policy)
        self.load_parameters()
        self.init_service_proxies()
        self.init_robot_connection()
        self.init_moveit()
        self.init_camera_stream()
        self.init_tensorboard()

    def load_parameters(self):
        self.base_frame = rospy.get_param("~base_frame_id")
        self.T_grasp_ee = Transform.from_list(rospy.get_param("~ee_grasp_offset")).inv()
        self.T_grasp_drop = Transform.from_list(rospy.get_param("~grasp_drop_config"))
        self.cam_frame = rospy.get_param("~camera/frame_id")
        self.depth_topic = rospy.get_param("~camera/depth_topic")
        self.min_z_dist = rospy.get_param("~camera/min_z_dist")
        self.control_rate = rospy.get_param("~control_rate")
        self.linear_vel = rospy.get_param("~linear_vel")
        self.policy_rate = rospy.get_param("policy/rate")
    
    def init_service_proxies(self):
        self.reset_env = rospy.ServiceProxy("reset", Reset)
        self.switch_controller = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )

    def init_robot_connection(self):
        self.arm = PandaArmClient()
        self.gripper = PandaGripperClient()
        topic = rospy.get_param("cartesian_velocity_controller/topic")
        self.cartesian_vel_pub = rospy.Publisher(topic, Twist, queue_size=10)
        rospy.Subscriber('sim_complete', Bool, self.sim_complete, queue_size=1)

    def init_moveit(self):
        self.moveit = MoveItClient("panda_arm")
        rospy.sleep(1.0)  # Wait for connections to be established.
        self.moveit.move_group.set_planner_id("RRTstar")
        self.moveit.move_group.set_planning_time(3.0)

    def switch_to_cartesian_velocity_control(self):
        req = SwitchControllerRequest()
        req.start_controllers = ["cartesian_velocity_controller"]
        req.stop_controllers = ["position_joint_trajectory_controller"]
        req.strictness = 1
        self.switch_controller(req)

    def switch_to_joint_trajectory_control(self):
        req = SwitchControllerRequest()
        req.start_controllers = ["position_joint_trajectory_controller"]
        req.stop_controllers = ["cartesian_velocity_controller"]
        req.strictness = 1
        self.switch_controller(req)

    def init_camera_stream(self):
        self.cv_bridge = cv_bridge.CvBridge()
        rospy.Subscriber(self.depth_topic, Image, self.sensor_cb, queue_size=1)

    def init_tensorboard(self):
        self.writer = SummaryWriter()
        self.frame = 0

    def sensor_cb(self, msg):
        self.latest_depth_msg = msg

    def sim_complete(self, msg):
        if msg.data:
            self.complete = True

    def run(self):
        self.policy.init_tsdf()
        self.policy.target_bb = self.reset()
        self.complete = False
        voxel_size = 0.0075
        x_off = 0.35
        y_off = -0.15
        z_off = 0.2

        bb_min = [x_off,y_off,z_off]
        bb_max = [40*voxel_size+x_off,40*voxel_size+y_off,40*voxel_size+z_off]
        self.bbox = AABBox(bb_min, bb_max)

        self.view_sphere = ViewHalfSphere(self.bbox, self.min_z_dist)
        self.policy.activate(self.bbox, self.view_sphere)

        self.switch_to_cartesian_velocity_control()
        r = rospy.Rate(self.policy_rate)

        # sample = self.sample_environment(self.bbox)

        state = self.get_state()
        enc_state = self.policy.get_encoded_state(state[0], state[1], state[2])

        # init_occ = len(self.policy.coordinate_mat)

        replay_mem = []
        grasp_mask = []
        view_mask = []
        it = 0 
        max_it = 20
        replay_size = 5
        batch_size = 32
        gamma = 0.9

        grasp_criterion = torch.nn.MSELoss()
        view_criterion = torch.nn.MSELoss()

        while not self.complete and it < max_it:
            with Timer("inference_time"):
                state = self.get_state()
                enc_state = self.policy.get_encoded_state(state[0], state[1], state[2])
                model_sample = self.action_inference(enc_state, state[2], self.bbox)
                [grasp, view, action, value, lprob, terminal] = model_sample


            init_occ = len(self.policy.coordinate_mat) if len(self.policy.coordinate_mat) > 0 else 1
            
            if grasp:
                print("grasping")
                start_time = time.time()
                self.switch_to_joint_trajectory_control()
                grasp_thread = threading.Thread(target=self.execute_grasp, args= (action,))
                with Timer("grasp_time"):
                    grasp_thread.start()
                    while True:
                        if self.grasp_integrate:
                            img, pose, q = self.get_state()
                            self.policy.integrate(img, pose, q)
                        r.sleep()
                        if not grasp_thread.is_alive():
                            res = self.grasp_result
                            break
                self.switch_to_cartesian_velocity_control()
                print("grasp_ig:", self.grasp_ig)
                occ_diff = torch.tensor(float(10-10*(1 - self.grasp_ig/init_occ)), requires_grad= True).to("cuda") #+ve diff is good
                exec_time = time.time() - start_time
                info = self.collect_info(res)
                grasp_mask.append(1)
                view_mask.append(0)
            elif view:
                start_time = time.time()
                t = 0
                self.policy.x_d = action
                timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.send_vel_cmd)
                print("Executing for: 3 seconds")
                while t < (3.0):
                    img, pose, q = self.get_state()
                    self.policy.integrate(img, pose, q)
                    t += 1/self.policy_rate
                    r.sleep()
                rospy.sleep(0.2)        
                timer.shutdown()
                exec_time = time.time() - start_time
                occ_diff = torch.tensor(float(10-10*(len(self.policy.coordinate_mat)/init_occ)), requires_grad= True).to("cuda")  #+ve diff is good
                grasp_mask.append(0)
                view_mask.append(1)
            else:
                res = "aborted"

            reward = self.compute_reward(grasp, view, terminal, occ_diff, exec_time)
            print("Reward", reward)

            next_state = self.get_state()
            next_enc_state = self.policy.get_encoded_state(next_state[0], next_state[1], next_state[2])

            replay_mem.append([enc_state, q, action, reward, next_enc_state, next_state[2], self.policy.done])

            if len(replay_mem) > replay_size:
                del replay_mem[0]

            batch = sample(replay_mem, min(len(replay_mem), batch_size))
            state_batch, q_batch, action_batch, reward_batch, next_state_batch, next_q_batch, terminal_batch = zip(*batch)

            # print("state", state_batch)
            cur_pred_batch = []
            for enum_state, enum_q in zip(state_batch, q_batch):
                val = self.action_inference(enum_state, enum_q, self.bbox)
                cur_pred_batch.append(val)

            next_pred_batch = []
            for enum_state, enum_q in zip(next_state_batch, next_q_batch):
                val = self.action_inference(enum_state, enum_q, self.bbox)
                next_pred_batch.append(val)
                
            # cur_pred_batch = self.action_inference(state_batch, q_batch, self.bbox)
            # next_pred_batch = self.action_inference(next_state_batch, next_q_batch, self.bbox)
            # print(reward_batch)
            # print(terminal_batch)
            # print(next_pred_batch)

            y_batch = torch.tensor([reward if terminal else reward + gamma * prediction[3][0] for reward, terminal, prediction in
                  zip(reward_batch, terminal_batch, next_pred_batch)], requires_grad=True).to(self.policy.device)
            
            # print(y_batch)

            # print("current:", cur_pred_batch)

            cur_val_batch = torch.tensor([value[3] for value in cur_pred_batch], requires_grad=True).to(self.policy.device)
            # print(cur_val_batch)
            # print(action_batch)

            q_value = torch.sum(cur_val_batch, dim=1)

            self.policy.grasp_nn.optimizer.zero_grad()
            self.policy.view_nn.optimizer.zero_grad()

            grasp_loss = grasp_criterion(q_value, y_batch)
            view_loss = view_criterion(q_value, y_batch)

            print("grasp loss:", grasp_loss)
            print("view loss:", view_loss)

            grasp_loss.backward()
            view_loss.backward()

            self.policy.grasp_nn.optimizer.step()
            self.policy.view_nn.optimizer.step()

            # state = next_state

            # self.update_networks(grasp, view, action, reward, lprob)

            self.writer.add_scalar("Grasp Loss/train", grasp_loss, self.frame)
            self.writer.add_scalar("View Loss/train", view_loss, self.frame)

            print("Frame:", self.frame)
            self.frame += 1

            it += 1


        self.writer.flush()
        return info

    def reset(self):
        Timer.reset()
        self.moveit.scene.clear()
        res = self.reset_env(ResetRequest())
        rospy.sleep(1.0)  # Wait for the TF tree to be updated.
        bb = from_bbox_msg(res.bbox)
        # for i in range(len(bbs)):
            # bbs[i] = from_bbox_msg(bbs[i])
        return bb
    
    
    def action_inference(self, state, q, bbox):
        self.view_sphere = ViewHalfSphere(bbox, self.min_z_dist)
        self.policy.init_data()
        timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.send_vel_cmd)
        # model_sample = self.policy.update(state, q)
        # grasp_q, grasp_t, view_q, view_t = self.policy.update(state, q)
        start = time.time()
        grasp_vals, view_vals = self.policy.update(state,q)
        print("inference took:", time.time() - start)
        # print("network output", out)
        # model_sample = self.policy.sample_action(grasp_q, grasp_t, view_q, view_t)
        model_sample = self.policy.sample_action(grasp_vals, view_vals)

        timer.shutdown()
        return model_sample


    def search_grasp(self, bbox):
        self.view_sphere = ViewHalfSphere(bbox, self.min_z_dist)
        self.policy.activate(bbox, self.view_sphere)
        timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.send_vel_cmd)
        r = rospy.Rate(self.policy_rate)
        while not self.policy.done:
            img, pose, q = self.get_state()
            self.policy.update(img, pose, q)
            r.sleep()
        rospy.sleep(0.2)  # Wait for a zero command to be sent to the robot.
        self.policy.deactivate()
        timer.shutdown()
        return self.policy.best_grasp
    
    def get_scene_grasps(self, bbox):
        self.view_sphere = ViewHalfSphere(bbox, self.min_z_dist)
        self.policy.activate(bbox, self.view_sphere)
        origin = self.policy.T_base_task
        origin.translation[2] -= 0.05
        voxel_size, tsdf_grid = self.policy.tsdf.voxel_size, self.policy.tsdf.get_grid()
        # Then check whether VGN can find any grasps on the target
        out = self.policy.vgn.predict(tsdf_grid)
        grasps, qualities = select_local_maxima(voxel_size, out, threshold=0.8)

        for grasp in grasps:
            pose = origin * grasp.pose
            tip = pose.rotation.apply([0, 0, 0.05]) + pose.translation
            if bbox.is_inside(tip):
                return True, grasp
        return False, None
    
    def grasp_ig(self, grasp):
        #naive estimation of information gain from the grasping of an opject
        t = (self.policy.T_task_base * grasp.pose).translation
        i, j, k = (t / self.policy.tsdf.voxel_size).astype(int)
        bb_voxel = [5,5,5] #place holder for the actual target object size 
        # grasp_ig_mat = self.policy.occ_mat[i:i+2*bb_voxel[0],j-(bb_voxel[1]//2):j+(bb_voxel[1]//2),:] #most "correct approach" but gives some issues
        grasp_ig_mat = self.policy.occ_mat[i:,j-(bb_voxel[1]//2):j+(bb_voxel[1]//2),:] #most "correct approach" but gives some issues
        grasp_ig = grasp_ig_mat.sum()
        print("Grasp information gain:", grasp_ig)
        return grasp_ig

    def compute_reward(self, grasp, view, terminal, occ_diff, action_time):
        if terminal:
            return 100.0
        elif grasp:
            reward = occ_diff - action_time
            return reward
        elif view:
            reward = occ_diff - 3
            return reward
        
    def update_networks(self, grasp, view, action, reward, lprob):
        if grasp:
            self.policy.grasp_nn.optimizer.zero_grad()
            loss = lprob * reward
            loss.backward()
            self.policy.grasp_nn.optimizer.step()
        elif view:
            self.policy.view_nn.optimizer.zero_grad()
            loss = lprob * reward
            reward.backward()
            self.policy.view_nn.optimizer.step()

        self.writer.add_scalar("Loss/train", loss, self.frame)

    def get_state(self):
        q, _ = self.arm.get_state()
        msg = copy.deepcopy(self.latest_depth_msg)
        img = self.cv_bridge.imgmsg_to_cv2(msg).astype(np.float32) * 0.001
        pose = tf.lookup(self.base_frame, self.cam_frame, msg.header.stamp)
        return img, pose, q

    def send_vel_cmd(self, event):
        if self.policy.x_d is None or self.policy.done:
            cmd = np.zeros(6)
        else:
            x = tf.lookup(self.base_frame, self.cam_frame)
            cmd = self.compute_velocity_cmd(self.policy.x_d, x)
        self.cartesian_vel_pub.publish(to_twist_msg(cmd))

    def compute_velocity_cmd(self, x_d, x):
        r, theta, phi = cartesian_to_spherical(x.translation - self.view_sphere.center)
        e_t = x_d.translation - x.translation
        e_n = (x.translation - self.view_sphere.center) * (self.view_sphere.r - r) / r
        linear = 1.0 * e_t + 6.0 * (r < self.view_sphere.r) * e_n
        scale = np.linalg.norm(linear) + 1e-6
        linear *= np.clip(scale, 0.0, self.linear_vel) / scale
        angular = self.view_sphere.get_view(theta, phi).rotation * x.rotation.inv()
        angular = 1.0 * angular.as_rotvec()
        return np.r_[linear, angular]

    def execute_grasp(self, grasp):
        self.grasp_integrate = True
        self.create_collision_scene()
        T_base_grasp = self.postprocess(grasp.pose)
        self.gripper.move(0.08)
        T_base_approach = T_base_grasp * Transform.t_[0, 0, -0.06] * self.T_grasp_ee
        success, plan = self.moveit.plan(T_base_approach, 0.2, 0.2)
        if success:
            self.moveit.scene.clear()
            self.moveit.execute(plan)
            rospy.sleep(0.5)  # Wait for the planning scene to be updated
            self.moveit.gotoL(T_base_grasp * self.T_grasp_ee)
            rospy.sleep(0.5)
            self.gripper.grasp()
            self.grasp_integrate = False
            #remove the body from the scene
            T_base_retreat = Transform.t_[0, 0, 0.05] * T_base_grasp * self.T_grasp_ee
            self.moveit.gotoL(T_base_retreat)
            rospy.sleep(1.0)  # Wait to see whether the object slides out of the hand
            success = self.gripper.read() > 0.002

            if success:
                self.grasp_result = "succeeded"
                remove_body = rospy.ServiceProxy('remove_body', Reset)
                response = from_bbox_msg(remove_body(ResetRequest()).bbox)
                self.grasp_ig = self.policy.tsdf_cut(response)
                self.moveit.goto([0.79, -0.79, 0.0, -2.356, 0.0, 1.57, 0.79])
                self.gripper.move(0.08)
            else:
                self.grasp_result = "failed" 
                self.grasp_ig = 0.0
 
        else:
            self.grasp_result = "no_motion_plan_found"

    def create_collision_scene(self):
        # Segment support surface
        cloud = self.policy.tsdf.get_scene_cloud()
        cloud = cloud.transform(self.policy.T_base_task.as_matrix())
        _, inliers = cloud.segment_plane(0.01, 3, 1000)
        support_cloud = cloud.select_by_index(inliers)
        cloud = cloud.select_by_index(inliers, invert=True)
        # o3d.io.write_point_cloud(f"{time.time():.0f}.pcd", cloud)

        # Add collision object for the support
        self.add_collision_mesh("support", compute_convex_hull(support_cloud))

        # Cluster cloud
        labels = np.array(cloud.cluster_dbscan(eps=0.01, min_points=8))

        # Generate convex collision objects for each segment
        self.hulls = []

        if len(labels) == 0: #with active seach sometimes this is empty
            self.search_grasp(self.bbox)

        if len(labels) > 0:
            for label in range(labels.max() + 1):
                segment = cloud.select_by_index(np.flatnonzero(labels == label))
                try:
                    hull = compute_convex_hull(segment)
                    name = f"object_{label}"
                    self.add_collision_mesh(name, hull)
                    self.hulls.append(hull)
                except:
                    # Qhull fails in some edge cases
                    pass
        else:
            self.search_grasp(self.bbox)

    def add_collision_mesh(self, name, mesh):
        frame, pose = self.base_frame, Transform.identity()
        co = create_collision_object_from_mesh(name, frame, pose, mesh)
        self.moveit.scene.add_object(co)
        # # Add a box to the planning scene to avoid collisions with the table.
        msg = geometry_msgs.msg.PoseStamped()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = 0.4
        msg.pose.position.z = 0.18
        self.moveit.scene.add_box("table", msg, size=(0.6, 0.6, 0.02))

    def postprocess(self, T_base_grasp):
        rot = T_base_grasp.rotation
        if rot.as_matrix()[:, 0][0] < 0:  # Ensure that the camera is pointing forward
            T_base_grasp.rotation = rot * Rotation.from_euler("z", np.pi)
        T_base_grasp *= Transform.t_[0.0, 0.0, 0.01]
        return T_base_grasp

    def collect_info(self, result):
        points = [p.translation for p in self.policy.views]
        d = np.sum([np.linalg.norm(p2 - p1) for p1, p2 in zip(points, points[1:])])
        info = {
            "result": result,
            "view_count": len(points),
            "distance": d,
        }
        info.update(self.policy.info)
        info.update(Timer.timers)
        return info


def compute_convex_hull(cloud):
    hull, _ = cloud.compute_convex_hull()
    triangles, vertices = np.asarray(hull.triangles), np.asarray(hull.vertices)
    return trimesh.base.Trimesh(vertices, triangles)


class ViewHalfSphere:
    def __init__(self, bbox, min_z_dist):
        self.center = bbox.center
        self.r = 0.5 * bbox.size[2] + min_z_dist

    def get_view(self, theta, phi):
        eye = self.center + spherical_to_cartesian(self.r, theta, phi)
        up = np.r_[1.0, 0.0, 0.0]
        return look_at(eye, self.center, up)

    def sample_view(self):
        raise NotImplementedError
