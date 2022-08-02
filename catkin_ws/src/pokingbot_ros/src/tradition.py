#! /usr/bin/env python3
import os
import rospy
import tensorflow as tf
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan, Joy
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R
import yaml
import time
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from std_srvs.srv import Trigger
from gazebo_msgs.msg import ContactsState

class GoalNav(object):
    def __init__(self):
        super().__init__()
        self.max_dis = 10  # meters
        self.laser_n = 4
        self.pos_n = 10
        self.cnt = 0
        self.frame = rospy.get_param("~frame", "map")
        self.action_scale = {'linear': rospy.get_param(
            '~linear_scale', 0.3), 'angular': rospy.get_param("~angular_scale", 0.18)}

        self.auto = 0
        self.goal = None
        self.pos_track = None
        self.laser_stack = None
        self.last_pos = None
        self.collision_states = False

        self.last_omega = 0
        self.omega_gamma = 0.25
        self.vel_ratio = 0

        self.total_traj = []

        # network
        obs_dim = 243
        action_dim = 2
        gpu = tf.config.experimental.list_physical_devices('GPU')
        tf.config.experimental.set_memory_growth(gpu[0], True)
        self.my_dir = os.path.abspath(os.path.dirname(__file__))
        model_path = os.path.join(self.my_dir, "../../../../../model/goal/policy")
        self.policy_network = tf.saved_model.load(model_path)

        # read yaml
        with open(os.path.join(self.my_dir,"../../../../Data/goal.yaml"), 'r') as f:
            data = yaml.load(f)

        self.goal_totoal = data['goal']

        # metric
        self.count = 0
        self.total = len(self.goal_totoal)
        self.success = 0
        self.coi = 0

        # pub cmd
        self.pub_cmd = rospy.Publisher("cmd_out", Twist, queue_size=1)

        # subscriber, timer
        self.sub_joy = rospy.Subscriber("joy", Joy, self.cb_joy, queue_size=1)
        self.sub_odom = rospy.Subscriber(
            "odom_in", PoseStamped, self.cb_odom, queue_size=1)
        self.sub_laser = rospy.Subscriber(
            "laser_in",  LaserScan, self.cb_laser, queue_size=1)
        self.initial = rospy.ServiceProxy("/husky_ur5/init", Trigger)
        self.sub_collision = rospy.Subscriber("/robot/bumper_states", ContactsState, self.cb_collision, queue_size=1)
        self.get_robot_pos = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.loop()

    def loop(self):

        while(1):

            # initial
            self.initial()

            if(self.count == self.total):
                # finish all goal

                # calculate metric
                s_r = (self.success/self.total) * 100
                f_r = 100 - s_r
                a_c = self.coi / self.total

                # output result 
                d = {'success_rate':s_r, "fail_rate":f_r, "average_coillision":a_c}

                tra = {'environment' : "room_door", "policy": "RL_oa", "trajectories" : self.total_traj}

                with open(os.path.join(self.my_dir,"../../../../Data/RL_oa_trajectory.yaml"), "w") as f:

                    yaml.dump(tra, f)

                with open(os.path.join(self.my_dir,"../../../../Data/RL_oa_result.yaml"), "w") as f:

                    yaml.dump(d, f)

                rospy.loginfo('End')
                break
            else:
                self.goal = self.goal_totoal[self.count]
                self.count += 1
                self.auto = 1
                self.inference()

    def cb_collision(self, msg):
        if self.collision_states == True:
            if msg.states == [] and self.cnt > 10000:
                self.collision_states = False
            else:
                self.cnt += 1
        elif msg.states != [] and self.cnt == 0:
            self.collision_states = True
            self.coi += 1
        else:
            self.collision_states = False
            self.cnt = 0

    def scale_pose(self, value):
        if value > 0:
            return math.log(1 + value)
        elif value < 0:
            return -math.log(1 + abs(value))

    def cb_joy(self, msg):
        start_button = 7
        back_button = 6

        if (msg.buttons[start_button] == 1) and not self.auto:
            self.auto = 1
            rospy.loginfo('go auto')
        elif msg.buttons[back_button] == 1 and self.auto:
            self.auto = 0
            rospy.loginfo('go manual')

    def cb_odom(self, msg):
        if self.goal is None:
            self.pos_track = None
            return

        # caculate angle diff
        new_pos = np.array(
            [msg.pose.position.x, msg.pose.position.y])
        diff = self.goal - new_pos
        r = R.from_quat([msg.pose.orientation.x,
                         msg.pose.orientation.y,
                         msg.pose.orientation.z,
                         msg.pose.orientation.w])
        yaw = r.as_euler('zyx')[0]
        angle = math.atan2(diff[1], diff[0]) - yaw
        if angle >= np.pi:
            angle -= 2*np.pi
        elif angle <= -np.pi:
            angle += 2*np.pi

        # update pose tracker
        diff = np.array([self.scale_pose(v) for v in diff])
        track_pos = np.append(diff, angle)
        if self.pos_track is None:
            self.pos_track = np.tile(track_pos, (self.pos_n, 1))
        else:
            self.pos_track[:-1] = self.pos_track[1:]
            self.pos_track[-1] = track_pos
        self.last_pos = new_pos

    def cb_laser(self, msg):
        intensities = np.array(msg.intensities)
        ranges = np.array(msg.ranges)
        ranges = np.clip(ranges, 0, self.max_dis)

        for i in range(len(intensities)):
            if intensities[i]==1: ranges[i]=100 # ignore

        if self.laser_stack is None:
            self.laser_stack = np.tile(ranges, (self.laser_n, 1))
        else:
            self.laser_stack[:-1] = self.laser_stack[1:]
            self.laser_stack[-1] = ranges

    def check(self):

        if self.pos_track is None:
            return "nothing"
        else:
            pos = True
        if self.laser_stack is None:
            return "nothing"
        else:
            laser = True

        if(pos and laser):
            return "ready"

    def inference(self):

        while(1):
            if(self.check() == "ready"):
                break
                
        begin = time.time()

        tra = []

        while(1):

            robot_pose = self.get_robot_pos("robot", "")
            r_pose = {"position" : [robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z],
                      "orientation" : [robot_pose.pose.orientation.x, robot_pose.pose.orientation.y, robot_pose.pose.orientation.z, robot_pose.pose.orientation.w]}
            tra.append(r_pose)

            # reshape
            laser = self.laser_stack.reshape(-1)
            track = self.pos_track.reshape(-1)
            state = np.append(laser, track)

            state = tf.convert_to_tensor([state], dtype=tf.float32)

            action = self.policy_network(state)[0].numpy()
            self.last_omega = self.omega_gamma * \
                action[1] + (1-self.omega_gamma)*self.last_omega

            cmd = Twist()
            cmd.linear.x = action[0]*self.action_scale['linear']
            cmd.angular.z = self.last_omega * \
                self.action_scale['angular']

            self.pub_cmd.publish(cmd)
            
            dis = np.linalg.norm(self.goal-self.last_pos)
            if dis < 0.8:
                self.success += 1
                self.total_traj.append(tra[0:-1:50])
                rospy.loginfo("goal reached")
                break

            if((time.time() - begin) >= 60):
                self.total_traj.append(tra[0:-1:50])
                break

if __name__ == "__main__":
    rospy.init_node("goal_nav_rl")
    goalNav = GoalNav()
    rospy.spin()