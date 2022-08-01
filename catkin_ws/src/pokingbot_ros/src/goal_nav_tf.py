#! /usr/bin/env python3
import os
import rospy
import tensorflow as tf
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import LaserScan, Joy
from std_msgs.msg import Bool, Float64
from scipy.spatial.transform import Rotation as R
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
import yaml
import time
from std_srvs.srv import Trigger

class GoalNav(object):
    def __init__(self):
        super().__init__()
        self.max_dis = 10  # meters
        self.laser_n = 4
        self.pos_n = 10
        self.cnt = 0
        self.frame = rospy.get_param("~frame", "map")
        self.action_scale = {'linear': rospy.get_param(
            '~linear_scale', 0.3), 'angular': rospy.get_param("~angular_scale", 0.45)}

        self.auto = 0
        self.goal = None
        self.pos_track = None
        self.state_stack, self.state_label_stack = None, None
        self.last_pos = None
        self.collision_states = False

        self.last_omega = 0
        self.omega_gamma = 0.25
        self.vel_ratio = 0

        self.total_traj = []
        self.pose = PoseStamped()

        # network
        gpu = tf.config.experimental.list_physical_devices('GPU')
        tf.config.experimental.set_memory_growth(gpu[0], True)
        self.my_dir = os.path.abspath(os.path.dirname(__file__))
        model_path = os.path.join(self.my_dir, "../model/policy")
        self.policy_network = tf.saved_model.load(model_path)

        # read yaml
        with open(os.path.join(self.my_dir,"../goal.yaml"), 'r') as f:
            data = yaml.load(f)

        self.goal_totoal = data['goal']

        # metric
        self.count = 0
        self.total = len(self.goal_totoal)
        self.success = 0
        self.coi = 0

        # pub cmd
        self.pub_cmd = rospy.Publisher("cmd_out", Twist, queue_size=1)
        self.pub_poking = rospy.Publisher('/robot/stick_move_joint_position_controller/command', Float64, queue_size = 1)

        # subscriber, timer
        self.sub_joy = rospy.Subscriber("joy", Joy, self.cb_joy, queue_size=1)
        self.sub_odom = rospy.Subscriber(
            "odom_in", PoseStamped, self.cb_odom, queue_size=1)
        self.sub_laser = rospy.Subscriber(
            "/RL/scan_label",  LaserScan, self.cb_laser, queue_size=1)
        self.initial = rospy.ServiceProxy("/husky/init", Trigger)
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

                tra = {'environment' : "room_door", "policy": "Pokingbot", "trajectories" : self.total_traj}

                with open(os.path.join(self.my_dir,"../pokingbot_trajectory.yaml"), "w") as f:

                    yaml.dump(tra, f)

                with open(os.path.join(self.my_dir,"../pokingbot_result.yaml"), "w") as f:

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
            self.start = time.time()
        elif msg.buttons[back_button] == 1 and self.auto:
            self.auto = 0
            rospy.loginfo('go manual')

    def cb_goal(self, msg):
        if msg.header.frame_id != self.frame:
            self.goal = None
            return

        self.goal = np.array([
            msg.pose.position.x, msg.pose.position.y])

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
        
        scan = np.array(msg.ranges)
        scan = np.clip(scan, 0, self.max_dis)
        intensities = np.array(msg.intensities)
        for i in range(len(intensities)):
            if intensities[i]==2: intensities[i]=1

        if self.state_stack is None:
            self.state_stack = np.tile(scan,(4,1))
            self.state_label_stack = np.tile(intensities,(4,1)) #self.state_num=4
        else:
            self.state_stack[:-1] = self.state_stack[1:]
            self.state_stack[-1] = scan
            self.state_label_stack[:-1] = self.state_label_stack[1:]
            self.state_label_stack[-1] = intensities

    def check(self):

        if self.pos_track is None:
            return "nothing"
        else:
            pos = True
        if self.state_stack is None:
            return "nothing"
        else:
            state = True

        if(pos and state):
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
            laser = self.state_stack.reshape(-1)
            label = self.state_label_stack.reshape(-1)
            track = self.pos_track.reshape(-1)
            state = laser
            state = np.append(state, label)
            state = np.append(state, track)

            state = tf.convert_to_tensor([state], dtype=tf.float32)

            action = self.policy_network(state)[0].numpy()
            self.last_omega = self.omega_gamma * \
                action[1] + (1-self.omega_gamma)*self.last_omega

            cmd = Twist()
            cmd.linear.x = action[0]*self.action_scale['linear']
            cmd.angular.z = self.last_omega * \
                self.action_scale['angular']

            self.pub_cmd.publish(cmd)
            self.pub_poking.publish(action[2])
            
            dis = np.linalg.norm(self.goal-self.last_pos)
            if dis < 0.8:
                rospy.loginfo("goal reached")
                self.total_traj.append(tra[0:-1:50])
                self.success += 1
                break

            if((time.time() - begin) >= 60):
                self.total_traj.append(tra[0:-1:50])
                break

if __name__ == "__main__":
    rospy.init_node("goal_nav_rl")
    goalNav = GoalNav()
    rospy.spin()
