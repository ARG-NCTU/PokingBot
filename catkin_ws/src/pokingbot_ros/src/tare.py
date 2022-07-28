#! /usr/bin/env python3

import os
import rospy
import time 
import yaml
import numpy as np
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *

class GoalNav(object):
    def __init__(self):
        super().__init__()

        self.my_dir = os.path.abspath(os.path.dirname(__file__))
         # read yaml
        with open(os.path.join(self.my_dir,"../goal.yaml"), 'r') as f:
            data = yaml.load(f)

        self.goal_totoal = data['goal']

        # metric
        self.count = 0
        self.total = len(self.goal_totoal)
        self.success = 0
        self.coi = 0
        self.cnt = 0
        self.collision_states = False

        self.initial = rospy.ServiceProxy("/husky_ur5/init", Trigger)
        self.goal_pub = rospy.Publisher("/tare/goal", PoseStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/state', String, queue_size=10)
        self.get_robot_pos = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.sub_collision = rospy.Subscriber("/robot/bumper_states", ContactsState, self.cb_collision, queue_size=1)
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

                with open(os.path.join(self.my_dir,"../tare_result.yaml"), "w") as f:

                    yaml.dump(d, f)
                
                rospy.loginfo('End')
                break
            else:
                self.goal = self.goal_totoal[self.count]
                self.count += 1

                self.inference()

    def cb_collision(self, msg):
        if self.collision_states == True:
            if msg.states == [] and self.cnt > 1000:
                self.collision_states = False
            else:
                self.cnt += 1
        elif msg.states != [] and self.cnt == 0:
            self.collision_states = True
            self.coi += 1
        else:
            self.collision_states = False
            self.cnt = 0

    def inference(self):

        # publish goal to tare
        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.pose.position.x = self.goal[0]
        pose.pose.position.y = self.goal[1]

        self.goal_pub.publish(pose)

        # publish state to enable tare start
        self.state_pub.publish("nav")

        begin = time.time()

        # check robot navigate to goal
        while(1):
            robot_pose = self.get_robot_pos("robot","")

            x, y = robot_pose.pose.position.x, robot_pose.pose.position.y
            dis = np.linalg.norm(self.goal - np.array([x, y])) 

            if(dis < 0.8):
                
                self.success += 1
                self.state_pub.publish("stop")
                break

            if((time.time() - begin) >= 180):
                self.state_pub.publish("stop")
                break

            cur_x, cur_y = x,y

if __name__ == "__main__":
    rospy.init_node("loop_node")
    goalNav = GoalNav()
    rospy.spin()