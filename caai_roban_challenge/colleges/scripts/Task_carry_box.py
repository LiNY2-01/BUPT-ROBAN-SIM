#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
import time
import numpy as np
import rospy
import rospkg
import yaml
from std_msgs.msg import *
from geometry_msgs.msg import *

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *
from public import PublicNode
SCRIPTS_PATH=os.path.split(sys.argv[0])[0]

NODE_NAME = 'carry_box_node'
CONTROL_ID = 2
with open(os.path.join(SCRIPTS_PATH,"slam_carry_box.yaml"),"r")as f:
    SLAM_POINT=yaml.load(f)
    print(SLAM_POINT)

class Task_carry_box(PublicNode):
    def __init__(self,nodename=NODE_NAME,control_id=CONTROL_ID):
        super(Task_carry_box, self).__init__(nodename, control_id)

    def carry(self, num):
        self.bodyhub_walk()
        self.set_arm_mode(0)
        self.path_tracking(SLAM_POINT["box{}_pos".format(num)], pos_wait_mode=1)#go to box_n_pos

        self.bodyhub_ready()
        self.set_arm_mode(0)
        self.take()#take up

        self.bodyhub_walk()
        self.bodyhub.walking_n_steps([-0.08, 0.0, 0.0], 3)
        self.bodyhub.wait_walking_done()

        self.turn_around(d_right=True if num==3 else False)
        self.bodyhub.wait_walking_done()

        #go back to origin pos
        self.path_tracking(SLAM_POINT["carry_box_origin_pos"], pos_wait_mode=1,ignore_angle=True)
        self.bodyhub_ready()

        self.set_arm_mode(1)
        self.bodyhub_walk()
        self.turn_around(d_right=False if num==3 else True,step=18)
        self.bodyhub_ready()

        # self.set_head_rot([0, 0])
        # self.set_arm_mode(1)
    def debug(self):
        self.set_arm_mode(1)
        self.bodyhub_walk()
        self.path_tracking(SLAM_POINT["carry_box_origin_pos"])
        for i in range(2,4):
            self.carry(i)
    def start_carry_box(self):
        self.set_arm_mode(1)
        self.bodyhub_walk()
        self.path_tracking(SLAM_POINT["carry_box_origin_pos"])
        for i in range(1,4):
            self.carry(i)
if __name__ == '__main__':
    if len(sys.argv)>1:
        Task_carry_box().debug()
    else:
        Task_carry_box().start_carry_box()