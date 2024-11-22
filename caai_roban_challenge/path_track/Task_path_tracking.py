#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
import math
import time
import numpy as np
import rospy
import rospkg
import yaml

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
sys.path.append(rospkg.RosPack().get_path('ros_higher_vocational_training_platform') + "/scripts")


from std_msgs.msg import *
from geometry_msgs.msg import *
import bodyhub_action as bodact

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
# from lejulib import *
from frames import RobanFrames
from public import PublicNode
SCRIPTS_PATH=os.path.split(sys.argv[0])[0]

NODE_NAME = 'path_tracking_node'
CONTROL_ID = 6
with open(os.path.join(SCRIPTS_PATH,"slam_path_tracking.yaml"),"r")as f:
    SLAM_POINT=yaml.load(f)
    print(SLAM_POINT)
class Task_path_tracking(PublicNode):
    def __init__(self,nodename=NODE_NAME,control_id=CONTROL_ID):
        super(Task_path_tracking, self).__init__(nodename, control_id)

    def start_path_tracking(self):
        self.set_arm_mode(1)
        self.bodyhub_walk()
        # self.path_tracking(SLAM_POINT["path_tracking_points"],pos_wait_mode=1,wait_time=0.1,pos_exit_high_acc=False)
        self.path_tracking(pos_wait_mode=1,wait_time=0.2,pos_exit_high_acc=False)

if __name__ == '__main__':
    task=Task_path_tracking()
    task.start_path_tracking()
