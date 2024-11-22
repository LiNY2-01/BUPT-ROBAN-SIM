#!/usr/bin/env python
# -*- coding: utf-8 -*-

from ast import Pass
import sys
import os
import time
import rospy
import rospkg
from std_msgs.msg import *
from geometry_msgs.msg import *

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *
from Task_clear_obstruction import Task_clear_obstruction
from Task_identify_numbers_n import Task_Identify_numbers
from Task_path_tracking import Task_path_tracking
SCRIPTS_PATH=os.path.split(sys.argv[0])[0]

NODE_NAME = 'race_node'
# POSE_TOPIC = "/sim/torso/pose"
POSE_TOPIC = "/initialpose"
CONTROL_ID = 2


class RaceNode(Task_clear_obstruction,Task_Identify_numbers,Task_path_tracking):
    def __init__(self,start_index):
        super(RaceNode, self).__init__(NODE_NAME, CONTROL_ID)
        self.start_index = start_index 

    def debug(self):
        self.set_arm_mode(0)
        self.bodyhub_ready()
        self.set_head_rot([0, 10])
        self.__debug = True
        while not rospy.is_shutdown():
            time.sleep(0.01)
        self.bodyhub_ready()
        self.set_head_rot([0, 0])
        self.set_arm_mode(1)

    def start(self):
        self.bodyhub_ready()
        self.set_arm_mode(1)

        print(f"从第 {self.start_index} 关开始运行")
        if self.start_index == 1:
            self.start_identify_numbers_n()
            self.start_index += 1 
        if self.start_index == 2:
            self.start_path_tracking()
            self.start_index += 1
        if self.start_index == 3:
            self.start_clear_obstruction()      
        if self.start_index != 1 and self.start_index != 2 and self.start_index != 3:
            print("Invalid level number.") 
            
        rospy.signal_shutdown('exit')
    
if __name__ == '__main__':
    if len(sys.argv) >= 3 and sys.argv[1] == 'start':
        start_index = int(sys.argv[2])
        RaceNode(start_index).start()
    elif len(sys.argv) >= 2 and sys.argv[1] == 'debug':
        print("Invalid command line arguments.")