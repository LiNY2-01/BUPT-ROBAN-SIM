#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2 as cv
import urllib as url
import sys
import os
import math
import time
import numpy as np
import rospy
import rospkg
import yaml
from std_msgs.msg import *
from geometry_msgs.msg import *

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *
from frames import RobanFrames
from public import PublicNode
SCRIPTS_PATH=os.path.split(sys.argv[0])[0]

NODE_NAME = 'identify_numbers_n_node'
CONTROL_ID = 2
SNAPSHOT_HEAD_URL = 'http://localhost:8080/snapshot?topic=/camera/color/image_raw'
POINTS = [[135, 132], [487, 136], [0, 422], [640, 422]]
ROI_RANGE = [88, 124, 440, 270]
MIN_AREA_OF_NUMBER_FRAME = 5500
MODEL_NUMBER_IMG_DIR_PATH = '/home/lemon/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/caai_roban_challenge/higher_vocational_schools/number_img/'
MODEL_NUMBER_IMG_FILEPATH = MODEL_NUMBER_IMG_DIR_PATH + '1.jpg' 
with open(os.path.join(SCRIPTS_PATH,"slam_identify_num_n.yaml"),"r")as f:
    SLAM_POINT=yaml.load(f)
    print(SLAM_POINT)
is_debug = False
class Identifier():
    def get_num_n_pos(self, n):
        return slam.IdentifyDigit(n)

class Task_Identify_numbers(PublicNode):
    def __init__(self,nodename=NODE_NAME,control_id=CONTROL_ID):
        super(Task_Identify_numbers, self).__init__(nodename, control_id)
        self.identifier=Identifier()
    def start_identify_numbers_n(self):
        self.bodyhub_ready()
        self.set_arm_mode(1)
        # self.bodyhub_walk()#导航到起点
        # self.path_tracking(SLAM_POINT["TASK1_origin_POINT"],mode=1)
        self.bodyhub_ready()
        self.frame_action(RobanFrames.squat_frames)
        time.sleep(0.5)
        target_number = 1
        pos=self.identifier.get_num_n_pos(target_number)
        if pos in SLAM_POINT:
            self.bodyhub_ready()
            self.set_head_rot([0, 10])
            self.bodyhub_walk()
            self.path_tracking(SLAM_POINT[pos],pos_wait_mode=1)
            self.bodyhub_ready()
            time.sleep(2)
        else:
            print('digit {} not found!'.format(target_number))

if __name__ == '__main__':
    Task_Identify_numbers().start_identify_numbers_n()