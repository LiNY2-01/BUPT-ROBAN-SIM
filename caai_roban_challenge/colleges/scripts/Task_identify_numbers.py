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
from ros_vision_node.srv import DigitalDetectSrv
from cv_bridge import CvBridge, CvBridgeError
import json

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *
from frames import RobanFrames
from public import PublicNode
SCRIPTS_PATH=os.path.split(sys.argv[0])[0]

NODE_NAME = 'identify_numbers_node'
CONTROL_ID = 2
SNAPSHOT_HEAD_URL = 'http://localhost:8080/snapshot?topic=/camera/color/image_raw'
POINTS = [[162, 60], [481, 60], [40, 380], [620, 380]]
PERSPECT_IMSHOW_FRAME_OFFSET = {"x": 100, "y": 100}
MODEL_NUMBER_IMG_DIR_PATH = '/home/lemon/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/caai_roban_challenge/colleges/number_img/'
MODEL_NUMBER_IMG_FILEPATH = [MODEL_NUMBER_IMG_DIR_PATH + '1.jpg', MODEL_NUMBER_IMG_DIR_PATH + '2.jpg', MODEL_NUMBER_IMG_DIR_PATH + '3.jpg', MODEL_NUMBER_IMG_DIR_PATH + '4.jpg'] 
with open(os.path.join(SCRIPTS_PATH,"slam_identify_num.yaml"),"r")as f:
    SLAM_POINT=yaml.load(f)
    print(SLAM_POINT)
Adjust_len=0.2 #机器人在slam地图中往前看和往后看的距离差
is_debug = False
LABEL_TO_DIGITAL = {0: 4, 1: 3, 2: 1, 3: 2}
DETECT_RESULT_BOX_COLOR = {1: (255, 0, 0), 2: (0, 255, 0), 3: (0, 0, 255), 4: (0, 0, 0)}
ACCURACY = 0.5
class Identifier():
    def __init__(self):
        self.cv_bridge = CvBridge()

    def get_eye_camera_img(self):
        try:
            url_response = url.urlopen(SNAPSHOT_HEAD_URL)
        except IOError as err:
            rospy.logerr(err)
            exit(2)
        head_camera_img_data = np.asarray(bytearray(url_response.read()), dtype='uint8')
        head_camera_img = cv.imdecode(head_camera_img_data, cv.IMREAD_COLOR)
        return head_camera_img

    def detect_digital(self):
        global is_debug
        while not rospy.core.is_shutdown_requested():
            available_labels = []
            origin_img = self.get_eye_camera_img()
            try:
                origin_img_msg = self.cv_bridge.cv2_to_imgmsg(origin_img, "bgr8")
                rospy.wait_for_service("ros_vision_node/digital_detection", timeout=10)
            except CvBridgeError as err:
                rospy.logerr(err)
            except rospy.ROSException:
                rospy.logerr("wait for ros_vision_node/digital_detection timeout")
            digital_detection_client = rospy.ServiceProxy("ros_vision_node/digital_detection", DigitalDetectSrv)
            client_result = digital_detection_client(origin_img_msg)
            detect_result = client_result.detect_result
            result = json.loads(detect_result, encoding="utf-8")
            map_result = result["map_item"]
            sorted_map_result_by_accuracy = sorted(map_result, key=(lambda x:x[1]), reverse=True)
            digitals = list(LABEL_TO_DIGITAL.keys())
            for item in sorted_map_result_by_accuracy:
                if item[1] > ACCURACY and item[0] in digitals:
                    available_labels.append(item)
                    digitals.remove(item[0])
                if len(digitals) == 0: break
            if len(available_labels) < 4:
                rospy.logerr("只检测出 {} 个数字".format(len(available_labels)))
                continue
            else:
                for index, item in enumerate(available_labels):
                    digital = LABEL_TO_DIGITAL[int(item[0])]
                    available_labels[index][0] = digital
                if is_debug:
                    for digital in available_labels:
                        rospy.loginfo("is: {}\n坐标: {}".format(digital[0], [digital[2], digital[3], digital[4], digital[5]]))
                        cv.rectangle(origin_img, (int(digital[2]), int(digital[3])), (int(digital[4]), int(digital[5])), DETECT_RESULT_BOX_COLOR[digital[0]], 2)
                        text = "{}  acc: {}".format(digital[0], round(digital[1], 3))
                        cv.putText(origin_img, text, (int(digital[2]), int(digital[3])), cv.FONT_HERSHEY_DUPLEX, 0.6, (255, 0, 255), 2)
                    cv.imshow("detect_digital_result", origin_img)
                    cv.waitKey(0)
                return available_labels

    def filter_frames_position(self,frames_list):
        frames_array_sum = [0, 0]
        frames_array = np.array(frames_list)
        frames_array_lenght = len(frames_array)
        for frame_data in frames_array:
            frames_array_sum = frames_array_sum + frame_data
        center_position_of_frames = frames_array_sum / frames_array_lenght
        left_index = []
        right_index = []
        map_positoin_to_numbers = {}

        for index in range(frames_array_lenght):
            if (frames_array[index][0]) < center_position_of_frames[0]:
                left_index.append(index)
            else:
                right_index.append(index)

        if frames_array[left_index[0]][1] > frames_array[left_index[1]][1]:
            map_positoin_to_numbers[left_index[0] + 1] = 'Lower_left'
            map_positoin_to_numbers[left_index[1] + 1] = 'Upper_left'
        elif frames_array[left_index[0]][1] < frames_array[left_index[1]][1]:
            map_positoin_to_numbers[left_index[0] + 1] = 'Upper_left'
            map_positoin_to_numbers[left_index[1] + 1] = 'Lower_left'

        if frames_array[right_index[0]][1] > frames_array[right_index[1]][1]:
            map_positoin_to_numbers[right_index[0] + 1] = 'Lower_right'
            map_positoin_to_numbers[right_index[1] + 1] = 'Upper_right'
        elif frames_array[right_index[0]][1] < frames_array[right_index[1]][1]:
            map_positoin_to_numbers[right_index[0] + 1] = 'Upper_right'
            map_positoin_to_numbers[right_index[1] + 1] = 'Lower_right'
        return map_positoin_to_numbers

    def get_digital_image_center(self, available_labels):
        center_coordinate_of_digital = []
        available_labels_sorted = sorted(available_labels, key=(lambda x:x[0]))
        for item in available_labels_sorted:
            center_of_digital = ((item[2] + item[4]) / 2, (item[3] + item[5]) / 2)
            center_coordinate_of_digital.append(center_of_digital)
        return center_coordinate_of_digital

    def get_numbers_pos(self):
        map_positoin_to_numbers = {}
        available_labels = self.detect_digital()
        center_coordinate_of_digital = self.get_digital_image_center(available_labels)
        map_positoin_to_numbers = self.filter_frames_position(center_coordinate_of_digital)
        return map_positoin_to_numbers

    def debug(self):
        global is_debug
        is_debug = True
        map_positoin_to_numbers = self.get_numbers_pos()
        print(map_positoin_to_numbers)
class Task_Identify_numbers(PublicNode):
    def __init__(self,nodename=NODE_NAME,control_id=CONTROL_ID):
        super(Task_Identify_numbers, self).__init__(nodename, control_id)
        self.identifier=Identifier()
    
    def path_plan(self,map_positoin_to_numbers):
        # [SLAM_POINT[map_positoin_to_numbers[i]][0] for i in range(1,5)]#生成路径点
        print([SLAM_POINT[map_positoin_to_numbers[i]][0] for i in range(1,5)])
        path_points=[]
        p_u=SLAM_POINT["Upper_left"][0]
        p_d=SLAM_POINT["Lower_left"][0]
        
        d_angle=math.atan2(p_u[0]-p_d[0],p_u[1]-p_d[1])
        dx=-math.sin(d_angle)*Adjust_len
        dy=-math.cos(d_angle)*Adjust_len
        print(d_angle,dx,dy)
        
        if "upper"in map_positoin_to_numbers[2].lower():#第二点在上方
            print("upper")
            for i in range(1,5):
                slampos=SLAM_POINT[map_positoin_to_numbers[i]][0]
                if i==3:
                    slampos[0]+=dx#加上偏置量
                    slampos[1]+=dy
                path_points.append(slampos)
        else:
            for i in range(1,5):
                slampos=SLAM_POINT[map_positoin_to_numbers[i]][0]
                if i==2:
                    slampos[0]+=dx/2#加上偏置量
                    slampos[1]+=dy/2
                path_points.append(slampos)

        return path_points

    def debug(self):
        self.bodyhub_ready()
        self.frame_action(RobanFrames.squat_frames)
        time.sleep(0.5)

        map_positoin_to_numbers =  self.identifier.get_numbers_pos()
        print(map_positoin_to_numbers)
        task1_path = self.path_plan(map_positoin_to_numbers)#路径规划
        print(task1_path)

    def start_identify_numbers(self):
        self.bodyhub_ready()
        self.set_arm_mode(1)
        # self.bodyhub_walk()#导航到起点
        # self.path_tracking(SLAM_POINT["TASK1_origin_POINT"],mode=1)
        self.bodyhub_ready()
        self.frame_action(RobanFrames.squat_frames)
        time.sleep(0.5)
        
        map_positoin_to_numbers =  self.identifier.get_numbers_pos()
        task1_path = self.path_plan(map_positoin_to_numbers)#路径规划
        print(task1_path)
        self.bodyhub_ready()
        self.set_head_rot([0, 10])
        self.bodyhub_walk()
        self.path_tracking(task1_path,pos_wait_mode=1,ignore_angle=True)

if __name__ == '__main__':
    task=Task_Identify_numbers()
    if len(sys.argv) >=2 and (sys.argv[1] == 'debug'):
        task.debug()
    else:
        task.start_identify_numbers()
        # Identifier().get_numbers_pos()