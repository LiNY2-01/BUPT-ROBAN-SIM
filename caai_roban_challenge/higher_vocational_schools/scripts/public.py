#!/usr/bin/env python
# -*- coding: utf-8 -*-

from ast import Pass
import sys
import os
import math
import time
import numpy as np
import rospy
import rospkg
import tf
import yaml
from std_msgs.msg import *
from geometry_msgs.msg import *
import bodyhub_action as bodact
from algorithm import pidAlgorithm as pidAlg


sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *
from frames import RobanFrames
SCRIPTS_PATH=os.path.split(sys.argv[0])[0]

# POSE_TOPIC = "/sim/torso/pose"
POSE_TOPIC = "/initialpose"
STEP_LEN_MAX = [0.06, 0.02, 10]


class PublicNode(bodact.Action):
    def __init__(self,NODE_NAME,CONTROL_ID):
        super(PublicNode, self).__init__(NODE_NAME, CONTROL_ID)
        self.__gait_cmd_pub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=2)
        rospy.Subscriber(POSE_TOPIC, PoseWithCovarianceStamped, self.pose_callback)

        self.__debug = False
        self.current_pose = [0, 0, 0]
        self.pose_update = False
        self.err_threshold = [0.12, 0.12, 10]

    def quart_to_rpy(self, w, x, y, z):
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*x))
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))
        return [r, p, y]


    def toRPY(self, pose):
        return tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    def pose_callback(self, msg):
        p = msg.pose.pose.position
        x, y = p.x, p.y
        _, _, yaw = self.toRPY(msg.pose.pose)
        old_pos=self.current_pose
        self.current_pose = [p.x, p.y, yaw * 180.0 / math.pi]
        if self.__debug:
            print ('pose:', self.current_pose)
        if ((old_pos[0]-self.current_pose[0])**2+(old_pos[1]-self.current_pose[1])**2)>=1 or abs(old_pos[2]-self.current_pose[2])>50:
            return#滤掉高频跳点
        self.pose_update = True

    def rot_mat(self, theta):
        theta = theta * math.pi / 180.0
        return np.array([[math.cos(theta), -math.sin(theta)], 
                        [math.sin(theta), math.cos(theta)]], dtype=np.float32)

    def path_tracking(self, path_point, pos_wait_mode=0,ignore_angle=False,pos_exit_high_acc=True,wait_time=0.4):
        '''
        pos_wait_mode=1:等待机器人站稳后才进行定位,精度高但慢
        ignore_angle模式:用于回程运动,忽略标记点的方向
        wait_time:等待机器人定位稳定的时间
        pos_exit_high_acc:确保终点精度高,调节时间会增加
        '''
        step_len = [0, 0, 0]
        rot_adjust = False
        path_marker_index, marker_num = 0, len(path_point)
        while not rospy.is_shutdown():
            rospy.wait_for_message('/requestGaitCommand', Bool, 10)

            if self.pose_update == False:
                print ('location not updated!')
                time.sleep(0.5)
                continue
            for i in range(3):
                step_len[i] = path_point[path_marker_index][i] - self.current_pose[i]
            
            if (pos_wait_mode == 1):
                time.sleep(wait_time)
            
            v = np.dot(np.linalg.inv(self.rot_mat(self.current_pose[2])), np.array([step_len[0], step_len[1]])).tolist()
            if (pos_wait_mode == 1) and not ignore_angle and v[0]**2+v[1]**2<=(self.err_threshold[0]*1.8)**2:
                w = step_len[2]
            else:
                w = (math.atan2(step_len[1], step_len[0]) * 180.0 / math.pi) - self.current_pose[2]
            w = (w-360.0) if w >= 180.0 else w
            w = (w+360.0) if w <= -180.0 else w
            step_len = [v[0], v[1], w]
            print("total steplen:",step_len)
            self.pose_update = False

            pos_err_scale, rot_err_scale = 1.0, 10.0
            if (pos_wait_mode == 1) and (path_marker_index == marker_num-1) and pos_exit_high_acc:#end pos 
                pos_err_scale, rot_err_scale = 0.4, 0.2
            if (abs(step_len[0]) < (self.err_threshold[0]*pos_err_scale)) and \
            (abs(step_len[1]) < (self.err_threshold[1]*pos_err_scale)) and \
            (abs(step_len[2]) < (self.err_threshold[2]*rot_err_scale) or ignore_angle):
                path_marker_index = path_marker_index + 1
                print ('path marker', path_marker_index, '/', marker_num)
                if path_marker_index >= marker_num:
                    self.bodyhub.wait_walking_done()
                    break

            if abs(step_len[2]) > 20:
                rot_adjust = True
            else:
                rot_adjust = False

            for i in range(3):
                step_len[i] = STEP_LEN_MAX[i] if step_len[i] > STEP_LEN_MAX[i] else step_len[i]
                step_len[i] = -STEP_LEN_MAX[i] if step_len[i] < -STEP_LEN_MAX[i] else step_len[i]
            if rot_adjust:
                self.__gait_cmd_pub.publish(data=[0.0, 0, step_len[2]])
            else:
                self.__gait_cmd_pub.publish(data=step_len)
        self.bodyhub.wait_walking_done()
    
    def set_head_rot(self, head_rot):
        keyframes = [
            ([0, -1, 16, -34, -17, -1, 0, 1, -16, 34, 17, 1, 0, -70, -15, 0, 70, 15, 0, 0, head_rot[0], head_rot[1]], 500, 0)
        ]
        self.movement.linearMove(keyframes)
        self.head_pitch = head_rot[1]

    def stand_straight(self, head_rot):
        keyframes = [
            ([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -70, -15, 0, 70, 15, 0, 0, head_rot[0], head_rot[1]], 500, 0)
        ]
        self.movement.linearMove(keyframes)
        self.head_pitch = head_rot[1]

    def take(self):
        self.set_arm_mode(0)
        self.bodyhub_ready()
        self.frame_action(RobanFrames.take_frames)

    def frame_action(self,key_frame):
        try:
            client_action.custom_action([],key_frame)
        except Exception as err:
            serror(err)
        finally:
            pass 

    def turn_around(self,d_right=False,step=14):
        self.bodyhub.walking_n_steps([0,0,10 if d_right else -10],step)


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

    
    