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
from nav_msgs.msg import Path
import bodyhub_action as bodact
from algorithm import pidAlgorithm as pidAlg


sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *
from frames import RobanFrames
SCRIPTS_PATH=os.path.split(sys.argv[0])[0]

# POSE_TOPIC = "/sim/torso/pose"
# POSE_TOPIC = "/initialpose"
POSE_TOPIC = "/sim/torso/PoseCov"
GOAL_TOPIC = "/sim/torso/PoseCov"
PATH_TOPIC = "/path"
STEP_LEN_MAX = [0.06, 0.06, 3]


class PublicNode(bodact.Action):
    def __init__(self,NODE_NAME,CONTROL_ID):
        super(PublicNode, self).__init__(NODE_NAME, CONTROL_ID)
        self.__gait_cmd_pub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=2)
        self.arrive_pub = rospy.Publisher("/roban_arrived",Bool,queue_size=1)

        rospy.Subscriber(POSE_TOPIC, PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber(PATH_TOPIC, Path, self.path_callback)
        self.__debug = False
        self.current_pose = [0, 0, 0]
        self.pose_update = False
        self.err_threshold = [0.2, 0.2, 2]
        self.path_points = []
        self.move_target = None
        self.endpos = False

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
    def path_callback(self,msg): # type: Path
        self.path_points = msg.poses
        if len(self.path_points) < 3:
            self.endpos = True
            target_pt = self.path_points[-1].pose
        else :
            self.endpos = False
            target_pt = self.path_points[2].pose
        x,y = target_pt.position.x , target_pt.position.y
        _ , _ , yaw = self.toRPY(target_pt)
        self.move_target = [x,y,yaw * 180.0 / math.pi]
    def rot_mat(self, theta):
        theta = theta * math.pi / 180.0
        return np.array([[math.cos(theta), -math.sin(theta)], 
                        [math.sin(theta), math.cos(theta)]], dtype=np.float32)

    def path_tracking(self,  pos_wait_mode=0,ignore_angle=False,pos_exit_high_acc=True,wait_time=0.4):
        '''
        pos_wait_mode=1:等待机器人站稳后才进行定位,精度高但慢
        ignore_angle模式:用于回程运动,忽略标记点的方向
        wait_time:等待机器人定位稳定的时间
        pos_exit_high_acc:确保终点精度高,调节时间会增加
        '''
        step_len = [0, 0, 0]
        now_error = [0, 0, 0]

        rot_adjust = False
        # path_marker_index, marker_num = 0, len(path_point)
        while not rospy.is_shutdown():
            rospy.wait_for_message('/requestGaitCommand', Bool, 40)

            if self.pose_update == False:
                print ('location not updated!')
                time.sleep(0.5)
                continue
            if self.move_target is None:
                # print ('location not updated!')
                time.sleep(2)
                continue
            
            for i in range(3):
                now_error[i] = self.move_target[i] - self.current_pose[i]
                step_len[i] = self.move_target[i] - self.current_pose[i]
            
            if (pos_wait_mode == 1):
                # time.sleep(wait_time)
                rospy.sleep(wait_time)
            
            v = np.dot(np.linalg.inv(self.rot_mat(self.current_pose[2])), np.array([step_len[0], step_len[1]])).tolist()
            if (pos_wait_mode == 1) and not ignore_angle and self.endpos:
                w = step_len[2]
            else:
                w = (math.atan2(step_len[1], step_len[0]) * 180.0 / math.pi) - self.current_pose[2]
            print("target       :",[self.move_target[0],self.move_target[1],w])
            
            w = (w-360.0) if w >= 180.0 else w
            w = (w+360.0) if w <= -180.0 else w

            
            step_len = [v[0], v[1], w]
            
            print("total steplen:",step_len)
            self.pose_update = False

            pos_err_scale, rot_err_scale = 1.0, 1.0
            if (pos_wait_mode == 1) and (self.endpos ) and pos_exit_high_acc:#end pos 
                pos_err_scale, rot_err_scale = 0.8, 0.2
            if (abs(now_error[0]) < (self.err_threshold[0]*pos_err_scale)) and \
            (abs(now_error[1]) < (self.err_threshold[1]*pos_err_scale)) and \
            (abs(now_error[2]) < (self.err_threshold[2]*rot_err_scale) or ignore_angle) and\
             self.endpos:
                # path_marker_index = path_marker_index + 1
                print ("arrived")
                # if path_marker_index >= marker_num:
                self.bodyhub.wait_walking_done()
                arrmsg = Bool()
                arrmsg.data = True
                self.arrive_pub.publish(arrmsg)
                # break
                # step_len = [0,0,0]
            else:
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

    
    