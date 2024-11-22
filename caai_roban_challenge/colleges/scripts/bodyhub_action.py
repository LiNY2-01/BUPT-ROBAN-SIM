#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
import rospkg
import numpy as np
from mediumsize_msgs.srv import *
from motion import bodyhub_client as bodycli
from motion import TrajectoryPlan as tPlan

class Movement:
    '''
    生成、发送动作轨迹
    '''
    def __init__(self, bodyhub_cli):
        self.bodyhub = bodyhub_cli

    def jointMoveToPosition(self, duration, ids, current_pos, goal_pos):
        sequence = []
        joint_pos = [float(0) for i in range(len(ids))]
        for i in range(len(ids)):
            sequence.append(list(np.linspace(current_pos[i], goal_pos[i], duration/0.01)))
        for y in range(len(sequence[0])):
            for x in range(len(ids)):
                joint_pos[x] = sequence[x][y]
            self.bodyhub.set_joint_position(ids, joint_pos)

    def linearMove(self, keyframes):
        ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
        keyframe = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 1000, 0]
        keyframe[0] = self.bodyhub.get_joint_position(ids)
        keyframes.insert(0, keyframe)

        for idx in range(1, len(keyframes)):
            self.jointMoveToPosition(keyframes[idx][1]/1000.0, ids, keyframes[idx-1][0], keyframes[idx][0])
            if keyframes[idx][2] > 0:
                self.bodyhub.wait_joint_done(True)
                time.sleep(keyframes[idx][2]/1000.0)
            else:
                self.bodyhub.wait_joint_done()
        self.bodyhub.wait_joint_done(True)

    def sendBezierTraj(self, trajectoryPoint):
        ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
        for m in range(len(trajectoryPoint[0])):
            jointPosition = []
            for n in range(len(trajectoryPoint)):
                jointPosition.append(trajectoryPoint[n][m].y)
            self.bodyhub.set_joint_position(ids, jointPosition)

    def bezierMove(self, keyframes):
        ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
        keyframe = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 1000, 0]
        keyframe[0] = self.bodyhub.get_joint_position(ids)
        keyframes.insert(0, keyframe)

        tpObject = tPlan.TrajectoryPlanning(22, 10.0)
        tpObject.setInterval(keyframes[1][1])
        tpObject.planningBegin(keyframes[0][0], keyframes[1][0])

        for idx in range(2, len(keyframes)):
            tpObject.setInterval(keyframes[idx][1])
            trajectoryPoint = tpObject.planning(keyframes[idx][0])
            self.sendBezierTraj(trajectoryPoint)
            if keyframes[idx][2] > 0:
                self.bodyhub.wait_joint_done(True)
                time.sleep(keyframes[idx][2]/1000.0)
            else:
                self.bodyhub.wait_joint_done()

        trajectoryPoint = tpObject.planningEnd()
        self.sendBezierTraj(trajectoryPoint)
        self.bodyhub.wait_joint_done(True)

class Action(object):
    '''
    robot action
    '''

    def __init__(self, name, ctl_id):
        rospy.init_node(name, anonymous=True)
        time.sleep(0.2)
        rospy.on_shutdown(self.__ros_shutdown_hook)

        self.bodyhub = bodycli.BodyhubClient(ctl_id)
        self.movement = Movement(self.bodyhub)

    def __ros_shutdown_hook(self):
        if self.bodyhub.reset() == True:
            rospy.loginfo('bodyhub reset, exit')
        else:
            rospy.loginfo('exit')

    def bodyhub_ready(self):
        if self.bodyhub.ready() == False:
            rospy.logerr('bodyhub to ready failed!')
            rospy.signal_shutdown('error')
            time.sleep(1)
            exit(1)

    def bodyhub_walk(self):
        if self.bodyhub.walk() == False:
            rospy.logerr('bodyhub to walk failed!')
            rospy.signal_shutdown('error')
            time.sleep(1)
            exit(1)

    def set_arm_mode(self, mode):
        '''
        set bodyhub arm mode
        mode: 0 action mode, 1 gait pattern
        '''
        try:
            rospy.wait_for_service('MediumSize/BodyHub/armMode',  10)
        except:
            print ('error: wait_for_service armMode!')
            return None
        client = rospy.ServiceProxy('MediumSize/BodyHub/armMode', SetAction)
        response = client(mode, 'set arm mode')
        return response.result
        

    def __start(self):
        '''
        action example
        '''
        # 步态
        self.bodyhub_walk()
        self.bodyhub.walking_the_distance(1.0, 0.0, 0.0)
        self.bodyhub.wait_walking_done()
        # 动作
        self.bodyhub_ready()
        keyframes = [
            ([0, -1, 16, -34, -17, -1, 0, 1, -16, 34, 17, 1, 0, -70, -15, 0, 70, 15, 0, 0, 0, 25], 500, 100),
            ([0, -12, 16, -34, -17, -12, 0, -11, -16, 34, 17, -11, 0, -70, -15, 0, 70, 15, 0, 0, 0, 25], 800, 0),
            ([-5, -14, 35, -70, -35, -14, -5, -10, -16, 34, 17, -12, 0, -70, -15, 0, 70, 15, 0, 0, 0, 25], 500, 0),
            ([-10, -14, 30, -75, -30, -14, -10, -10, -16, 34, 17, -12, 0, -70, -15, 0, 70, 15, 0, 0, 0, 25], 500, 200),
            ([5, -14, 55, -40, 15, -14, 5, -10, -16, 34, 17, -12, 0, -70, -15, 0, 70, 15, 0, 0, 0, 25], 400, 200),
            ([-5, -14, 35, -70, -35, -14, -5, -10, -16, 34, 17, -12, 0, -70, -15, 0, 70, 15, 0, 0, 0, 25], 500, 0),
            ([0, -14, 18, -36, -18, -14, 0, -11, -16, 34, 17, -11, 0, -70, -15, 0, 70, 15, 0, 0, 0, 25], 1000, 0),
            ([0, -1, 16, -34, -17, -1, 0, 1, -16, 34, 17, 1, 0, -70, -15, 0, 70, 15, 0, 0, 0, 25], 1000, 100)
        ]
        self.movement.linearMove(keyframes)
