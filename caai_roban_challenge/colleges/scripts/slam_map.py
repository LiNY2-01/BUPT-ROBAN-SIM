# -*- coding: utf-8 -*-

import sys
import os
import math
import time
import numpy as np
import rospy
import rospkg
import tf
import tty
import termios
import select
from std_msgs.msg import *
from geometry_msgs.msg import *
import bodyhub_action as bodact
import thread
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *
SCRIPTS_PATH=os.path.split(sys.argv[0])[0]
STEP_LEN = [0.06, 0.06, 8.0]
NODE_NAME = 'slam_debug_node'
# POSE_TOPIC = "/sim/torso/pose"
POSE_TOPIC = "/initialpose"
CONTROL_ID=2

class SlamMapNode(bodact.Action):
    def __init__(self):
        super(SlamMapNode, self).__init__(NODE_NAME, CONTROL_ID)
        self.__gait_cmd_pub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=2)
        rospy.Subscriber(POSE_TOPIC, PoseWithCovarianceStamped, self.pose_callback)
        self.debug = False
        self.current_pose = {0, 0, 0}
        self.pose_update = False
        self.err_threshold = [0.15, 0.15, 10]

        self.key_val = ' '
        self.step_len = STEP_LEN
        self.timeout = 15
        self.onhead_rot=False
        self.auto_scan=False
    def printTeleInfo(self):
        print("slam_map_controller")
        print ('%-15s%-15s%-15s' % ('r--headscan', 'w--forward',"e--always_scan"))
        print ('%-15s%-15s%-15s%-15s' % ('a--left', 's--backward', 'd--right','h--headrot'))
        print ('%-15s%-15s%-15s' % ('z--trun left', 'x--in situ', 'c--trun right'))
        print ('%-15s%s\n' % (' ', 'q--quit'))

    def walking_wait(self):
        rospy.wait_for_message('/requestGaitCommand', Bool, self.timeout)

    def walking_send(self, delta):
        self.__gait_cmd_pub.publish(data=delta)

    def walking(self, delta_x, delta_y, theta):
        rospy.wait_for_message('/requestGaitCommand', Bool, self.timeout)
        self.__gait_cmd_pub.publish(data=[delta_x, delta_y, theta])

    def walking_n_steps(self, x, y, a, n):
        for i in range(0, n):
            self.walking(x, y, a)
            print ('%s %-10s%-10s%-10s%-10s' % ('step', i+1, x, y, a))
        self.printTeleInfo()

    def getch(self, str=''):
        print (str),
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print (ch)
        return ch

    def getKey(self, key_timeout):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ' '
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def keyboard_thread(self, args):
        while not rospy.is_shutdown():
            time.sleep(0.02)
            self.key_val = self.getKey(0.5)
            if self.key_val == 'q':
                rospy.signal_shutdown('exit')
                return
            elif self.key_val=='e':
                self.auto_scan=False

    def control_thread(self, args):
        w_cmd = [0.0, 0.0, 0.0]
        while not rospy.is_shutdown():
            self.walking_wait()
            if self.key_val == 'w':
                w_cmd = [self.step_len[0], 0.0, 0.0]
            elif self.key_val == 's':
                w_cmd = [-self.step_len[0]*0.8, 0.0, 0.0]
            elif self.key_val == 'a':
                w_cmd = [0, self.step_len[1], 0.0]
            elif self.key_val == 'd':
                w_cmd = [0, -self.step_len[1], 0.0]
            elif self.key_val == 'z':
                w_cmd = [0.0, 0.0, self.step_len[2]]
            elif self.key_val == 'c':
                w_cmd = [0.0, 0.0, -self.step_len[2]]
            elif self.key_val == 'x':
                w_cmd = [0.0, 0.0, 0.0]
            elif self.key_val == 'h':
                self.onhead_rot= ~ self.onhead_rot
                if  self.onhead_rot:
                    self.bodyhub_ready()
                    self.set_head_rot([0,20])
                else:
                    self.set_head_rot([0,0])
                    self.bodyhub_walk()
                continue
            elif self.key_val == 'r':
                self.bodyhub_ready()
                self.headscan()
                self.bodyhub_walk()
                continue
            elif self.key_val == 'e':                
                self.bodyhub_ready()
                self.key_val=" "
                self.auto_scan=True
                while self.auto_scan:
                    self.headscan(True)
                    time.sleep(2)
                self.bodyhub_walk()
                continue
            else:
                time.sleep(0.02)
                continue
            self.walking_send(w_cmd)
            time.sleep(0.2)

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
        
        self.current_pose = [p.x, p.y, yaw * 180.0 / math.pi]
        self.pose_update = True
        if self.debug:
            print ('pose:', self.current_pose)

    def rot_mat(self, theta):
        theta = theta * math.pi / 180.0
        return np.array([[math.cos(theta), -math.sin(theta)], 
                        [math.sin(theta), math.cos(theta)]], dtype=np.float32)

    def path_tracking(self, path_point, mode=0):
        step_len = [0, 0, 0]
        rot_adjust = False
        path_marker, marker_num = 0, len(path_point)
        while not rospy.is_shutdown():
            rospy.wait_for_message('/requestGaitCommand', Bool, 10)

            if self.pose_update == False:
                print ('location not updated!')
                time.sleep(0.5)
                continue

            if (mode == 1) and (path_marker == marker_num-1):
                time.sleep(0.5)

            for i in range(3):
                step_len[i] = path_point[path_marker][i] - self.current_pose[i]
            v = np.dot(np.linalg.inv(self.rot_mat(self.current_pose[2])), np.array([step_len[0], step_len[1]])).tolist()
            if (mode == 1) and (path_marker == marker_num-1):
                w = step_len[2]
            else:
                w = (math.atan2(step_len[1], step_len[0]) * 180.0 / math.pi) - self.current_pose[2]
            w = (w-360.0) if w >= 180.0 else w
            w = (w+360.0) if w <= -180.0 else w
            step_len = [v[0], v[1], w]
            self.pose_update = False

            pos_err_scale, rot_err_scale = 1.0, 10.0
            if (mode == 1) and (path_marker == marker_num-1):
                pos_err_scale, rot_err_scale = 0.4, 0.2
            if (abs(step_len[0]) < (self.err_threshold[0]*pos_err_scale)) and (abs(step_len[1]) < (self.err_threshold[1]*pos_err_scale)) and (abs(step_len[2]) < (self.err_threshold[2]*rot_err_scale)):
                path_marker = path_marker + 1
                print ('path marker', path_marker, '/', marker_num)
                time.sleep(2)
                if path_marker >= marker_num:
                    self.bodyhub.wait_walking_done()
                    break

            if abs(step_len[2]) > 30:
                rot_adjust = True
            else:
                rot_adjust = False

            for i in range(3):
                step_len[i] = STEP_LEN_MAX[i] if step_len[i] > STEP_LEN_MAX[i] else step_len[i]
                step_len[i] = -STEP_LEN_MAX[i] if step_len[i] < -STEP_LEN_MAX[i] else step_len[i]
            if rot_adjust:
                self.__gait_cmd_pub.publish(data=[0.01, 0, step_len[2]])
            else:
                self.__gait_cmd_pub.publish(data=step_len)
        self.bodyhub.wait_walking_done()
    
    def set_head_rot(self, head_rot):
        keyframes = [
            ([0, -1, 16, -34, -17, -1, 0, 1, -16, 34, 17, 1, 0, -70, -15, 0, 70, 15, 0, 0, head_rot[0], head_rot[1]], 500, 0)
        ]
        self.movement.linearMove(keyframes)
        self.head_pitch = head_rot[1]

    def headscan(self,automode=False):
        rotids=[[i,0] for i in range(0,50,6)]+[[i,10] for i in range(50,-45,-6)]+[[i,0] for i in range(-45,0,6)]
        for i in rotids:
            self.set_head_rot(i)
            if automode and not self.auto_scan:
                self.set_head_rot([0,0])
                break   
    def start(self):
        self.set_arm_mode(0)
        # self.bodyhub_ready()
        # self.headscan()
        # self.set_head_rot([20, 25])
        self.bodyhub_walk()
        self.printTeleInfo()
        try:
            thread.start_new_thread(self.keyboard_thread, (None,))
            thread.start_new_thread(self.control_thread, (None,))
            while not rospy.is_shutdown():
                time.sleep(0.2)
        except:
            print ("Error: unable to start thread")
        rospy.signal_shutdown('exit')

if __name__ == '__main__':
    node=SlamMapNode()
    if len(sys.argv)>1:
        node.debug=True
        print("debug print mode")
        rospy.spin()
    else:
        node.start()
