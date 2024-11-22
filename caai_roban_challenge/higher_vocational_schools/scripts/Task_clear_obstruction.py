#!/usr/bin/env python
# -*- coding: utf-8 -*-
from operator import ne
import sys
import os
import time
import numpy as np
import rospy
import rospkg
import yaml
from std_msgs.msg import *
from geometry_msgs.msg import *

import threading
import cv2
from cv_bridge import *
from sensor_msgs.msg import *
from std_msgs.msg import *
sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *
from public import PublicNode
from frames import RobanFrames

SCRIPTS_PATH=os.path.split(sys.argv[0])[0]
IMAGE_TOPIC = '/camera/color/image_raw'
NODE_NAME = 'clear_obstruction_node'
CONTROL_ID = 2
TARGET_BOX_COLOR="blue"

with open(os.path.join(SCRIPTS_PATH,"slam_clear_obstruction.yaml"),"r")as f:
    SLAM_POINT=yaml.load(f)
    print(SLAM_POINT)
def show_img(name, img):
    cv2.imshow(name, img)
    k = cv2.waitKey(1)

class HSVColor:
    yellow_low = np.array([1, 160, 140])
    yellow_high = np.array([80, 215, 260])

    blue_low = np.array([70,120, 100])
    blue_high = np.array([110, 250, 190])

    red_low = np.array([0, 43, 46])
    red_high = np.array([10, 255, 255])

class Box_color_detecter():
    def __init__(self,debug=False):
        self.sub=rospy.Subscriber(IMAGE_TOPIC, Image, self.image_callback)
        self.cv_bridge = CvBridge()
        self.__debug=debug
        self.detecter=Detect_thread(self,debug=debug)
        self.detecter.start()

    def image_callback(self,msg):
        if self.detecter.detecting or  not self.detecter.runing:#in busy or exit
            time.sleep(0.02)
            return
        try:
            img =  self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as err:
            rospy.logerr(err)
        self.detecter.update_frame(img)
    def get_box_pos(self):
        if self.detecter.pos_updated:
            self.detecter.pos_updated=False
            return self.detecter.pos
        else:
            rospy.logwarn("pos not update!!")
            return None
    def stop_detecter(self):
        self.detecter.runing=False
        self.sub.unregister()
        rospy.loginfo("color detecter exit!")
    
class Detect_thread(threading.Thread):
    def __init__(self,parent,debug=False):
        super(Detect_thread,self).__init__()
        self.parent=parent
        self.detecting=False
        self.frame=None
        self.__debug=debug
        self.__fps_time=0
        self.pos_updated=False
        self.pos={"yellow":[0,0,0],"blue":[0,0,0]}
        self.runing=False

    def run(self):
        self.runing=True
        while ((not rospy.is_shutdown())and self.runing):
            if self.detecting:
                self.detect()
                self.detecting=False
                continue
            time.sleep(0.02)
        rospy.loginfo("color detecter thread exit!")
        self.runing=False
    def update_frame(self,img):
        self.frame=img
        self.detecting=True
        
    def sign_hsv_value(self):
        HSV=self.pre_process()
        def getpos(event,x,y,flags,param):
            if event==cv2.EVENT_LBUTTONDOWN:
                print(HSV[y,x])

        cv2.imshow("image_HSV",HSV)
        cv2.setMouseCallback('image_HSV',getpos)
        cv2.waitKey(0)

    def detect(self):
        detect_time = time.time()
        hsv_img=self.pre_process()
        try:
            y=self.color_trace(hsv_img,"yellow")
            b=self.color_trace(hsv_img,"blue")
            if self.__debug:
                detect_time = time.time() - detect_time
                fps = 1.0/(time.time() - self.__fps_time)
                self.__fps_time = time.time()
                print ('fps: {:.2f}, detect time(ms): {:.2f}'.format(fps, detect_time*1000.0 ),y,b)
                show_img('image', self.frame)

                def getpos(event,x,y,flags,param):
                    if event==cv2.EVENT_LBUTTONDOWN:
                        print(hsv_img[y,x])

                cv2.setMouseCallback('image',getpos)
                cv2.waitKey(1)

            if y!=None and b != None and abs(y[1]-b[1])<50:
                self.pos["yellow"],self.pos["blue"]=y,b
            else:
                time.sleep(0.2)# not found color box
                return
            self.pos_updated=True
            
            
        except:
            print(sys.exc_info(),104)
    def pre_process(self):
        return cv2.cvtColor(self.frame,cv2.COLOR_BGR2HSV)

    def color_trace(self,hsv, color):
        mask = None
        if color == "red":
            mask = cv2.inRange(hsv, HSVColor.red_low, HSVColor.red_high)
        elif color == "blue":
            mask = cv2.inRange(hsv, HSVColor.blue_low, HSVColor.blue_high)
        elif color == "yellow":
            mask = cv2.inRange(hsv, HSVColor.yellow_low, HSVColor.yellow_high)
        else:
            exit("color is wrong\n\n\n")
        erosion = cv2.erode(mask, None, iterations=2)#np.ones((1, 1), np.uint8)
        dilation = cv2.dilate(erosion, np.ones((1, 1), np.uint8), iterations=2)
        target = cv2.bitwise_and(self.frame, self.frame, mask=dilation)
        # show_img("dilation",dilation)
        # show_img("target",target)
        ret, binary = cv2.threshold(dilation, 127, 255, cv2.THRESH_BINARY)
        # find contours in binary and sort them
        __c, contours,__a = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        m = 0
        if len(contours)==0:return None
        def cnt_area(cnt):
            area = cv2.contourArea(cnt)
            return area
        contours.sort(key = cnt_area, reverse=True)#find the max_c
        x, y, w, h = cv2.boundingRect(contours[0])
        if self.__debug:
            cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 255,), 3)
            # mark the rects
            cv2.putText(self.frame, color, (x - 10, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) 
        
        return (x+w)//2,(y+h)//2

class Task_clear_obstruction(PublicNode):
    def __init__(self,nodename=NODE_NAME,control_id=CONTROL_ID):
        super(Task_clear_obstruction, self).__init__(nodename, control_id)

    def debug(self):
        self.colordetecter=Box_color_detecter(debug=True)

    def detect_color_box(self):
        pos=self.colordetecter.get_box_pos()
        print(pos)
        if pos and TARGET_BOX_COLOR in pos:
            for color in pos.keys():
                if color !=TARGET_BOX_COLOR :
                    return "left" if pos[color][0]>pos[TARGET_BOX_COLOR][0] else "right"
                
        rospy.logerr("UNKNOW BOX COLOR!")
        return

    def detectcolor_tracking(self):
        res=self.detect_color_box()
        print("\n\n to the {}".format(res))
        if res in ["left","right"]:
            self.colordetecter.stop_detecter()
            self.path_tracking(SLAM_POINT["{}_box_pos".format(res)],pos_wait_mode=1)
        else:
            time.sleep(1)
            print("not detect!retry...")
            self.detectcolor_tracking()
        return res

    def push_box_action(self,direction):
        self.set_arm_mode(0)
        self.bodyhub_ready()
        key_frame=RobanFrames.hang_up_left_arm_frames if direction =="right" else  RobanFrames.hang_up_right_arm_frames
        self.frame_action(key_frame)
        self.bodyhub_walk()
        self.bodyhub.walking_n_steps([0.0, 0.0, -10 if direction =="right" else 10], 20)
        self.bodyhub.walking_n_steps([-0.09, 0 , 0.0], 1)
        self.bodyhub.wait_walking_done()

        # #disarm
        # self.bodyhub_ready()
        # key_frame=RobanFrames.left_disarm_frames if direction =="right" else  RobanFrames.right_disarm_frames
        # self.frame_action(key_frame)

        self.set_arm_mode(1)

    def start_clear_obstruction(self):
        self.bodyhub_walk()
        self.set_arm_mode(1)
        self.path_tracking(SLAM_POINT["origin_pos"])
        self.colordetecter=Box_color_detecter()
        self.path_tracking(SLAM_POINT["detect_pos"],pos_wait_mode=1)
        self.bodyhub_ready()
        self.set_arm_mode(0)
        self.bodyhub_walk()
        direction=self.detectcolor_tracking()
        self.push_box_action(direction)
        self.set_arm_mode(1)
        self.colordetecter.stop_detecter()
     

if __name__ == '__main__':
    if len(sys.argv)>1:
        Task_clear_obstruction().debug()
    else:
        Task_clear_obstruction().start_clear_obstruction()