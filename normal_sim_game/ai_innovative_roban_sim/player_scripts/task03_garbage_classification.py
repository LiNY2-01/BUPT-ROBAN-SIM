#!/usr/bin/env python
# coding=utf-8
import Queue
import sys
import os
import threading
import numpy as np
import math
import time
import rospy
import rospkg
import numpy as np
import cv2 
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
from cv_bridge import *
from std_msgs.msg import *
from sensor_msgs.msg import *
import vision.imageProcessing as imgPrcs
from lejufunc import client_action
import algorithm.pidAlgorithm as pidAlg
from motion.bodyhub_client import BodyhubClient
from scipy import optimize as op

from ik_lib.ikmodulesim import IkModuleSim
from ik_lib.ikmodulesim.CtrlType import CtrlType as C
from motion.motionControl import ResetBodyhub, GetBodyhubStatus, SendJointCommand,WaitTrajectoryExecute,SetBodyhubTo_setStatus,SendGaitCommand,SetBodyhubTo_walking
CONTROL_ID = 6
target_pos = [183, 167,0] # 期望目标落于画面中的位置[x, y, angle]
error_threshold = [10,10,4] # 误差阈值


color_mask = [
    lambda img: cv2.inRange(cv2.cvtColor(img,cv2.COLOR_BGR2HSV), np.array([96, 220, 140]), np.array([104,255,255])),
    lambda img: cv2.bitwise_or(cv2.inRange(cv2.cvtColor(img,cv2.COLOR_BGR2HSV), np.array([0, 160, 80]), np.array([10,255,255])),
        cv2.inRange(cv2.cvtColor(img,cv2.COLOR_BGR2HSV), np.array([170, 160, 80]), np.array([180,255,255]))),
    lambda img: cv2.inRange(cv2.cvtColor(img,cv2.COLOR_BGR2HSV), np.array([20,200,80]), np.array([40,255,255])),
    lambda img: cv2.inRange(cv2.cvtColor(img,cv2.COLOR_BGR2HSV), np.array([0, 0, 50]), np.array([30,30,125]))
]
color_mask_name = ['blue','red','yellow','gray']
class DetecterDSP(threading.Thread):
    def __init__(self):
        super(DetecterDSP,self).__init__()
        self.QUEUE_IMG_FRAME = Queue.Queue(maxsize=2)
        self.display = True
        
    def set_display(self,d):
        self.display = bool(d)

    def push(self,image,flip=0, tips="",wn = "pose_result"):
        dspinfo={}
        dspinfo["tips"],dspinfo["wn"],dspinfo["flip"] = tips,wn,flip
        image = {"image":image,"info":dspinfo}
        if self.QUEUE_IMG_FRAME.full():
            d = self.QUEUE_IMG_FRAME.get()
        self.QUEUE_IMG_FRAME.put(image, block=True,timeout=1)

    def run(self):
        while not rospy.core.is_shutdown_requested():
            if not self.display or self.QUEUE_IMG_FRAME.empty():
                if not self.display:
                    cv2.destroyAllWindows()
                time.sleep(0.2)
                continue
            queus_data = self.QUEUE_IMG_FRAME.get()
            frame = queus_data["image"]
            dspinfo = queus_data["info"]
            self.display_frame(frame,dspinfo["flip"],dspinfo["tips"],dspinfo["wn"])
        cv2.destroyAllWindows()

    def display_frame(self,image, flip=False, tips="",wn = "result"):
        if flip:
            image = cv2.flip(image, 1) 
        image=cv2.putText(image,tips,(20,25),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,100,0),2)
        cv2.imshow(wn, image)
        key = cv2.waitKey(1)
        if key in [ord('q'), 27]:
            print("break")
            self.running = False
            exit(0)

DSPthread = DetecterDSP()
DSPthread.start()


class Detecter():
    def __init__(self, debug = False,image_topic='/chin_camera/image'):
        self.__debug = debug
        self.detectorname = image_topic
        rospy.Subscriber(image_topic, Image, self.__image_callback) # 订阅图像topic,设置图像回调函数
        self.__cv_bridge = CvBridge()
        self.__img_origin = np.zeros((640, 480, 3), np.uint8)
    
    def __image_callback(self, msg):
        """图像回调函数"""
        try:
            self.__img_origin = self.__cv_bridge.imgmsg_to_cv2(msg, 'bgr8') # 转化为cv2中的bgr格式
        except CvBridgeError as err:
            rospy.logerr(err)
        if self.__debug:
            cv_image = self.__img_origin.copy()
            copy = cv_image.copy()
            # copy = cv2.cvtColor(copy, cv2.COLOR_BGR2HSV)
            for i in range(4):
                mask_select= color_mask[i](cv_image)
                opened = cv2.morphologyEx(mask_select, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
                closing = cv2.morphologyEx(opened, cv2.MORPH_CLOSE,
                                        np.ones((3, 3), np.uint8))
                ret, thresh = cv2.threshold(closing, 127, 255, cv2.THRESH_BINARY)
                im2, contours, hierarchy = cv2.findContours(
                    thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                if(len(contours) > 0):
                    contours.sort(key=lambda cnt: cv2.contourArea(cnt), reverse=True)
                    cnt = contours[0]
                    x0, y0, w0, h0 = cv2.boundingRect(cnt)
                    if(w0 < 20 or cv2.contourArea(cnt) < 300):
                        continue
                    # cv2.drawContours(copy, contours, 0, (255, 255, 0), 2)
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(copy, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(copy,color_mask_name[i],(x, y-10 if y > 10 else y),cv2.FONT_ITALIC, 0.7, [50, 50, 0],2)
            DSPthread.push(copy,wn=self.detectorname)
    
    def detect(self):
        cv_image = self.__img_origin.copy()
        detectresult = {}
        # copy = cv2.cvtColor(copy, cv2.COLOR_BGR2HSV)
        for i in range(4):
            mask_select= color_mask[i](cv_image)
            opened = cv2.morphologyEx(mask_select, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            closing = cv2.morphologyEx(opened, cv2.MORPH_CLOSE,
                                    np.ones((3, 3), np.uint8))
            ret, thresh = cv2.threshold(closing, 127, 255, cv2.THRESH_BINARY)
            im2, contours, hierarchy = cv2.findContours(
                thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if(len(contours) > 0):
                contours.sort(key=lambda cnt: cv2.contourArea(cnt), reverse=True)
                cnt = contours[0]
                x0, y0, w0, h0 = cv2.boundingRect(cnt)
                area = cv2.contourArea(cnt)
                if(w0 < 20 or area < 300):
                    continue
                detectresult[color_mask_name[i]]={"rect": [x0, y0, w0, h0],"Cx":x0+w0//2,"Cy":y0+h0//2,"area":area}
            
        return detectresult

def right_gripper(status):
    if status == 'open':
        print('手爪打开')
        SendJointCommand(CONTROL_ID, [19,20], [40,0])
    if status == 'close':
        print('手爪关闭')
        SendJointCommand(CONTROL_ID, [19,20], [10,0])
        
           
class Classifier(IkModuleSim,BodyhubClient):
    def __init__(self,debug = True):
        IkModuleSim.__init__(self)
        BodyhubClient.__init__(self, CONTROL_ID)
        self.chindetecter = Detecter(debug=debug, image_topic="/sim/camera/UVC/colorImage")
        self.headdetecter = Detecter(debug=debug, image_topic="/sim/camera/D435/colorImage")
        self.__err_threshold = error_threshold
        self.__pid_x = pidAlg.PositionPID(p=0.001)
        self.__pid_y = pidAlg.PositionPID(p=0.001)
        self.__pid_a = pidAlg.PositionPID(p=1)
        self.__gait_cmd_pub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=2)
        self.is_walking = 0
        def walk_state_cb(msg):
            self.is_walking =  msg.data
        rospy.Subscriber("/MediumSize/BodyHub/WalkingStatus", Float64, walk_state_cb, queue_size=2)
        while GetBodyhubStatus().data != "preReady":
            time.sleep(0.1)
            ResetBodyhub()
        self.targetcolor="red"
            
    def hand_reach(self, pos, count=100):
        self.body_motion([C.RArm_x, C.RArm_y, C.RArm_z], pos, count)
        
    # def walking(self, delta_x, delta_y, theta, timeout = None):
    #     self.is_walking = 1
    #     self.__gait_cmd_pub.publish(data=[delta_x, delta_y, theta])
    #     return 0
    
    # def wait_walking_done(self):
    #     while not rospy.is_shutdown():
    #         if self.is_walking == 0:
    #             break
                    
    def run(self):
        self.targetcolor = self.get_target_color()# 获取目标颜色
        print("目标颜色为{}".format(self.targetcolor))
        self.pick_up_action()# 执行拿起动作
        self.move_to_target()# 移动到目标
        self.put_down_action()# 执行放下动作
        # rospy.spin()
    
    def is_reach_target(self,ids,target_angle,err = 5):
        current_angle = self.get_joint_position([22])
        if current_angle is not None:
            match_angle = [current_angle[id-1] for id in ids]
            error_sum = sum([ abs(i) for i in list(np.array(match_angle)-np.array(target_angle))])
            if error_sum< err:
                return True
        return False
    
    def wait_to_reach_target(self,ids,target_angle,err = 5):
        while not rospy.is_shutdown():
            self.set_joint_position(ids,target_angle)
            if self.is_reach_target(ids,target_angle,err):
                break
            time.sleep(0.05)
            
    def right_gripper(self,status):
        if status == 'open':
            print('手爪打开')
            self.wait_to_reach_target([19,20], [40,0])
            time.sleep(0.5)
        if status == 'close':
            print('手爪关闭')
            # self.wait_to_reach_target([19,20], [30,0])
            # self.wait_to_reach_target([19,20], [20,0])
            self.wait_to_reach_target([19,20], [15,0])
            self.wait_to_reach_target([19,20], [7,0])
            
    def walking_n_steps(self,cmd, num):
        for i in range(num):
            self.walking(cmd[0],cmd[1],cmd[2],timeout = 120)
        self.wait_walking_done()
        
    def trun_around(self,angle = 90, direction = "left"):
        step_num = 1
        while not rospy.is_shutdown():
            step_len = angle/step_num
            if step_len <= 9:
                break
            step_num +=1
        print("往{}转{}°,共{}步".format(direction,angle,step_num))
        self.walking_n_steps([0,0,step_len if direction == "left" else -step_len],step_num)
        
    def move_to_target(self):
        self.walk()
        self.trun_around(angle = 90, direction="right")#右转
        print("往前走")
        self.walking_n_steps([0.1,0,0],10)# 往前走
        self.trun_around(angle = 90, direction="left")#左转
        while not rospy.is_shutdown():
            headdetectresult = self.headdetecter.detect()#检测头部摄像头是否识别到颜色区域
            chindetectresult = self.chindetecter.detect()#下巴摄像头有目标
            if len(headdetectresult) < 4 or (len(chindetectresult)==4 and list(chindetectresult.values())[0]["area"]>3000):
                
                break#当头部摄像头识别不到垃圾桶时,转为下巴摄像头识别
            else:
                self.walking(0.1,0,0)#一直往前走
                time.sleep(1)
                continue
        while not rospy.is_shutdown():
            detectresult = self.chindetecter.detect()#使用下巴摄像头识别颜色
            print(detectresult)
            if self.targetcolor in detectresult:
                detect_color = detectresult[self.targetcolor]
                detect_angle = self.get_angle(detectresult)
                print("current pos",detect_color["Cx"], detect_color["Cy"])
                yError = target_pos[0] - detect_color['Cx'] # 计算水平方向目标在画面中的位置与期望位置的差值
                dError = target_pos[1] - detect_color['Cy']# 计算机器人前进方向的差值
                aError = target_pos[2] - detect_angle# 计算机器人前进方向的差值
                print("diff {} {} {}".format(dError,yError,aError))
                if (abs(yError) < self.__err_threshold[0]) and (abs(dError) < self.__err_threshold[1]):
                    print("到达目的地")
                    break # 误差小于阈值则退出
                xLength = self.__pid_x.run(dError) # 将差值传入pid控制器获得x\y\a方向的控制量
                yLength = self.__pid_y.run(yError)
                aLength = self.__pid_a.run(aError)
                print("step len",xLength, yLength, aLength)
                self.walk()
                self.walking_the_distance(xLength, yLength, aLength) # 执行动作进行校正, 往前:x+, 往左:y+, 逆时针:a+
                print("walking sended")
                self.wait_walking_done()# 等待动作执行完毕
                print("walking done")
        self.reset()
        
    def get_angle(self,detect_result):
        keys = list(detect_result)
        average_area = sum([detect_result[color]["area"] for color in keys])/len(keys)
        for color in keys:
            if abs(detect_result[color]["area"] - average_area) > average_area/2:
                detect_result.pop(color)
        keys = list(detect_result)
        if len(keys) <= 1: return 0

        X,Y = [detect_result[color]["Cx"] for color in keys],[detect_result[color]["Cy"] for color in keys]
        # 需要拟合的函数
        def func(x, A, B):
            return A * x + B
        # 得到返回的A，B值
        A, B = op.curve_fit(func, X,Y)[0]
        return np.degrees(np.arctan(A))
        
    def pick_up_action(self):
        if self.toInitPoses():
            print("下蹲")
            self.body_motion([C.Torso,C.RArm_x, C.RArm_y, C.RArm_z], [[0, 0.1, 0, 0.01, 0.0, -0.12],0.09, -0.025, 0.10],count = 110)
            self.right_gripper("open")
            self.hand_reach([0.02, 0.0, 0.006], count=15)
            self.hand_reach([0.016, 0.01, 0.00], count=25)
            self.right_gripper("close")
            
            self.body_motion([C.Torso,C.RArm_x, C.RArm_y, C.RArm_z], [[0, -0.1, 0, -0.01, 0.0, 0.12],-0.04, 0.015, 0],count = 110)
            self.hand_reach([-0.088, 0.0, -0.106], count=15)
            self.reset()
            print("end")
            
    def put_down_action(self):
        if self.toInitPoses():
            print("举手")
            self.body_motion([C.Torso,C.RArm_x, C.RArm_y, C.RArm_z], [[0, 0.3, 0, 0.01, 0.0, -0.05],0.11, 0.01, 0.10],count = 80)
            self.body_motion([C.RArm_x, C.RArm_y, C.RArm_z], [0.04, 0.05, 0.03],count = 50)
            self.right_gripper("open")
            self.body_motion([C.RArm_x, C.RArm_y, C.RArm_z], [0.005, 0.0, 0.03],count = 50)
            self.body_motion([C.Torso,C.RArm_x, C.RArm_y, C.RArm_z], [[0, -0.3, 0, -0.01, 0.0, 0.05],-0.11, -0.06, -0.10],count = 80)
            self.body_motion([C.RArm_x, C.RArm_y, C.RArm_z], [-0.04, 0.0, -0.03],count = 50)
            self.reset()
            print("end")
    def get_target_color(self): 
        print("检测目标颜色中...")
        while not rospy.is_shutdown():
            color = self.chindetecter.detect()
            if len(color):
                return color.keys()[0]
            else:
                print("未找到目标")
                time.sleep(1)

if __name__ == '__main__':
    rospy.init_node('task07Classifier', anonymous=True)
    cf = Classifier()
    def rosShutdownHook():
        cf.reset()
        rospy.signal_shutdown('node_close')
        
    rospy.on_shutdown(rosShutdownHook)    
    cf.run()

