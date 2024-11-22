#!/usr/bin/env python
# coding=utf-8
import sys
import os
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

from ik_lib.ikmodulesim import IkModuleSim
from ik_lib.ikmodulesim.CtrlType import CtrlType as C
from motion.motionControl import ResetBodyhub, GetBodyhubStatus, SendJointCommand,WaitTrajectoryExecute,SetBodyhubTo_setStatus,SendGaitCommand,SetBodyhubTo_walking
target_RGB = [236, 223, 206]
target_pos = [422, 400, 0.138] # 期望目标落于画面中的位置[x, y, depth]
error_threshold = [20,20,0.02] # 行走误差阈值

CONTROL_ID = 6
SCRIPTS_PATH=os.path.split(sys.argv[0])[0]


class detecter():
    """使用级联分类器的目标检测类"""
    def __init__(self, target_color = target_RGB , debug = False,image_topic='/chin_camera/image'):
        self.__debug = debug
        rospy.Subscriber(image_topic, Image, self.__image_callback) # 订阅图像topic,设置图像回调函数
        self.depth_image_sub = rospy.Subscriber("/sim/camera/D435/depthImage", Image, self.depthImageCallback, queue_size=1)

        self.__cv_bridge = CvBridge()
        self.__img_origin = np.zeros((640, 480, 3), np.uint8)
        self.id = 0
        self.cascadeClassifier = cv2.CascadeClassifier(os.path.join(SCRIPTS_PATH,"cascade.xml"))
        self.result = {"update":False, "pos":[0,0,0,0],"Cx":0,"Cy":0}
        self.depth_map = {"update":False, "depth":None}
        
    def depthImageCallback(self,data):
        try:
            cv_image = self.__cv_bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            rospy.loginfo(e)
        outdep = np.array(cv_image, dtype=np.float)
        self.depth_map = {"update":True, "depth":outdep}
        
    def __image_callback(self, msg):
        """图像回调函数"""
        try:
            self.__img_origin = self.__cv_bridge.imgmsg_to_cv2(msg, 'bgr8') # 转化为cv2中的bgr格式
        except CvBridgeError as err:
            rospy.logerr(err)
        gray = cv2.cvtColor(self.__img_origin, cv2.COLOR_RGB2GRAY)
        res = self.cascadeClassifier.detectMultiScale(gray,scaleFactor=1.01,
                                        minNeighbors=30,
                                        minSize=(18, 18),maxSize=(200,150))
        if len(res):
            x,y,w,h = res[0]
            self.result["Cx"] = x+w//2
            self.result["Cy"] = y+h//2
            self.result["pos"]=res[0]
            self.result["update"]=True
        else:
            self.result["update"] = False
        
        if self.__debug:# 如果在debug模式则显示debug图片和rgb取色窗口
            
            painterimg = self.__img_origin.copy()
            # print(self.result)
            for (x,y,w,h) in res:
                cv2.rectangle(painterimg, (x,y), (x+w,y+h),(255,0,0),2)
            
            cv2.imshow("ImageResult",painterimg)# 显示识别结果
            
            cv2.setMouseCallback('ImageResult',self.click_to_print)# 设置窗口点击事件回调
            
            key = cv2.waitKey(1)
            if key == ord("s"):
                self.id = time.strftime("%Y%m%d%H%M%S", time.localtime(time.time()))
                print("save posimage{}.png".format(self.id))
                cv2.imwrite("/home/fan/Pictures/infonew/image{}.png".format(self.id),self.__img_origin)
            elif key == ord("w"):
                self.id = time.strftime("%Y%m%d%H%M%S", time.localtime(time.time()))
                print("save negimage{}.png".format(self.id))
                cv2.imwrite("/home/fan/Pictures/bad_rgb/image{}.png".format(self.id),self.__img_origin)
           
            
            
    def click_to_print(self,event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN:
            hsvImg = cv2.cvtColor(self.__img_origin, cv2.COLOR_BGR2HSV) # 转换颜色空间到HSV
            bgr = self.__img_origin[y,x]
            print("\npos {}:\nrgb value:{},hsv value:{}".format([y,x],(bgr[2],bgr[1],bgr[0]),hsvImg[y,x])) # 打印所选点的hsv值
            self.click_callback([y,x])
            
    def click_callback(self,pos):
        pass  
    
    def detect(self,wait_time = 10):
        stime = time.time()
        while not rospy.is_shutdown() and time.time() -stime < wait_time:
            if self.depth_map["update"]:
                self.result["depth"] = self.depth_map["depth"][self.result["Cy"],self.result["Cx"]]
                self.result["depthupdate"] = True
                self.depth_map["update"]=False
            else:
                self.result["depthupdate"]= False
            if self.result["update"] and self.result["depthupdate"]:
                return self.result
        # print(self.result)
        return self.result


def right_gripper(status):
    gripper_id_list = [20,22]
    if status == 'open':
        print('手爪打开')
        SendJointCommand(CONTROL_ID, [19,20], [40,0])
    if status == 'close':
        print('手爪关闭')
        SendJointCommand(CONTROL_ID, [19,20], [10,0])


class Hand_tea(IkModuleSim,detecter,BodyhubClient):
    def __init__(self,debug = True):
        IkModuleSim.__init__(self)
        detecter.__init__(self,target_RGB,debug=debug,image_topic="/sim/camera/D435/colorImage")
        BodyhubClient.__init__(self, CONTROL_ID)
        self.__err_threshold = error_threshold
        self.__pid_x = pidAlg.PositionPID(p=0.5)
        self.__pid_y = pidAlg.PositionPID(p=0.00045)
        self.__pid_d = pidAlg.PositionPID(p=0.09)
        self.__gait_cmd_pub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=2)
        self.is_walking = 0
        def walk_state_cb(msg):
            self.is_walking =  msg.data
        rospy.Subscriber("/MediumSize/BodyHub/WalkingStatus", Float64, walk_state_cb, queue_size=2)
        while GetBodyhubStatus().data != "preReady":
            time.sleep(0.1)
            ResetBodyhub()
    
    def hand_reach(self, pos, count=100):
        self.body_motion([C.RArm_x, C.RArm_y, C.RArm_z], pos, count) 
    
    def run(self):
        self.vision_move_to()
        self.hand_tea_action()
        rospy.signal_shutdown("play ended")
        
    def vision_move_to(self):
        self.reset()
        while not rospy.is_shutdown():
            detectresult = self.detect()
            detecter_angle = 0
            if not detectresult["update"] or not detectresult["depthupdate"]:
                print("not detected",detectresult["update"],detectresult["depthupdate"])
                search_result = self.search_for_target()
                if search_result is not None:
                    detectresult,search_pos = search_result
                    detecter_angle = search_pos[0]
                else:
                    continue
            detectresult["Cx"] = detectresult["Cx"] if detecter_angle == 0 else 640+320*np.sin(np.deg2rad(-detecter_angle))
            print("current pos",detectresult["Cx"], detectresult["Cy"], detectresult['depth'])
            yError = target_pos[0] - detectresult['Cx'] # 计算水平方向目标在画面中的位置与期望位置的差值
            dError = detectresult['depth'] - target_pos[2] if detecter_angle == 0 else 0 # 计算机器人前进方向/图像深度方向的差值
            print("diff {} {}".format(yError,dError))
            if (abs(yError) < self.__err_threshold[0]) and (abs(dError) < self.__err_threshold[2]):
                print("到达目的地")
                break # 误差小于阈值则退出
            xLength = self.__pid_x.run(dError) # 将差值传入pid控制器获得x\y\a方向的控制量
            yLength = self.__pid_y.run(yError)
            aLength = 0
            print("step len",xLength, yLength, aLength)
            self.walk()
            self.walking_the_distance(xLength, yLength, aLength) # 执行动作进行校正, 往前:x+, 往左:y+, 逆时针:a+
            print("walking sended")
            self.wait_walking_done()# 等待动作执行完毕
            print("walking done")
                
            time.sleep(0.1)
        self.reset()
        
    def search_for_target(self):
        self.ready()
        detectresult = self.detect()
        search_trajectory = [[0,35]]+[[id21,35] for id21 in range(-10,-61,-10)]+[[id21,35] for id21 in range(-60,61,10)]
        while not detectresult["update"] and not rospy.is_shutdown():
            for search_pos in search_trajectory:
                while not rospy.is_shutdown():
                    print("searching to {}".format(search_pos))
                    self.set_joint_position([21,22], search_pos)
                    if self.is_reach_target([21,22], search_pos):
                        break
                    time.sleep(1)
                time.sleep(1)
                detectresult = self.detect()
                if detectresult["update"]:
                    print("reach target {}".format(search_pos))
                    return detectresult,search_pos
                
    def is_reach_target(self,ids,target_angle,err = 5):
        current_angle = self.get_joint_position([22])
        if current_angle is not None:
            match_angle = [current_angle[id-1] for id in ids]
            error_sum = sum([ abs(i) for i in list(np.array(match_angle)-np.array(target_angle))])
            if error_sum< err:
                return True
        return False

    def walking(self, delta_x, delta_y, theta, timeout = None):
        self.is_walking = 1
        self.__gait_cmd_pub.publish(data=[delta_x, delta_y, theta])
        return 0
    
    def wait_walking_done(self):
        while not rospy.is_shutdown():
            if self.is_walking == 0:
                break
            
    def hand_tea_action(self):
        if self.toInitPoses():

            self.hand_reach([0.06, -0.05, 0.154], count=50)
            self.hand_reach([0.08, 0.125, 0.0], count=50)
            print("手爪打开")
            right_gripper("open")
            time.sleep(1)
            self.hand_reach([-0.04, -0.125, 0.0], count=50)
            self.hand_reach([-0.04, 0.0, 0.0], count=10)
            self.hand_reach([-0.08, 0.05, -0.154], count=50)
            right_gripper("close")

            time.sleep(2)

            print("reset")
            self.reset()
            

if __name__ == '__main__':
   
    action = Hand_tea()
    
    def rosShutdownHook():
        action.reset()
        rospy.signal_shutdown('node_close')

    rospy.init_node('Hand_tea', anonymous=True)
    rospy.on_shutdown(rosShutdownHook)    
    action.run()
