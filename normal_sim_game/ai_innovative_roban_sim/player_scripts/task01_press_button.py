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
import motion.bodyhub_client as bodycli
from ik_lib.ikmodulesim import IkModuleSim
from ik_lib.ikmodulesim.CtrlType import CtrlType as C
from motion.motionControl import ResetBodyhub, GetBodyhubStatus, SendJointCommand
target_RGB = [138, 221, 192]
target_pos = [511, 326, 0.155] # 期望目标落于画面中的位置[x, y, depth]
error_threshold = [20,20,0.025] # 行走误差阈值

def rgb_to_hsv(rgb):
    """
    convert rgb to hsv value
    """
    rgb_pixel = np.uint8([[rgb]])
    hsv = cv2.cvtColor(rgb_pixel, cv2.COLOR_RGB2HSV)
    return hsv[0][0]


def color_range(hsv, delta=[0.1, 0.2, 0.6]):
    """
    color hsv upper and lower boundary
    """
    delta_h = int(delta[0] * 0.5 * 180.0)  # h
    delta_s = int(delta[1] * 0.5 * 255.0)  # s
    hsv_l = [hsv[0] - delta_h, hsv[1] - delta_s, int((1-delta[2]) * 0.5 * 255)]
    hsv_h = [hsv[0] + delta_h, hsv[1] + delta_s, int((1 - ((1-delta[2]) * 0.5)) * 255)]
    for i, v in enumerate(hsv_l):
        hsv_l[i] = hsv_l[i] if hsv_l[i] > 0 else 0
    for i, v in enumerate(hsv_h):
        hsv_h[i] = hsv_h[i] if hsv_h[i] < 255 else 255
    hsv_h[0] = hsv_h[0] if hsv_h[0] < 180 else hsv_h[0]
    return np.array(hsv_l), np.array(hsv_h)


class Color_detecter(imgPrcs.ColorObject):
    """继承自颜色识别类"""
    def __init__(self, target_color = target_RGB , debug = False,image_topic='/chin_camera/image'):
        lowerRed,upperRed = color_range(rgb_to_hsv(target_color), [0.1, 0.4, 1])
        print("HSV color range",lowerRed,upperRed)
        super(Color_detecter, self).__init__(lowerRed, upperRed)
        self.__debug = debug
        rospy.Subscriber(image_topic, Image, self.__image_callback) # 订阅图像topic,设置图像回调函数

        self.__cv_bridge = CvBridge()
        self.__img_origin = np.zeros((640, 480, 3), np.uint8)

    def __image_callback(self, msg):
        """图像回调函数"""
        try:
            self.__img_origin = self.__cv_bridge.imgmsg_to_cv2(msg, 'bgr8') # 转化为cv2中的bgr格式
        except CvBridgeError as err:
            rospy.logerr(err)

        if self.__debug:# 如果在debug模式则显示debug图片和rgb取色窗口
            hsvImg = cv2.cvtColor(self.__img_origin, cv2.COLOR_BGR2HSV) # 转换颜色空间到HSV
            imgPrcs.putVisualization(self.__img_origin, self.detection(self.__img_origin))# 绘制识别结果框
            cv2.imshow("hsvImg: click to get HSV",hsvImg)# 显示HSV图像,点击该图像中的像素点可以打印出HSV值
            cv2.imshow("ImageResult",self.__img_origin)# 显示识别结果
            cv2.setMouseCallback('ImageResult',self.click_to_print)# 设置窗口点击事件回调
            cv2.setMouseCallback('hsvImg: click to get HSV',self.click_to_print)# 设置窗口点击事件回调
            cv2.waitKey(1)
            
    def click_to_print(self,event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN:
            hsvImg = cv2.cvtColor(self.__img_origin, cv2.COLOR_BGR2HSV) # 转换颜色空间到HSV
            bgr = self.__img_origin[y,x]
            print("\npos {}:\nrgb value:{},hsv value:{}".format([y,x],(bgr[2],bgr[1],bgr[0]),hsvImg[y,x])) # 打印所选点的hsv值
            self.click_callback([y,x])
            
    def click_callback(self,pos):
        pass  
    
    def detect(self):
        """
        获取颜色识别结果
        @return result["find"]:是否寻找到匹配的图像区域,True/False
                result["Cx"]:返回最大匹配区域的中心x坐标, 以左上角为原点
                result["Cy"]:返回最大匹配区域的中心y坐标
                result["boundingR"]:匹配区域的最大内接矩形
                result["contour"]:匹配区域的轮廓
        """
        return self.detection(self.__img_origin)


class PressButton(IkModuleSim,Color_detecter,bodycli.BodyhubClient):
    def __init__(self,debug = False):
        ResetBodyhub()
        while GetBodyhubStatus().data != "preReady":
            time.sleep(0.1)
            ResetBodyhub()
            continue
        bodycli.BodyhubClient.__init__(self,6)
        IkModuleSim.__init__(self)
        Color_detecter.__init__(self,target_RGB,debug=debug,image_topic="/sim/camera/D435/colorImage")
        self.depth_image_sub = rospy.Subscriber("/sim/camera/D435/depthImage", Image, self.depthImageCallback, queue_size=1)
        self.depth_map = np.zeros((480,640,1))
        self.bridge = CvBridge()

        self.__pid_x = pidAlg.PositionPID(p=0.4)
        self.__pid_y = pidAlg.PositionPID(p=0.0005)
        self.__pid_d = pidAlg.PositionPID(p=0.09)
        self.__err_threshold = error_threshold

    def depthImageCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            rospy.loginfo(e)
        outdep = np.array(cv_image, dtype=np.float)
        self.depth_map = outdep
        
    def click_callback(self,pos):
        print("depth value: {}".format(self.depth_map[pos[0],pos[1]]))
        
    def run(self):
        self.vision_move(target_pos)
        self.press_button()
        
    def get_current_pos(self):
        result = self.detect()
        result["depth"] = self.depth_map[result['Cy'],result['Cx']] if result["find"] else 0
        return result
    
    def vision_move(self, targepos=target_pos):
        """走向目标的位置: 通过运动控制将目标落于画面中的targepos位置"""
        self.walk() #  状态跳转到walking, 允许行走
        # self.walking_the_distance(0.1,0.1,0)
        while not rospy.is_shutdown():
            result = self.get_current_pos() # 获取颜色识别结果, 返回目标位置
            if result['find'] != False:
                print("current pos",result["Cx"], result["Cy"], result['depth'])
                yError = targepos[0] - result['Cx'] # 计算水平方向目标在画面中的位置与期望位置的差值
                dError = result['depth'] - targepos[2] # 计算机器人前进方向/图像深度方向的差值
                print(yError,dError)
                if (abs(yError) < self.__err_threshold[0]) and (abs(dError) < self.__err_threshold[2]):
                    break # 误差小于阈值则退出
                xLength = self.__pid_x.run(dError) # 将差值传入pid控制器获得x\y\a方向的控制量
                yLength = self.__pid_y.run(yError)
                aLength = 0
                print("step len",xLength, yLength, aLength)
                self.walking_the_distance(xLength, yLength, aLength) # 执行动作进行校正, 往前:x+, 往左:y+, 逆时针:a+
                self.wait_walking_done()# 等待动作执行完毕
                time.sleep(0.2)
            else:
                rospy.logwarn('no target found!')
                time.sleep(0.5)
                  
    def press_button(self):
        print("press_button")
        self.reset()
        if self.toInitPoses():
            self.body_motion([C.RArm_z, C.RArm_x], [0.165, 0.074], 20) 

            time.sleep(0.1)
            
            self.body_motion([C.RArm_x, C.RArm_y], [0.073, 0.008], 30)
            
            self.body_motion([C.RArm_z, C.RArm_x], [0.05, -0.02], 20)  
            
            self.body_motion([C.RArm_z, C.RArm_x], [-0.05, -0.05], 20)
            print("over")
        self.reset()
            
    def calibrating(self):
        while not rospy.is_shutdown():
            result = self.detect()
            if result['find'] != False:
                center = [result['Cx'],result['Cy']]
                print("[cx,cy,depth] is {}".format([center[0],center[1],self.depth_map[center[1],center[0]]]))
            time.sleep(0.2)
            
def rosShutdownHook():
    rospy.signal_shutdown('node_close')
    
if __name__ == '__main__':
    rospy.init_node('PressButton', anonymous=True)
    rospy.on_shutdown(rosShutdownHook)
    pb = PressButton(debug= True)
    pb.run()
    # ps.calibrating()
    # rospy.spin()
