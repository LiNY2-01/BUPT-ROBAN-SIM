#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time

import rospy
import rospkg
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import motion.motionControl as mCtrl
import motion.TrajectoryPlan as tPlan
from ik_lib.ikmodulesim import IkModuleSim
from ik_lib.ikmodulesim.CtrlType import CtrlType as C
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from mediumsize_msgs.srv import *

NodeControlId = 2
tpObject = tPlan.TrajectoryPlanning(22,10.0)

idList = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22]
angleList = [[0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0, 0,0,0, 0,0, 0,0],1000,0]

class image_converter:
    def __init__(self):
        #创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/sim/camera/D435/colorImage", Image, self.callback)  # 前视摄像头
        # self.image_sub = rospy.Subscriber("/sim/camera/UVC/colorImage", Image, self.callback)  # 下巴摄像头
    
    def callback(self, data):
        #使用cv_bridge将ros的图像数据转换成openCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e
       

        #显示opencv格式的图像
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)








def walk(step):
    if mCtrl.SetBodyhubTo_walking(NodeControlId) == False:
        rospy.logerr('bodyhub to setStatus fail!')
        rospy.signal_shutdown('error')
        exit(1)

    for i in range(0, step):
        mCtrl.SendGaitCommand(0.1, 0.0, 0.0)

    mCtrl.WaitForWalkingDone()
    mCtrl.ResetBodyhub()

def rosShutdownHook():
    mCtrl.ResetBodyhub()


#返回roban位置信息
def torso_callback(data):               
    global roban_torsoPR   #record coordinates of ROBAN
    roban_torsoPR = data.data 
    # print(roban_torsoPR)


def listener():

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
    
    rospy.Subscriber("/sim/torso/PR", Float64MultiArray, torso_callback)
    

        # spin() simply keeps python from exiting until this node is stopped

def set_arm_mode(mode):
    '''
    set bodyhub arm mode
    mode: 0 action mode, 1 gait pattern
    '''
    try:
        rospy.wait_for_service('MediumSize/BodyHub/armMode',  10)
    except:
        print 'error: wait_for_service armMode!'
        return None
    client = rospy.ServiceProxy('MediumSize/BodyHub/armMode', SetAction)
    response = client(mode, 'set arm mode')
    return response.result



if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)     #初始化ros节点
    print('node runing...')
    set_arm_mode(0)
    roban_torsoPR = [1,2,3]          #初始化roban_torsoPR，这个列表记录即机器人的位置,
    listener()                  #开始监听，启动rospy.subscriber(),
    print("显示Roban坐标：")
    mCtrl.ResetBodyhub()                
    time.sleep(1)


    pub = rospy.Publisher('/playerStart', Float64, queue_size=10)

    try:
        image_converter()           # 获取摄像头图像
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test."
        cv2.destroyAllWindows()

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        #获取机器人的位置坐标相关信息
        hello_str = 0.0
        pub.publish(hello_str)
        print('\r Ctrl+c to break;  当前坐标为：X:%f 当前坐标为：Y:%f 当前坐标为：Z:%f ' % (roban_torsoPR[0],roban_torsoPR[1],roban_torsoPR[2]))
        
        rate.sleep()


