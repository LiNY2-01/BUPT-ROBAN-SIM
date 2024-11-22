#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# from sensor_msgs.point_cloud2 import PointCloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import PointField

class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.color_image_sub = rospy.Subscriber("/sim/camera/D435/colorImage", Image, self.colorImageCallback, queue_size=1)
        self.depth_image_sub = rospy.Subscriber("/sim/camera/D435/depthImage", Image, self.depthImageCallback, queue_size=1)
        self.color_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('/camera/aligned_depth_to_color/image_raw', Image, queue_size=10)
        self.pointcloud_pub = rospy.Publisher("/cloud_in",PointCloud2,queue_size=10)
        self.pointcloud_msg = PointCloud2()
        self.pointcloud_msg.fields = [
		PointField('x', 0, PointField.FLOAT32, 1),
		PointField('y', 4, PointField.FLOAT32, 1),
		PointField('z', 8, PointField.FLOAT32, 1)
        ]
        
        self.pointcloud_msg.header.frame_id = "torso_map"
        self.pointcloud_msg.point_step = 12
        self.pointcloud_msg.height = 1
        # self.pointcloud_msg.width = 640
        # self.pointcloud_msg.row_step = self.pointcloud_msg.point_step * 640
        self.pointcloud_msg.is_dense = False
        self.cam_fx = 457.00736
        self.cam_cx = 320.000
        self.cam_fy = 457.00736
        self.cam_cy = 240.000
        
        rospy.spin()

    def colorImageCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)

        outgray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        grayimg = self.bridge.cv2_to_imgmsg(outgray, "mono8")
        self.color_pub.publish(grayimg)

    def depthImageCallback(self, data):
        # try:
            # cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        # except CvBridgeError as e:
            # rospy.loginfo(e)
        # outdep = np.array(cv_image, dtype=np.float32)
        # print(data.step,data.height)
        outdep  = np.frombuffer(data.data,np.float32,-1)
        # print(np.max(outdep),np.min(outdep),np.mean(outdep),np.median(outdep))
        points = []
        
        outdep = outdep.reshape(480,640)[::-1,::]
        outdep = outdep * 1000
        outdep[outdep > 900] = 0 

        for i in range(0,480,13):
            for j in range(0,640,13):
                x = outdep[i,j] / 20
                if x > 1e-4:
                    y = (j - self.cam_cx) * x / self.cam_fx
                    z = (i - self.cam_cy) * x / self.cam_fy
                    points.append([x,-y,-z])

        self.pointcloud_msg.header.stamp = rospy.Time().now()
        self.pointcloud_msg.width = len(points)
        self.pointcloud_msg.row_step = self.pointcloud_msg.point_step * self.pointcloud_msg.width
        self.pointcloud_msg.data = np.asarray(points, np.float32).tostring()
        # print(points)
        # cv2.imwrite('depth_image_16UC1.png', outdep)

        # print(outdep[-150:-145,45:46])
        outdep = np.round(outdep).astype(np.uint16)
        # cv2.
        # depth_map_uint = cv2.convertScaleAbs(outdep, alpha=(255.0/np.max(outdep)))

        # 将32位深度图转换为16位深度图
        # depth_map_16uc1 = cv2.convertScaleAbs(outdep, alpha=(65535.0/1000.0*np.max(outdep)))
        
        
        # depth_normalized = cv2.normalize(outdep * 1000 , None, 0, 60000, cv2.NORM_MINMAX, cv2.CV_16UC1)

        depthimg = self.bridge.cv2_to_imgmsg(outdep, "16UC1")
        self.depth_pub.publish(depthimg)
        self.pointcloud_pub.publish(self.pointcloud_msg)
        # print(self.pointcloud_msg)
        # print("in the depth service!")

if __name__ == '__main__':
    rospy.init_node('sim_image_format_to_slam_node', anonymous=True)  # 初始化ros节点
    rospy.loginfo('node runing...')
    image_converter()  
