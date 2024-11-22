#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
# import quaternion
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# from sensor_msgs.point_cloud2 import PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped,TransformStamped ,PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray

# from sensor_msgs.msg import PointCloud2
# from sensor_msgs.msg import PointCloud
# from sensor_msgs.msg import PointField


def pr_callback(data  ):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = data.data[0]
    msg.pose.position.y = data.data[1]
    msg.pose.position.z = data.data[2]

    msg.pose.orientation.x = data.data[4]
    msg.pose.orientation.y = data.data[5]
    msg.pose.orientation.z = data.data[6]
    msg.pose.orientation.w = data.data[3]
    pose_pub.publish(msg)
    global msgcov
    msgcov = PoseWithCovarianceStamped()
    msgcov.header.frame_id = 'map'
    msgcov.header.stamp = rospy.Time.now()

    msgcov.pose.pose.position.x = data.data[0]
    msgcov.pose.pose.position.y = data.data[1]
    msgcov.pose.pose.position.z = data.data[2]

    msgcov.pose.pose.orientation.x = data.data[4]
    msgcov.pose.pose.orientation.y = data.data[5]
    msgcov.pose.pose.orientation.z = data.data[6]
    msgcov.pose.pose.orientation.w = data.data[3]
    posecov_pub.publish(msgcov)

    transform_msg.header.stamp = rospy.Time.now()
    transform_msg.transform.translation.x = data.data[0]
    transform_msg.transform.translation.y = data.data[1]
    transform_msg.transform.translation.z = data.data[2]

    q = np.array([data.data[4],data.data[5],data.data[6],data.data[3]])
    # print(q)
    # q = np.array([-data.data[3],-data.data[4],-data.data[5],data.data[6]])/np.linalg.norm(q)
    # q = -q
    transform_msg.transform.rotation.x = q[0]
    transform_msg.transform.rotation.y = q[1]
    transform_msg.transform.rotation.z = q[2]
    transform_msg.transform.rotation.w = q[3]



if __name__ == "__main__":
    rospy.init_node("pr_convert")
    rospy.loginfo('node runing...')
    pr_sub = rospy.Subscriber("/sim/torso/PR",Float64MultiArray,pr_callback,queue_size=10)
    pose_pub = rospy.Publisher("/sim/torso/Pose",PoseStamped,queue_size=1)
    posecov_pub = rospy.Publisher("/sim/torso/PoseCov",PoseWithCovarianceStamped,queue_size=1)
    planpos_pub = rospy.Publisher("/sim/torso/PoseCov_plan",PoseWithCovarianceStamped,queue_size=1)

    msgcov = PoseWithCovarianceStamped()
    tf_broadcaster = TransformBroadcaster()
    transform_msg = TransformStamped()
    transform_msg.header.frame_id = 'map'
    transform_msg.child_frame_id = 'torso_map'
    rate = rospy.Rate(5)
    cnt = 0 
    while not rospy.is_shutdown():
        tf_broadcaster.sendTransform(transform_msg)
        if cnt == 3:
            planpos_pub.publish(msgcov)
            cnt = 0
        rate.sleep()  
        cnt += 1