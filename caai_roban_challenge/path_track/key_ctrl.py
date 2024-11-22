#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import tty
import termios
import select

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

class Museum():
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node("walk_telecontrol", anonymous=True)      
        
        # 发布 /move_base_simple/goal 话题
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=2)
        
        # 订阅机器人到达信号
        self.arrive_sub  = rospy.Subscriber("/roban_arrived", Bool, self.arriveCb, queue_size=1)
        
        # 标记机器人是否到达目标点
        self.arrived = False
        
        # 定义展位目标点的列表
        self.museum_keypts = [
            # [x, y, z, ox, oy, oz, ow]
            [2.790, 2.200, 0, 0, 0, -0.9998766443022261, 0.015706564835109935],  # 1号展位
            [3.205, -2.404, 0, 0, 0, 0.0026814663313504987, 0.9999964048626944],  # 2号展位
            [-3.670, 2.766, 0, 0, 0, 0.7047431459614374, 0.7094625418021562],     # 3号展位
            [-2.540, -3.000, 0, 0, 0, 0.9997855690400177, 0.020707871434023622],  # 4号展位
        ]
    
    # 到达回调函数
    def arriveCb(self, msg):
        self.arrived = msg.data
        if self.arrived:
            print("已到达展位")

    # 显示控制说明
    def printTeleInfo(self):
        print ('%-15s%-15s%-15s%-15s' % ('w--1号展位', 'e--2号展位', 'r--3号展位','t--4号展位'))
        print('%-15s%s\n' % (' ', 'q--退出'))

    # 发布目标点
    def pubGoal(self, idx):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time().now()
        msg.pose.position.x = self.museum_keypts[idx][0]
        msg.pose.position.y = self.museum_keypts[idx][1]
        msg.pose.position.z = self.museum_keypts[idx][2]
        msg.pose.orientation.x = self.museum_keypts[idx][3]
        msg.pose.orientation.y = self.museum_keypts[idx][4]
        msg.pose.orientation.z = self.museum_keypts[idx][5]
        msg.pose.orientation.w = self.museum_keypts[idx][6]

        # 发布到 move_base 的目标话题
        self.goal_pub.publish(msg)
        print("目标已发送，等待到达...")

    # 获取键盘输入
    def getch(self, str=''):
        print(str,)
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print( ch)
        return ch

    # 主键盘控制循环
    def keyboard_poll(self):
        while not rospy.is_shutdown():
            cmd = self.getch('key:')
            if cmd == 'w':
                self.pubGoal(0)  # 1号展位
            elif cmd == 'e':
                self.pubGoal(1)  # 2号展位
            elif cmd == 'r':
                self.pubGoal(2)  # 3号展位
            elif cmd == 't':
                self.pubGoal(3)  # 4号展位
            elif cmd == 'q':
                return

    # 启动控制
    def start(self):
        self.printTeleInfo()
        self.keyboard_poll()
        rospy.signal_shutdown('退出程序')

if __name__ == '__main__':
    Museum().start()
