#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import os 
import rospy
import sys
import signal
import rospy
from std_msgs.msg import Float64
import time


guan=input('请输入关卡的序号（1-3）：')
guan = int (guan)



if guan == 1:
    # os.system("gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run ; bash ./roscore.sh ; exec bash\"' && gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/ ; bash ./start_ik.sh ; exec bash\"' && gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/start_vrep/ ; bash ./start_vrep1.sh;exec bash\"' && gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/ ; bash ./start_bodyhub.sh ;exec bash\"' && gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/python_P/ ; bash ./python1.sh;exec bash\"' ")
    os.system("gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/start_vrep; bash ./start_vrep1.sh;exec bash\"' && gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run; bash ./node_start.sh ;exec bash\"' ")
elif guan == 2:
    os.system("gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/start_vrep/; bash ./start_vrep3.sh;exec bash\"' && gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run; bash ./node_start.sh ;exec bash\"'")
elif guan == 3:
    os.system("gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/start_vrep/; bash ./start_vrep7.sh;exec bash\"' && gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run; bash ./node_start.sh ;exec bash\"' ")
elif guan == 0:
    os.system("gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/start_vrep/; bash ./start_vrep1.sh;exec bash\"' && gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run; bash ./node_start.sh ;exec bash\"'")
# elif guan == 4:
#     os.system("gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/start_vrep/; bash ./start_vrep4.sh;exec bash\"' && gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run; bash ./node_start.sh ;exec bash\"' ")
# elif guan == 5:
#     os.system("gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/start_vrep/; bash ./start_vrep5.sh;exec bash\"' && gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run; bash ./node_start.sh ;exec bash\"' ")
# elif guan == 6:
#     os.system("gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/start_vrep/; bash ./start_vrep6.sh;exec bash\"' && gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run; bash ./node_start.sh ;exec bash\"' ")
# elif guan == 7:
#     os.system("gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/start_vrep/; bash ./start_vrep7.sh;exec bash\"' && gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run; bash ./node_start.sh ;exec bash\"' ")
# elif guan == 8:
#     os.system("gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/start_vrep/; bash ./start_vrep8.sh;exec bash\"' && gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run; bash ./node_start.sh ;exec bash\"' ")
# elif guan == 9:
#     os.system("gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/start_vrep/; bash ./start_vrep9.sh;exec bash\"' && gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run; bash ./node_start.sh ;exec bash\"' ")
else:
    os.system("gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/start_vrep/; bash ./start_vrep10.sh;exec bash\"' && gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run; bash ./node_start.sh ;exec bash\"' ")


def callback(data):
    global time_total,vrep_start_flag
    time_total= data.data
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", x)
    vrep_start_flag = vrep_start_flag+1
    
def listener():

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
    
    
    rospy.Subscriber("/sim/joint/time", Float64, callback)

        # spin() simply keeps python from exiting until this node is stopped
    
def talker():
    global time_total
    pub = rospy.Publisher('/playerStart', Float64, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = 0.0
        #rospy.loginfo(hello_str)
        pub.publish(hello_str)
        print('\r Ctrl+C to break; total time use .... %f' % time_total, end="")
        rate.sleep()


if __name__ == '__main__':
    try:
        global time_total,vrep_start_flag
        rospy.init_node('listener', anonymous=True)
        rospy.loginfo("tool start...")
        listener()
        time_total = 0
        vrep_start_flag = 0

        print("vrep 启动后，请在 vrep 中按 ok,等待中。。。")
        while vrep_start_flag < 20:
            time.sleep(1)

        print("将开始运行用户程序")
        time.sleep(40)
        if guan == 1:
            os.system(" gnome-terminal -e 'bash -c \" cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/python_P/; bash ./python1.sh;exec bash\"' ")
        elif guan == 2:
            os.system(" gnome-terminal -e 'bash -c \" cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/python_P/; bash ./python2.sh;exec bash\"' ")
        elif guan == 3:
            os.system(" gnome-terminal -e 'bash -c \" cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/python_P/; bash ./python3.sh;exec bash\"' ")
        # elif guan ==4:
        #     os.system(" gnome-terminal -e 'bash -c \" cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/python_P/; bash ./python4.sh;exec bash\"' ")
        # elif guan == 5:
        #     os.system(" gnome-terminal -e 'bash -c \" cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/python_P/; bash ./python5.sh;exec bash\"' ")
        # elif guan == 6:
        #     os.system(" gnome-terminal -e 'bash -c \" cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/python_P/; bash ./python6.sh;exec bash\"' ")
        # elif guan == 7:
        #     os.system(" gnome-terminal -e 'bash -c \" cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/python_P/; bash ./python7.sh;exec bash\"' ")
        # elif guan == 8:
        #     os.system(" gnome-terminal -e 'bash -c \" cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/python_P/; bash ./python8.sh;exec bash\"' ")
        # elif guan == 9:
        #     os.system(" gnome-terminal -e 'bash -c \" cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/python_P/; bash ./python9.sh;exec bash\"' ")
        elif guan == 0:
            os.system(" gnome-terminal -e 'bash -c \" cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/python_P/; bash ./demo.sh;exec bash\"' ")
        else:
            os.system(" gnome-terminal -e 'bash -c \" cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run/python_P/; bash ./demo.sh;exec bash\"' ")
        talker()
        while True:
            Q_uit = raw_input("以上显示的是仿真结束的总时间！    输入q+Enter为退出所有端口!")
            if Q_uit ==  'q':
                #os.system("gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/ ; bash ./kill_ik.sh; exec bash\"' && gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/ ; bash ./star_kill_launch.sh;exec bash\"' && gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/ ; bash ./kill_vrep.sh;exec bash\"'")
                os.system("gnome-terminal -e 'bash -c \"cd /home/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts/auto_run ; bash ./kill_all.sh; exec bash\"' ")
                break
            else:
                print("输入指令有误！请重新输入q+Enter！")

    except rospy.ROSInterruptException:
        pass






