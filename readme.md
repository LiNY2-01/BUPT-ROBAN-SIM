# BUPT-ROBAN-SIM 

北邮智能机器人交互实验仿真作业，仅供参考

## Build ROBAN V-Rep SIM Environment on your PC

due to the v-rep sim is extremely slow in cpu, you can try to build the sim on you own pc. I have built the sim on my ubuntu 20.04 PC .

### WSL (NOT TESTED)

TODO......

### Ubuntu 20.04 

TODO......

docker

## 程序运行使用说明（环境配置与参数设置等）

### 仿真环境

打开roscore后，启动V-Rep，打开ai_innovative_roban_sim_task10_upstair.ttt。

之后启动bodyhub和ikmodule：

```Bash
sleep 3s
rosrun  ik_module ik_module_node &
. /home/fan/robot_ros_application/catkin_ws/devel/setup.bash
echo "121" | sudo -S bash bodyhub.sh &
```


### 启动定位和建图功能

分别在三个终端中启动：

```Bash
# 终端1，转换图像格式
# 该脚本会将V-Rep中的图像转换为ros中可以使用的格式，以及转换深度图
cd ~/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts && python3 ./sim_image_convert_slam_form.py
# 终端2，转换坐标系
cd ~/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts && python3 ./sim_pr_convert_pose.py 
# 终端3，打开octomap建图
roslaunch ros_actions_node sim_octomap.launch
# 终端4，通过rviz查看建图结果
rosrun rviz rviz -d roban.rviz
```

### 导航模块使用

进入 nav_dev，使用catkin_make指令编译。

``` bash
cd nav_dev
catkin_make 
# or 'catkin build'
```

之后终端中输入

```Bash
source devel/setup.bash
roslaunch humanoid_planner_2d humanoid_planner_2d.launch 
```

#### 启动轨迹跟踪

```Bash
cd  ~/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/caai_roban_challenge/path_track && python3 Task_path_tracking.py 
```

#### 启动键盘控制

```Bash
cd  ~/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/caai_roban_challenge/path_track && python3 key_ctrl.py 
```

# Known Issues

1. The pose takes the truth-value from sim. You need to run a slam system like ORB-SLAM or VINS-Fusion to get odom when you deploy the code to the real robot.
2. This project only use the path plan, not considering the footstep planning. You can try the **footstep planning from the humanoid_navigation** to achieve a better motion.

# Reference 

[humanoid_navigation](http://wiki.ros.org/humanoid_navigation) : ROS stack with footstep planning and localization for humanoid robots

