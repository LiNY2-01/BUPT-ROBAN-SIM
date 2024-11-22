# 程序运行使用说明（环境配置与参数设置等）

## 仿真环境

打开roscore后，启动V-Rep，打开ai_innovative_roban_sim_task10_upstair.ttt。

之后启动bodyhub和ikmodule：

```Bash
sleep 3s
rosrun  ik_module ik_module_node &
. /home/fan/robot_ros_application/catkin_ws/devel/setup.bash
echo "121" | sudo -S bash bodyhub.sh &
```
过

## 启动定位和建图功能

分别在三个终端中启动：

```Bash
# 终端1
cd ~/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts && python3 ./sim_image_convert_slam_form.py
# 终端2
cd ~/fan/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/normal_sim_game/ai_innovative_roban_sim/scripts && python3 ./sim_pr_convert_pose.py 
# 终端3
roslaunch ros_actions_node sim_octomap.launch
```

## 导航模块使用

进入liny2_nav_dev，使用catkin_make指令编译。

之后终端中输入

```Bash
source devel/setup.bash
roslaunch humanoid_planner_2d humanoid_planner_2d.launch 
```

## 启动轨迹跟踪

```Bash
cd  ~/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/caai_roban_challenge/path_track && python3 Task_path_tracking.py 
```

## 启动键盘控制

```Bash
cd  ~/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/caai_roban_challenge/path_track && python3 key_ctrl.py 
```