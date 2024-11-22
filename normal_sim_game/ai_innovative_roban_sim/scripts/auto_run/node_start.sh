
sleep 3s

rosrun  ik_module ik_module_node &

. /home/fan/robot_ros_application/catkin_ws/devel/setup.bash
echo "121" | sudo -S bash bodyhub.sh &

