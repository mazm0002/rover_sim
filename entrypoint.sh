#!/bin/bash
set -e

# setup ros2 environment
trap 'return' SIGINT
source "/opt/ros/$ROS_DISTRO/setup.bash" && \
cd /catkin_ws/rover_sim_ws && colcon build
source "/catkin_ws/rover_sim_ws/install/setup.bash" && \
# exec "rviz2" 
# echo Please enter goal pose in the following format: x y z w
IFS=' ' read -ra goal_pose -p "Please enter goal pose in the following format: x y z w : "
ros2 launch rover_gz lunar_sim.launch.py & ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: ${goal_pose[0]}, y: ${goal_pose[1]}, z: ${goal_pose[2]}}, orientation: {w: ${goal_pose[3]}}}}"
