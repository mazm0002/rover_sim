#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" && \
cd /catkin_ws/rover_sim_ws && colcon build
source "/catkin_ws/rover_sim_ws/install/setup.bash" && \
# exec "rviz2" 
echo Please enter goal pose in the following format position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
read goal_pose
ros2 launch rover_gz lunar_sim.launch.py & ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
