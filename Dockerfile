FROM osrf/ros:humble-desktop-full
ENV IGNITION_VERSION=fortress
ENV GZ_VERSION=fortress
RUN apt update && \
   apt install -y mesa-utils && \
   apt install -y ros-humble-gazebo-ros-pkgs && \
   sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
   apt-get update; exit 0 && \
   apt -y install ros-humble-ros-gz && \
   apt -y install ros-humble-ros2-control && \
   apt -y install ros-humble-ros2-controllers && \
   apt -y install ros-humble-ign-ros2-control
   curl -sSL http://get.gazebosim.org | sh

ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib
ENV IGN_GAZEBO_RESOURCE_PATH=/catkin_ws/rover_sim_ws/src/rover_gz/models
