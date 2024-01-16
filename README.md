# rover_sim
![alt text](https://raw.githubusercontent.com/mazm0002/rover_sim/main/Screenshot%202024-01-16%20173225.png?token=GHSAT0AAAAAACMPQUU7NCTI4MRTZ6WVNLTUZNGEPQQ)
## Prerequisites
### Environment
Testing was done in Ubuntu 22.04 with ROS2 Humble and Gazebo Fortress, ymmv with different setups.
### Installation Instructions
First install docker engine: https://docs.docker.com/engine/install/ubuntu/ \
Then clone this repository and build the Docker image
```` bash
git clone https://github.com/mazm0002/rover_sim.git
cd rover_sim
docker build -t ros_gazebo .
````
Then install NVIDIA container toolkit in order to use a dedicated GPU within the container: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

Finally start the Docker container, enter the goal pose, then this should open the Gazebo GUI as well as Rviz2 with the SLAM map 
> :warning: **This command needs to be run while in the rover_sim folder**
```` bash
sudo docker run -it --device=/dev/dxg -e DISPLAY=$DISPLAY --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -e MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA --gpus all --mount src=$(pwd),target=/catkin_ws,type=bind ros_gazebo
````

## Limitations
* This implementation is currently incomplete, the robot does not follow the Nav2 Goal due to issues with the timestamps in Lidar sensor measurements.
* Testing was done in WSL with access to a dedicated GPU. If using a virtual machine ensure that your GPU is passing through.
