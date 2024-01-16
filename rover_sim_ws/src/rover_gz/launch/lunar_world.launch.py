#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import numpy as np
import xacro


def generate_launch_description():
    # launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0')
    y_pose = LaunchConfiguration('y_pose', default='0')

    world = os.path.join(
        get_package_share_directory('rover_gz'),
        'worlds',
        'moon.world'
    )

    gzworld_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('gz_args', [world + ' --render_engine ogre'])])

    xacro_file = os.path.join( get_package_share_directory('rover_gz'), 'models', 'x1_description', 'urdf', 'x1_from_sdf.xacro')
    # sdf_file = os.path.join( get_package_share_directory('rover_gz'), 'models', 'X1 Config 6', 'model.sdf')
    # print(sdf_file)
    # with open(sdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time': use_sim_time}

    print(params)


    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                     '-name', 'diff_drive',
                     '-allow_renaming', 'True',
                     '-x', '0', '-y', '0', '-z', '0.1'],
    )

        # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    slam_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('slam_toolbox'),
                              'launch', 'online_async_launch.py')]))
    # robot_localization_node = Node(
    #    package='robot_localization',
    #    executable='ekf_node',
    #    name='ekf_filter_node',
    #    output='screen',
    #    parameters=['/catkin_ws/rover_sim_ws/src/rover_gz/config/localization.yaml', {'use_sim_time': use_sim_time}]
    # )

    NUM_OBSTACLES = 10
    OBSTACLE_NAME = 'Blue cylinder'

    

    # gzobstacle_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'run', 'create')
    #     ),
    #     launch_arguments={ '-file', '/catkin_ws/rover_sim_ws/src/rover_gz/models/'+OBSTACLE_NAME+'/model.sdf', '-topic', 'robot_description', 'x', 5, 'y', 5, 'z', 0}
    # )
    # obs_coords = np.random.uniform(5,10,[2,NUM_OBSTACLES])
    # for i in range(NUM_OBSTACLES)
    # gzobstacle_cmd = Node( package='ros_gz_sim', executable='create', arguments=[ '-file', '/catkin_ws/rover_sim_ws/src/rover_gz/models/'+OBSTACLE_NAME+'/model.sdf', '-topic', 'robot_description', 'x', '5', 'y', '5', 'z', '0'], output='screen', )
    # gzclient_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
    #     )
    # )

    # robot_state_publisher_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
    #     ),
    #     launch_arguments={'use_sim_time': use_sim_time}.items()
    # )

    # spawn_turtlebot_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
    #     ),
    #     launch_arguments={
    #         'x_pose': x_pose,
    #         'y_pose': y_pose
    #     }.items()
    # )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(bridge)
    ld.add_action(gzworld_cmd)
    ld.add_action(gz_spawn_entity)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(slam_node)
    # ld.add_action(load_joint_state_controller)
    # ld.add_action(load_diff_drive_controller)
    # ld.add_action(robot_localization_node)
    # ld.add_action(gzobstacle_cmd)
    NUM_OBSTACLES = 15
    OBSTACLE_NAME = 'Blue cylinder'

    

    # gzobstacle_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'run', 'create')
    #     ),
    #     launch_arguments={ '-file', '/catkin_ws/rover_sim_ws/src/rover_gz/models/'+OBSTACLE_NAME+'/model.sdf', '-topic', 'robot_description', 'x', 5, 'y', 5, 'z', 0}
    # )
    x_coords = np.random.uniform(1, 10, NUM_OBSTACLES)
    y_coords = np.random.uniform(-10, 10, NUM_OBSTACLES)
    # X2D,Y2D = np.meshgrid(y_coords,x_coords)
    obs_coords = np.column_stack((x_coords, y_coords))
    print(obs_coords.shape)
    # obs_coords = np.random.randint(-10,10,[NUM_OBSTACLES,2])
    for i in range(NUM_OBSTACLES - 1):
        print('x', obs_coords[i][0], 'y', obs_coords[i][1])
        gzobstacle_cmd = Node( package='ros_gz_sim', executable='create', arguments=[ '-file', '/catkin_ws/rover_sim_ws/src/rover_gz/models/'+OBSTACLE_NAME+'/model.sdf', '-topic', 'robot_description', '-x', str(obs_coords[i][0]), '-y', str(obs_coords[i][1]), '-z', '0', '-name', 'Obstacle-'+str(i)], output='screen', )
        ld.add_action(gzobstacle_cmd)

    # gzclient_cmd = IncludeLaunchDescription(
    # ld.add_action(robot_state_publisher_cmd)
    # ld.add_action(spawn_turtlebot_cmd)

    return ld
