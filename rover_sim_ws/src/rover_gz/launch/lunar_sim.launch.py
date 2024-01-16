# Copyright 2022 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import numpy as np

import xacro


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    goal_pose = LaunchConfiguration('goal_pose', default="0,0,0,0")

    # ignition_ros2_control_demos_path = os.path.join(
    #     get_package_share_directory('ign_ros2_control_demos'))

    # xacro_file = os.path.join( get_package_share_directory('rover_gz'), 'models', 'x1_description', 'urdf', 'x1_from_sdf.xacro')
    # xacro_file = os.path.join( get_package_share_directory('rover_gz'), 'models', 'robot_description', 'robot.urdf.xacro')

    sdf_file = os.path.join( get_package_share_directory('rover_gz'), 'models', 'X1 Config 6', 'model.sdf')

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # doc = xacro.parse(open(xacro_file))
    # xacro.process_doc(doc)
    # params = {'robot_description': doc.toxml(), 'use_sim_time': use_sim_time}

    params = {'robot_description': robot_desc, 'use_sim_time': use_sim_time}


    print(params)

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-name', 'diff_drive',
                   '-allow_renaming', 'true',
                   '-x', '0', '-y', '0', '-z', '0.1'],
    )

    joint_state_publisher = Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
    )

    # load_diff_drive_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'diff_drive_base_controller'],
    #     output='screen'
    # )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=['/catkin_ws/rover_sim_ws/src/rover_gz/config/localization.yaml', {'use_sim_time': use_sim_time}]
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/moon/model/diff_drive/link/base_link/sensor/front_laser/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
        output='screen'
    )

    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/moon/model/diff_drive/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        output='screen'
    )

    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/rover/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
        output='screen',
        remappings=[
                ('/rover/odometry', '/odom')
            ]
    )
    
    bridge_odom_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/diff_drive/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
        output='screen',
        remappings=[
                ('/model/diff_drive/tf', '/tf')
            ]
    )

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=[('-d', '/catkin_ws/rover_sim_ws/src/rover_gz/config/slam_map.rviz')]
    )
    # goal_coords = goal_pose.split(",")
    goal_cmd = LaunchDescription([
        # ExecuteProcess(cmd=['ros2', 'topic', 'pub', '/goal_pose', 'geometry_msgs/PoseStamped', "{header: {stamp: {\sec: 0}, frame_id: 'map'}, pose: {\position: {x:" +goal_coords[0]+", y:"+goal_coords[1]+", z:"+goal_coords[2]+"}, orientation: {w:"+goal_coords[3]+"}\}\}"])
        ExecuteProcess(cmd=['ros2', 'topic', 'pub', '/goal_pose', 'geometry_msgs/PoseStamped', "{header: {stamp: {\sec: 0}, frame_id: 'map'}, pose: {\position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"])
    ])

    NUM_OBSTACLES = 15
    OBSTACLE_NAME = 'drc_practice_block_wall'

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
    obstacles = []
    for i in range(NUM_OBSTACLES - 1):
        print('x', obs_coords[i][0], 'y', obs_coords[i][1])
        gzobstacle_cmd = Node( package='ros_gz_sim', executable='create', arguments=[ '-file', '/catkin_ws/rover_sim_ws/src/rover_gz/models/'+OBSTACLE_NAME+'/model.sdf', '-topic', 'robot_description', '-x', str(obs_coords[i][0]), '-y', str(obs_coords[i][1]), '-z', '0.5', '-name', 'Obstacle-'+str(i)], output='screen', )
        obstacles.append(gzobstacle_cmd)
    print(obstacles[0])
    # gzobstacle_cmd = Node( package='ros_gz_sim', executable='create', arguments=[ '-file', '/catkin_ws/rover_sim_ws/src/rover_gz/models/'+OBSTACLE_NAME+'/model.sdf', '-topic', 'robot_description', '-x', '1', '-y', '1', '-z', '0.5', '-name', 'Obstacle-1'], output='screen', )

    world = os.path.join(
        get_package_share_directory('rover_gz'),
        'worlds',
        'moon.world'
    )

    return LaunchDescription([
        bridge,
        bridge_lidar,
        bridge_imu,
        bridge_odom,
        bridge_odom_tf,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('gz_args', [world+' -r -v 4 --render_engine ogre'])]),
        node_robot_state_publisher,
        ignition_spawn_entity,
        *obstacles,
        joint_state_publisher,
        robot_localization_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('slam_toolbox'),
                              'launch', 'online_async_launch.py')]),
            launch_arguments=[('slam_params_file','/catkin_ws/rover_sim_ws/src/rover_gz/config/mapper_params_online_async.yaml' )]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('nav2_bringup'),
                              'launch', 'navigation_launch.py')]),
            launch_arguments=[('params_file','/catkin_ws/rover_sim_ws/src/rover_gz/config/nav2_params.yaml' )]),
        rviz,
        goal_cmd,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])