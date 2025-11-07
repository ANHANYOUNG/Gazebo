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
# git test
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_example_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_example_gazebo')
    pkg_project_description = get_package_share_directory('ros_gz_example_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'tracked_v3', 'tracked_v3.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'agriculture_v3.sdf'
        ])}.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_example_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # 시뮬레이션 모델의 TF 설정을 위한 정적 변환 노드들
    static_base_prefix_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_prefix",
        arguments=["0","0","0","0","0","0", "chassis_link", "tracked_v3/chassis_link"],
        output="screen",
    )
    static_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_imu_to_base",
        arguments=["0.1", "0", "0", "0", "0", "0", "chassis_link", "tracked_v3/chassis_link/imu_sensor"],
        output="screen",
    )
    static_gps_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_gps_to_base",
        arguments=["0.1", "0", "0", "0", "0", "0", "chassis_link", "tracked_v3/chassis_link/navsat_sensor"],
        output="screen",
    )


    return LaunchDescription([
        # --- 시뮬레이션 (노트북) 측 실행 노드 ---
        gz_sim,
        bridge,
        robot_state_publisher,
        static_base_prefix_tf,
        static_imu_tf,
        static_gps_tf,

    ])