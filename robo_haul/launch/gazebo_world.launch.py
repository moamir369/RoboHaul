#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    package_name = 'robo_haul'
    pkg_share = get_package_share_directory(package_name)

    #eklediğimiz kısım
    gazebo_models_path = os.path.join(pkg_share, 'models')


    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    

    default_world = os.path.join(pkg_share, 'worlds', 'factory_world.world')
    robot_description_file = os.path.join(pkg_share, 'urdf', 'robo_haul.urdf.xacro')

    robot_description = Command(['xacro ', robot_description_file])

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to world file'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robo_haul',
            '-topic', 'robot_description',
            '-x', '-5.5',
            '-y', '-4.5',
            '-z', '0.2' 
        ],
        output='screen'
    )

      #eklediğimiz kısım
    set_gazebo_models = SetEnvironmentVariable(
    name='GAZEBO_MODEL_PATH',
    value=gazebo_models_path
    )

    return LaunchDescription([
        set_gazebo_models,
        declare_use_sim_time,
        declare_world,
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])