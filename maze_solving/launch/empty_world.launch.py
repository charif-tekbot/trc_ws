from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions
from launch.conditions import IfCondition
import os
import xacro

from ament_index_python.packages import get_package_share_directory

from pathlib import Path

ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value='',
                          description='The world path, by default is empty.world'),
    DeclareLaunchArgument('gui',
            default_value="true",
            description='if client gui launched'),  
]


def generate_launch_description():

    # Launch args
    world_path = LaunchConfiguration('world_path')
    
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )
    gazebo_2 = ExecuteProcess(
            cmd=['gazebo',  '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen')
   
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    # ld.add_action(gazebo_2)


    return ld

