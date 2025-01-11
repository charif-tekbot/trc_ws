from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    tekbot_urdf = PathJoinSubstitution(
        [FindPackageShare("tekbot_description"),
        "urdf",
        "tekbot.urdf"]
    )

    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', tekbot_urdf]), 'use_sim_time': use_sim_time}],
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}])
    ])
