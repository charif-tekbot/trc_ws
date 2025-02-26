import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


ARGUMENTS = [
    DeclareLaunchArgument('x_init',
            default_value='0.0',
            description='robot initial x'),
    DeclareLaunchArgument('y_init',
            default_value='0.0',
            description='robot initial y'),
    DeclareLaunchArgument('z_init',
            default_value='0.0',
            description='robot initial z'),
    DeclareLaunchArgument('yaw_init',
            default_value='0.0',
            description='robot initial yaw')
]

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='true')
  x3_description_path = PathJoinSubstitution(
        [FindPackageShare("x3_description"),
        "launch",
        "x3_description.launch.py"]
  )

  teleop_node = Node(
    package= "yahboomcar_ctrl",
    executable="yahboom_joy_X3",
    name="joy_X3",
    parameters=[{'use_sim_time': use_sim_time}]
  )

  joy_node = Node(
        package='joy',
        executable='joy_node',
    )

  x3_description_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([x3_description_path]))

  spawn_x_val = LaunchConfiguration('x_init')
  spawn_y_val = LaunchConfiguration('y_init')
  spawn_z_val = LaunchConfiguration('z_init')
  spawn_yaw_val = LaunchConfiguration('yaw_init')

  gazebo = ExecuteProcess(
            cmd=['gazebo', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so',],
            output='screen')
  
  spawn_x2_mecanum_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller', '-c', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
  spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
  
  rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("x3_description"),
        "config",
        "x3_des.rviz"]
  )

  rviz_x3 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
  )

  # Launch the robot
  spawn_entity_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', "rosmaster_X3", 
                '-topic', 'robot_description',
                    '-x', spawn_x_val,
                    '-y', spawn_y_val,
                    '-z', spawn_z_val,
                    '-Y', spawn_yaw_val],
                    output='screen',
    parameters=[{'use_sim_time': use_sim_time}])
  
  
  mecanum_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_x2_mecanum_controller],
        )
    )
  controllers_after_spawn =  RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_cmd,
            on_exit=[spawn_joint_state_broadcaster, spawn_x2_mecanum_controller]
    ))
  
  # Create the launch description and populate
  ld = LaunchDescription(ARGUMENTS)

  # Add any actions
  ld.add_action(x3_description_launch)
  ld.add_action(teleop_node)
  ld.add_action(spawn_entity_cmd)
#   ld.add_action(mecanum_controller_after_joint_state_broadcaster)
#   ld.add_action(controllers_after_spawn)
  ld.add_action(rviz_x3)
  ld.add_action(gazebo)
  ld.add_action(joy_node)
  
  return ld
