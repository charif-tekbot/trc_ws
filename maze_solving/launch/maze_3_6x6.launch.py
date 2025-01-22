from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node



def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    model_path = PathJoinSubstitution(
        [FindPackageShare("maze_solving"), "models",]
    )
    
    gz_model = SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=model_path
    )
    
    world_file = PathJoinSubstitution(
        [FindPackageShare("maze_solving"),
        "worlds",
        "maze_3_6x6.world"],
    )
    
    tekbot_description_path = PathJoinSubstitution(
        [FindPackageShare("tekbot_description"),
        "launch",
        "description.launch.py"])
    tekbot_description_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([tekbot_description_path]))

    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("maze_solving"),
        "launch",
        "empty_world.launch.py"],
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments={'world_path': world_file}.items(),
    )
    
    # Launch the robot
    spawn_entity_cmd = Node(
	package='gazebo_ros', 
	executable='spawn_entity.py',
	arguments=['-entity', "tekbot", 
		'-topic', 'robot_description',
		    '-x', "-2.5",
		    '-y', "2.5",
		    '-z', "0",
		    '-Y', "0"],
    parameters=[{'use_sim_time': use_sim_time}],
	output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(gz_model)
    ld.add_action(gazebo_sim)
    ld.add_action(tekbot_description_launch)
    ld.add_action(spawn_entity_cmd)

    return ld
