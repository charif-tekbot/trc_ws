from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode



def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    maze_world_and_robot_path = PathJoinSubstitution(
        [FindPackageShare("maze_solving"),
        "launch",
        "maze_3_6x6.launch.py"])
    
    tekbot_teleop_path = PathJoinSubstitution(
        [FindPackageShare("tekbot_control"),
        "launch",
        "tekbot_teleop_joy.launch.py"]
    )

    slam_params_file = PathJoinSubstitution(
        [FindPackageShare('maze_solving'), 'config', ('mapper_params_online_async.yaml')]
    )

    map_static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map',
            arguments=[
                '-2.5', '2.5', '0', 
                '0', '0', '0',
                'map',
                'odom'
            ],
            output='screen'
    )

    start_async_slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        namespace=''
    )

    maze_world_and_robot_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([maze_world_and_robot_path]))
    tekbot_teleop_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([tekbot_teleop_path]))

    ld = LaunchDescription()
    # ld.add_action(map_static_tf)
    ld.add_action(maze_world_and_robot_launch)
    ld.add_action(tekbot_teleop_launch) # + ekf loc
    ld.add_action(start_async_slam_toolbox_node)


    return ld
