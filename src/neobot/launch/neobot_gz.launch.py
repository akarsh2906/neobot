#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    neobot_path = FindPackageShare(package='neobot').find('neobot')
    gz_sim_share = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration("world_file")
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration("use_rviz")

    default_use_rviz = "False"
    default_world_file_path =  join(neobot_path, "worlds", "small_warehouse.sdf")
    default_rviz_config_path = join(neobot_path, 'rviz/rviz_config.rviz')
    
    
    
    # Declare the launch arguments  
    declare_world_path_cmd = DeclareLaunchArgument(
        name='world_file', 
        default_value=default_world_file_path, 
        description='Path to default world file')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value=default_use_rviz,
        description='Use RViz if True')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')
    




    start_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : PythonExpression(["'", world_file, " -r'"])

        }.items()
    )

    spawn_neobot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(neobot_path, "launch", "gz_spawn.launch.py")),
        launch_arguments={
            # Pass any arguments if your spawn.launch.py requires
        }.items()
    )

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])


    append_worlds_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=join(neobot_path, "worlds"))
    
    append_models_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=join(neobot_path, "models"))

    ld = LaunchDescription()

    ld.add_action(declare_world_path_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(append_worlds_path)
    ld.add_action(append_models_path)

    ld.add_action(start_gz_sim)
    ld.add_action(spawn_neobot)
    ld.add_action(start_rviz_cmd)

    return ld