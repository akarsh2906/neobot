from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_nav2_dir = FindPackageShare(package='neobot_nav2_bringup').find('neobot_nav2_bringup')
    neobot_path = FindPackageShare(package='neobot').find('neobot')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')
    rviz_config_file = join(neobot_path, 'rviz', 'nav2_default_config.rviz')



    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_nav2_dir, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': join(neobot_path, 'maps', 'neobot_map.yaml'),
            'params_file': join(neobot_path, 'config', 'nav2_params.yaml'),
            'package_path': neobot_path, 
        }.items()
    )

    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[['-d', rviz_config_file]]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': join(neobot_path, 'maps', 'neobot_map.yaml')}],
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )


    ld = LaunchDescription()

    ld.add_action(nav2_launch_cmd)
    ld.add_action(rviz_launch_cmd)
    ld.add_action(map_server_node)
    ld.add_action(static_transform_publisher_node)


    return ld

