import os
from pathlib import Path


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.command import Command
from launch.substitutions.find_executable import FindExecutable

from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('neobot')

    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
            'y': LaunchConfiguration('y_pose', default='0.00'),
            'z': LaunchConfiguration('z_pose', default='0.00'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='neobot',
        description='name of the robot')

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(bringup_dir, 'models', 'neobot_model.sdf'),
        description='Full path to robot sdf file to spawn the robot in gazebo')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        parameters=[
            {
                'config_file': os.path.join(
                    bringup_dir, 'config', 'ros_gz_bridge.yaml'
                ),
                'expand_gz_topic_names': True,
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace=namespace,
        arguments=[
            '-name', robot_name,
            '-string', Command([
                FindExecutable(name='xacro'), ' ', 'namespace:=',
                LaunchConfiguration('namespace'), ' ', robot_sdf]),
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']]
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(bringup_dir, 'models'))
    set_env_vars_resources2 = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(Path(os.path.join(bringup_dir)).parent.resolve()))

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)

    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_resources2)

    ld.add_action(bridge)
    ld.add_action(spawn_model)
    return ld