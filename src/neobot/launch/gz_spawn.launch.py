#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare(package='neobot').find('neobot')
    position_x = '0.0'
    position_y = '0.0'
    orientation_yaw = '0.0'
    default_urdf_path = join(pkg_share, 'urdf/neobot.xacro')
    ros_gz_bridge_config_path = join(pkg_share, 'config/ros_gz_bridge.yaml')
    ros_gz_bridge_no_ros2_control_config_path = join(pkg_share, 'config/ros_gz_bridge_no_ros2_control.yaml')
    twist_mux_params = join(pkg_share,'config','twist_mux.yaml')
    robot_name_in_urdf = 'neobot'
    

    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_lidar = LaunchConfiguration("use_lidar", default=True)
    use_imu = LaunchConfiguration("use_imu", default=True)
    use_camera = LaunchConfiguration("use_camera", default=True)
    use_ros2_control = LaunchConfiguration("use_ros2_control", default=True)
    publish_ground_truth = LaunchConfiguration("publish_ground_truth", default=True)
    odometry_source = LaunchConfiguration("odometry_source", default='world') # 'world' or 'encoder'
    


    # Declare the launch arguments  
    declare_urdf_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=default_urdf_path, 
        description='Absolute path to robot urdf file')
        

        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    


    # Specify the actions

    
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
                    {'use_sim_time': use_sim_time,
                        'robot_description': Command( \
                    ['xacro ', join(default_urdf_path),
                    ' use_imu:=', use_imu,
                    ' use_camera:=', use_camera,
                    ' use_lidar:=', use_lidar,
                    ' publish_ground_truth:=', publish_ground_truth,
                    ' use_ros2_control:=', use_ros2_control,
                    ' odometry_source:=', odometry_source 
                    ])
                    }]
    )


    # Spawn neobot in open GZ world
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "neobot",
            "-allow_renaming", "true",
            "-z", "0.2",    
            "-x", position_x,
            "-y", position_y,
            "-Y", orientation_yaw
        ]
    )



    start_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{"config_file": ros_gz_bridge_config_path}],
        condition=IfCondition(use_ros2_control)
    )
    start_gz_bridge_no_ros2_control = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{"config_file": ros_gz_bridge_no_ros2_control_config_path}],
        condition=UnlessCondition(use_ros2_control)
    )

    start_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["depth_camera/image", "depth_camera/depth_image"]
    )

    
    start_diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller"],
        condition=IfCondition(use_ros2_control)
    )

    start_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        condition=IfCondition(use_ros2_control)
    )

    
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel_out','/diff_controller/cmd_vel_unstamped')]
        )

    twist_stamper = Node(
            package='twist_stamper',
            executable='twist_stamper',
            name='twist_stamper',
            output='screen',
            remappings=[
                ('/cmd_vel_in', '/diff_controller/cmd_vel_unstamped'),
                ('/cmd_vel_out', '/diff_controller/cmd_vel')
            ]
        )



    # Create the launch description and populate
    ld = LaunchDescription()


    # Declare the launch options
    ld.add_action(declare_urdf_path_cmd)
    ld.add_action(declare_use_sim_time_cmd)


    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(gz_spawn_entity)
    ld.add_action(start_gz_bridge)
    ld.add_action(start_gz_bridge_no_ros2_control)
    ld.add_action(start_gz_image_bridge)
    ld.add_action(start_diff_drive_controller)
    ld.add_action(start_joint_state_broadcaster)
    ld.add_action(twist_mux)
    ld.add_action(twist_stamper)


    return ld