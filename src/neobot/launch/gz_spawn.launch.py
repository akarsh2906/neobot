#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare(package='neobot').find('neobot')
    position_x = '0.0'
    position_y = '0.0'
    orientation_yaw = '0.0'
    default_urdf_path = join(pkg_share, 'urdf/neobot_description.urdf')
    default_model_path = join(pkg_share, 'models/neobot_model.sdf')
    ros_gz_bridge_config_path = join(pkg_share, 'config/ros_gz_bridge.yaml')
    robot_name_in_urdf = 'neobot'
    

    # Launch configuration variables specific to simulation
    model = LaunchConfiguration('model')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_sim_time = LaunchConfiguration('use_sim_time')


    # Declare the launch arguments  
    declare_urdf_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=default_urdf_path, 
        description='Absolute path to robot urdf file')
        
    
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    

    declare_ros_gz_bridge = DeclareLaunchArgument(
        name='bridge_config',
        default_value=ros_gz_bridge_config_path,
        description='Bridge config file')
    


    # Specify the actions

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 
        'robot_description': Command(['xacro ', model])}],
        arguments=[default_urdf_path])


    # Spawn neobot in open GZ world
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file", default_model_path,
            "-name", "neobot",
            "-allow_renaming", "true",
            "-z", "0.2",
            "-x", position_x,
            "-y", position_y,
            "-Y", orientation_yaw
        ]
    )

    wheel_speed_pub = Node(
        package="neobot_control",
        executable="wheel_speed_pub"
    )
    

    # # Start ROS_GZ Bridge
    # start_gz_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
    #         "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    #         "/odom_encoders@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    #         "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    #         "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
    #         # "/tf_truth@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
    #         "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    #         "/camera_link/image@sensor_msgs/msg/Image[gz.msgs.Image",
    #         "/camera_link/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
    #         "/camera_link/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
    #         "/camera_link/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
    #         "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
    #         "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
    #         "/left_wheel_speed@std_msgs/msg/Float64@gz.msgs.Double",
    #         "/right_wheel_speed@std_msgs/msg/Float64@gz.msgs.Double"
            
    #     ],
    #     remappings=[
    #         # ('/camera_info', '/camera_link/camera_info')
    #     ]
    # )

    start_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{"config_file": ros_gz_bridge_config_path}]
    )




    
    # Create the launch description and populate
    ld = LaunchDescription()


    # Declare the launch options
    ld.add_action(declare_urdf_path_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)  
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_ros_gz_bridge)


    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(gz_spawn_entity)
    ld.add_action(wheel_speed_pub)
    ld.add_action(start_gz_bridge)


    return ld