import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    neobot_path = get_package_share_directory("neobot")
    # neobot_sdf_path = os.path.join(neobot_path,"models/neobot_model.sdf")
    neobot_sdf_path = "/home/akarsh/nmodels/neobot_model.sdf"
    neobot_urdf_path = os.path.join(neobot_path,"urdf/neobot_description.urdf")

    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
    odometry_source = LaunchConfiguration("odometry_source")


    start_gz_bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    # "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/model/neobot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                    "/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/rgb_camera@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    "/realsense@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    "/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                    "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
                    # "/world/default/model/bcr_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"
                ],
                remappings=[
                    ('/model/neobot/tf', '/tf'),
                    ('/camera_info', '/realsense/camera_info')
                ]
            )
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file", neobot_sdf_path,
            "-name", "noebot",
            "-allow_renaming", "true",
            "-z", "0.28",
            "-x", "0.00",
            "-y", "0.00",
            "-Y", "0.00"
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
                    {'use_sim_time': use_sim_time, 
                    'robot_description': Command(['xacro ', model])}],
        remappings=remappings,
        arguments=[default_model_path]
    )
    

    transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--x", "0.0",
                    "--y", "0.0",
                    "--z", "0.0",
                    "--yaw", "0.0",
                    "--pitch", "0.0",
                    "--roll", "0.0",
                    "--frame-id", "kinect_camera",
                    "--child-frame-id", "bcr_bot/base_footprint/kinect_camera"]
    )






    ld = LaunchDescription()

    ld.add_action(start_gz_bridge)

    return ld
