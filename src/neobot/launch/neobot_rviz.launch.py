#!/usr/bin/python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Set the path to different files and folders.
  pkg_share = FindPackageShare(package='neobot').find('neobot')
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz_config.rviz')
  
  # Launch configuration variables specific to simulation
  rviz_config_file = LaunchConfiguration('rviz_config_file')



  # Declare the launch arguments  
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')
    
   

  # Launch RViz
  start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])



  ld = LaunchDescription()

  ld.add_action(declare_rviz_config_file_cmd)

  # Add any actions
  ld.add_action(start_rviz_cmd)

  return ld
