import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sim_launch = os.path.join(
        get_package_share_directory('sim_node'), 'launch', 'sim_node_launch.py')
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'cpu_sim': 'true',
                'human_gui': 'false',
                'cv_exposure': '0.8',
                'cv_resolution_x': '1280',
                'cv_resolution_y': '720',
                'enable_lidar': 'false',
            }.items(),
        ),
        Node(package='sim_node', executable='keyboard_controls.py', output='screen'),
        Node(package='huskybot_cv', executable='huskybot_cv',
             parameters=[sim_time], output='screen'),
        Node(package='your_solution', executable='tf_broadcaster',
             parameters=[sim_time], output='screen'),
        Node(package='your_solution', executable='calc_error',
             parameters=[sim_time], output='screen'),
    ])