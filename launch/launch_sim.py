from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        # Node(
        #     package='sim_node',
        #     executable='main',
        #     name='general',
        # ),
        Node(
            package='sim_node',
            executable='keyboard_controls.py',
            name='keyboard',
        ),
        Node(
            package='huskybot_cv',
            executable='huskybot_cv',
            name='huskybot',
            ros_arguments=['--ros-args'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='your_solution',
            executable='tf_broadcaster',
            name='tf_broadcaster',
            ros_arguments=['--ros-args'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='your_solution',
            executable='calc_error',
            name='calc_error',
            parameters=[{'use_sim_time': True}]
        ),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sim_node'),
                'launch',
                'sim_node_launch.py',  # Match the exact filename from ls above
            )
        ),
        launch_arguments={
            'human_gui': 'false',
            'cv_exposure': '0.8',
            }.items(),
    )
    ])
