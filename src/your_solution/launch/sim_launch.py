from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_solution',
            executable='tf_broadcaster',
            name='tf_broadcaster',
            parameters=[
                {'use_sim_time': True}
            ],
            output='screen'
        ),
        Node(
            package='your_solution',
            executable='calc_error',
            name='calc_error',
            parameters=[
                {'use_sim_time': True}
            ],
            output='screen'
        ),
        Node(
            package='your_solution',
            executable='calc_error',
            name='calc_error',
            parameters=[
                {'use_sim_time': True}
            ],
            output='screen'
        ),
        Node(
            package='huskybot_cv',
            executable='huskybot_cv',
            name='huskybot_cv',
            parameters=[
                {'use_sim_time': True}
            ],
            output='screen'
        ),
        Node(
            package='sim_node',
            executable='keyboard_controls.py',
            name='keyboard_controls',
            output='screen'
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('foxglove_bridge'),
                    'launch',
                    'foxglove_bridge_launch.xml'
                ])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('sim_node'),
                    'launch',
                    'sim_node_launch.py'
                ])
            ),
            launch_arguments={
                'cv_exposure': '0.8',
                'cpu_sim': 'true',
            }.items()
        )
    ])