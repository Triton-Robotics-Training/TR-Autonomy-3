import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    human_gui = LaunchConfiguration('human_gui')
    cv_exposure = LaunchConfiguration('cv_exposure')
    foxglove = LaunchConfiguration('foxglove')

    human_gui_arg = DeclareLaunchArgument('human_gui', default_value='false')
    cv_exposure_arg = DeclareLaunchArgument('cv_exposure', default_value='0.8')
    foxglove_arg = DeclareLaunchArgument('foxglove', default_value='false')

    sim_launch_file = os.path.join(
        get_package_share_directory('sim_node'), 'launch', 'sim_node_launch.py')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_file),
        launch_arguments={
            'human_gui': human_gui,
            'cv_exposure': cv_exposure,
        }.items(),
    )

    keyboard = Node(
        package='sim_node',
        executable='keyboard_controls.py',
        output='screen',
    )

    huskybot = Node(
        package='huskybot_cv',
        executable='huskybot_cv',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    tf_broadcaster = Node(
        package='your_solution',
        executable='tf_broadcaster',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    calc_error = Node(
        package='your_solution',
        executable='calc_error',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(foxglove),
    )

    return LaunchDescription([
        human_gui_arg,
        cv_exposure_arg,
        foxglove_arg,
        sim,
        keyboard,
        huskybot,
        tf_broadcaster,
        calc_error,
        foxglove_bridge,
    ])
