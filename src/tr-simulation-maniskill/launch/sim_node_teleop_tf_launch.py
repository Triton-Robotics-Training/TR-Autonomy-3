from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # TODO: Remove this file (redundant to teleop launch)
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("robot_opponent_1", default_value="infantry"))
    ld.add_action(DeclareLaunchArgument("robot_opponent_2", default_value="sentry"))
    ld.add_action(DeclareLaunchArgument("robot_opponent_3", default_value="hero"))
    ld.add_action(DeclareLaunchArgument("blue", default_value="true"))

    sim_launch_path = (
        get_package_share_directory("sim_node") + "/launch/sim_node_launch.py"
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path=sim_launch_path),
        launch_arguments={
            "robot_opponent_1": LaunchConfiguration("robot_opponent_1"),
            "robot_opponent_2": LaunchConfiguration("robot_opponent_2"),
            "robot_opponent_3": LaunchConfiguration("robot_opponent_3"),
            "blue": LaunchConfiguration("blue"),
        }.items(),
    )

    tf_helper_node = Node(
        package="sim_node", executable="tf_tree_helper.py", output="screen"
    )
    keyboard_controls_node = Node(
        package="sim_node", executable="keyboard_controls.py", output="screen"
    )
    shm_forwarder = Node(
        package="sim_node", executable="zero_copy_forwarder", output="screen"
    )

    ld.add_action(sim_launch)
    ld.add_action(keyboard_controls_node)
    ld.add_action(tf_helper_node)
    ld.add_action(shm_forwarder)

    return ld
