from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Overridable from the command line, e.g.
    #   ros2 launch your_solution your_solution_launch.py cpu_sim:=true
    #   ros2 launch your_solution your_solution_launch.py human_gui:=false
    cpu_sim = DeclareLaunchArgument('cpu_sim', default_value='false')
    human_gui = DeclareLaunchArgument('human_gui', default_value='true')

    # Simulator: always started through sim_node_launch.py (not `ros2 run`),
    # because that launch file sets up the environment the sim process needs.
    sim_launch_path = (
        get_package_share_directory('sim_node') + '/launch/sim_node_launch.py')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_path),
        launch_arguments={
            'human_gui': LaunchConfiguration('human_gui'),
            'cv_exposure': '0.8',
            'cpu_sim': LaunchConfiguration('cpu_sim'),
        }.items(),
    )

    use_sim_time = [{'use_sim_time': True}]

    keyboard = Node(
        package='sim_node', executable='keyboard_controls.py',
        output='screen')

    huskybot = Node(
        package='huskybot_cv', executable='huskybot_cv',
        parameters=use_sim_time, output='screen')

    tf_broadcaster = Node(
        package='your_solution', executable='tf_broadcaster',
        parameters=use_sim_time, output='screen')

    calc_error = Node(
        package='your_solution', executable='calc_error',
        parameters=use_sim_time, output='screen')

    return LaunchDescription(
        [cpu_sim, human_gui, sim, keyboard, huskybot, tf_broadcaster,
         calc_error])
