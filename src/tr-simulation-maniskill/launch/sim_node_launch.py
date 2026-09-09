from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import os
import importlib.util


def _torch_lib_env():
    """Scope a libshm.so workaround to the sim_node process.

    torch ships its own libshm.so and so does this workspace's `shm` package.
    Sourcing install/setup.bash puts install/shm/lib on LD_LIBRARY_PATH. On
    torch builds where LD_LIBRARY_PATH takes precedence over the library's own
    search path (DT_RUNPATH rather than DT_RPATH), `import torch` then binds to
    the wrong libshm.so and fails; this was observed on an aarch64 Ubuntu VM.
    Prepending torch/lib globally would break every C++ node that links our
    shm, so it is only done here, for this one Python process. Where torch's
    rpath already wins this is a harmless no-op.
    """
    spec = importlib.util.find_spec("torch")
    if spec is None or not spec.origin:
        return {}
    lib = os.path.join(os.path.dirname(spec.origin), "lib")
    if not os.path.isfile(os.path.join(lib, "libshm.so")):
        return {}
    current = os.environ.get("LD_LIBRARY_PATH", "")
    return {"LD_LIBRARY_PATH": lib + (":" + current if current else "")}


def generate_launch_description():
    # Simulation parameters
    sim_params = [
        # General parameters
        DeclareLaunchArgument(
            "human_gui",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "spawn_scenario",
            default_value="center_1v1",
        ),
        DeclareLaunchArgument(
            "blue",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "sim_time_scale",
            default_value="1.0",
        ),
        DeclareLaunchArgument(
            "control_freq",
            default_value="150",
        ),
        DeclareLaunchArgument(
            "sim_freq",
            default_value="300",
        ),
        DeclareLaunchArgument(
            "cpu_sim",
            default_value="false",
        ),
        # One argument per opponent: the node declares robot_opponent_1..3 and
        # never reads a singular `robot_opponent`, so the old spelling silently
        # left all three at their defaults.
        DeclareLaunchArgument(
            "robot_opponent_1",
            default_value="infantry",
        ),
        DeclareLaunchArgument(
            "robot_opponent_2",
            default_value="sentry",
        ),
        DeclareLaunchArgument(
            "robot_opponent_3",
            default_value="hero",
        ),
        DeclareLaunchArgument(
            "seed",
            default_value="2930",
        ),
        # Referee state. game_state 4 is GAME_STATUS_IN_COMPETITION, which is
        # what the sentry state machine waits for before it will drive.
        DeclareLaunchArgument(
            "game_state",
            default_value="0",
        ),
        DeclareLaunchArgument(
            "robot_hp",
            default_value="400",
        ),
        # ---
        # CV Camera parameters
        DeclareLaunchArgument(
            "enable_cv_cam",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "cv_exposure",
            default_value="0.005",
        ),
        DeclareLaunchArgument(
            "cv_fov_horizontal",
            default_value="31.0",
        ),
        DeclareLaunchArgument(
            "cv_fov_vertical",
            default_value="20.0",
        ),
        DeclareLaunchArgument(
            "cv_ray_tracing",
            default_value="false",
        ),
        # ---
        # Overhead capture, for CI review media. Off by default: it is an extra
        # render pass per step. Declared here as well as in the node because
        # `ros2 launch` rejects an undeclared argument outright, so a scenario
        # passing enable_overhead_cam:=true would otherwise fail every run
        # rather than quietly getting no picture.
        DeclareLaunchArgument(
            "enable_overhead_cam",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "overhead_cam_height",
            default_value="14.0",
        ),
        DeclareLaunchArgument(
            "overhead_cam_resolution",
            default_value="480",
        ),
        DeclareLaunchArgument(
            "cv_resolution_x",
            default_value="1920",
        ),
        DeclareLaunchArgument(
            "cv_resolution_y",
            default_value="1200",
        ),
        # ---
        # LiDAR parameters
        DeclareLaunchArgument(
            "enable_lidar",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "lidar_pointcloud_resolution",
            default_value="20",
        ),
        # ---
        # ROS parameters
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),
    ]

    node = Node(
        package="sim_node",
        executable="main.py",
        parameters=[
            {
                "human_gui": LaunchConfiguration("human_gui"),
                "spawn_scenario": LaunchConfiguration("spawn_scenario"),
                "blue": LaunchConfiguration("blue"),
                "sim_time_scale": LaunchConfiguration("sim_time_scale"),
                "control_freq": LaunchConfiguration("control_freq"),
                "sim_freq": LaunchConfiguration("sim_freq"),
                "cpu_sim": LaunchConfiguration("cpu_sim"),
                "enable_cv_cam": LaunchConfiguration("enable_cv_cam"),
                "cv_exposure": LaunchConfiguration("cv_exposure"),
                "cv_fov_horizontal": LaunchConfiguration("cv_fov_horizontal"),
                "cv_fov_vertical": LaunchConfiguration("cv_fov_vertical"),
                "cv_ray_tracing": LaunchConfiguration("cv_ray_tracing"),
                "enable_overhead_cam": LaunchConfiguration("enable_overhead_cam"),
                "overhead_cam_height": LaunchConfiguration("overhead_cam_height"),
                "overhead_cam_resolution": LaunchConfiguration(
                    "overhead_cam_resolution"
                ),
                "cv_resolution_x": LaunchConfiguration("cv_resolution_x"),
                "cv_resolution_y": LaunchConfiguration("cv_resolution_y"),
                "enable_lidar": LaunchConfiguration("enable_lidar"),
                "lidar_pointcloud_resolution": LaunchConfiguration(
                    "lidar_pointcloud_resolution"
                ),
                "robot_opponent_1": LaunchConfiguration("robot_opponent_1"),
                "robot_opponent_2": LaunchConfiguration("robot_opponent_2"),
                "robot_opponent_3": LaunchConfiguration("robot_opponent_3"),
                "seed": LaunchConfiguration("seed"),
                "game_state": LaunchConfiguration("game_state"),
                "robot_hp": LaunchConfiguration("robot_hp"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
        additional_env=_torch_lib_env(),
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(sim_params + [node])
