import time
import rclpy
from rclpy.node import Node
from tr_messages.srv import WriteSerial, ListenSerial, LidarOdometry
from tr_messages.msg import (
    SimTeleopInput,
    RobotGroundTruth,
    SimGroundTruth,
    WriteChassis,
    WriteGimbal,
    EmbeddedRobotState,
    RefData,
)
from sensor_msgs.msg import Image
from sim_node import simulation

from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2  # pointcloud utilizes
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Pose
from nav_msgs.msg import Odometry
from sim_node import utils, constants
import numpy as np

from cv_bridge import CvBridge
import torch
from torch import Tensor

from sim_node.simulation import SPAWN_SCENARIO_KEYFRAME_MAPPING

from mani_skill.utils.structs import SimConfig
from sim_node.ARC2026_env import ARC2026EnvConfig


class Sim_Node(Node):
    def __init__(self):
        super().__init__("sim_node")

        # cv cam params
        self.declare_parameter("enable_cv_cam", True)
        self.declare_parameter("cv_resolution_x", 1920)
        self.declare_parameter("cv_resolution_y", 1200)
        self.declare_parameter("cv_fov_horizontal", 31.0)
        self.declare_parameter("cv_fov_vertical", 20.0)
        self.declare_parameter("cv_exposure", 0.005)
        self.declare_parameter("cv_ray_tracing", False)
        # TODO add ray tracing config values as ros params
        # TODO add cv camera matrix parameter

        # lidar params
        self.declare_parameter("enable_lidar", True)
        self.declare_parameter("lidar_pointcloud_resolution", 20)

        # general simulation params
        self.declare_parameter("spawn_scenario", "center_1v1")
        self.declare_parameter("human_gui", True)
        self.declare_parameter("control_freq", 150)
        self.declare_parameter("sim_freq", 300)
        self.declare_parameter("sim_time_scale", 1.0)
        self.declare_parameter("game_state", 0)
        self.declare_parameter("robot_hp", 400)
        self.declare_parameter("cpu_sim", False)
        self.declare_parameter("robot_opponent_1", "infantry")
        self.declare_parameter("robot_opponent_2", "sentry")
        self.declare_parameter("robot_opponent_3", "hero")
        self.declare_parameter("blue", True)

        # self.declare_parameter("parallel_in_single_scene", True)
        # self.declare_parameter("num_envs", 2)

        self.primary_robot_teleop_sub = self.create_subscription(
            SimTeleopInput,
            "simulation/primary_robot_teleop",
            self.primary_robot_teleop_callback,
            10,
        )
        self.secondary_robot_teleop_sub = self.create_subscription(
            SimTeleopInput,
            "simulation/secondary_robot_teleop",
            self.secondary_robot_teleop_callback,
            10,
        )
        self.third_robot_teleop_sub = self.create_subscription(
            SimTeleopInput,
            "simulation/third_robot_teleop",
            self.third_robot_teleop_callback,
            10,
        )
        self.fourth_robot_teleop_sub = self.create_subscription(
            SimTeleopInput,
            "simulation/fourth_robot_teleop",
            self.fourth_robot_teleop_callback,
            10,
        )

        self.write_chassis_sub = self.create_subscription(
            WriteChassis,
            "write_chassis",
            self.write_chassis_callback,
            rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT),
        )

        self.write_gimbal_sub = self.create_subscription(
            WriteGimbal,
            "write_gimbal",
            self.write_gimbal_callback,
            rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT),
        )

        # TODO migrate away from these services across the whole stack
        self.write_service = self.create_service(
            WriteSerial, "write_robot_state", self.write_robot_state
        )
        self.listen_service = self.create_service(
            ListenSerial, "read_robot_state", self.read_robot_state
        )
        self.lidar_odometry_service = self.create_service(
            LidarOdometry, "lidar_odometry", self.lidar_odometry_service
        )
        self.lidar_pose_queue = []

        self.ground_truth_pub = self.create_publisher(
            SimGroundTruth, "simulation/ground_truth", 10
        )
        self.pointcloud_pub = self.create_publisher(PointCloud2, "pointcloud", 10)
        qos_profile = rclpy.qos.qos_profile_sensor_data
        qos_profile.depth = 1
        self.image_pub = self.create_publisher(Image, "camera/image", qos_profile)
        self.robot_state_pub = self.create_publisher(
            EmbeddedRobotState, "/embedded_robot_state", qos_profile=qos_profile
        )
        self.lidar_pose_pub = self.create_publisher(
            Odometry, "/odin1/odometry_highfreq", 10
        )
        self.ref_data_pub = self.create_publisher(RefData, "/ref_data", 10)

        control_freq = self.gp("control_freq").integer_value
        sim_time_scale = self.gp("sim_time_scale").double_value
        # TODO make sure this is set correctly
        self.simulation_timer = self.create_timer(
            1 / (control_freq * sim_time_scale), self.simulation_callback
        )

        self.ros_clock_pub = self.create_publisher(
            msg_type=Clock, topic="/clock", qos_profile=10
        )
        self.clock_msg = Clock()

        sim_config = SimConfig()
        sim_config.control_freq = self.gp("control_freq").integer_value
        sim_config.sim_freq = self.gp("sim_freq").integer_value
        sim_config.spacing = 15

        spawn_scenario = (
            self.get_parameter("spawn_scenario").get_parameter_value().string_value
        )
        robot_keyframes = [
            SPAWN_SCENARIO_KEYFRAME_MAPPING[spawn_scenario]["primary_robot"],
            SPAWN_SCENARIO_KEYFRAME_MAPPING[spawn_scenario]["secondary_robot"],
            SPAWN_SCENARIO_KEYFRAME_MAPPING[spawn_scenario]["third_robot"],
            SPAWN_SCENARIO_KEYFRAME_MAPPING[spawn_scenario]["fourth_robot"],
        ]
        self.blue = self.get_parameter("blue").value
        robot_keyframes[0] = "blue_spawn" if self.blue else "red_spawn"
        # fmt: off
        arc2026_env_config = ARC2026EnvConfig(
            robot_keyframes=robot_keyframes,
            enable_cv_cam=                  self.gp("enable_cv_cam").bool_value,
            cv_exposure=                    self.gp("cv_exposure").double_value,
            cv_ray_tracing=                 self.gp("cv_ray_tracing").bool_value,
            cv_resolution_x=                self.gp("cv_resolution_x").integer_value,
            cv_resolution_y=                self.gp("cv_resolution_y").integer_value,
            cv_fov_horizontal=              self.gp("cv_fov_horizontal").double_value,
            cv_fov_vertical=                self.gp("cv_fov_vertical").double_value,
            enable_lidar=                   self.gp("enable_lidar").bool_value,
            lidar_pointcloud_resolution=    self.gp("lidar_pointcloud_resolution").integer_value,
            robot_1=                          self.gp("robot_opponent_1").string_value,
            robot_2=                          self.gp("robot_opponent_2").string_value,
            robot_3=                          self.gp("robot_opponent_3").string_value,
        )
        # fmt: on

        self.simulation = simulation.Simulation(
            seed=2930,
            cpu_sim=self.gp("cpu_sim").bool_value,
            human_gui=self.gp("human_gui").bool_value,
            sim_config=sim_config,
            arc2026_env_config=arc2026_env_config,
        )

        self.control_mode = "programmatic"
        self.programmatic_desired_robot_state = utils.robot_state()
        self.teleop_desired_robot_state = utils.robot_state()
        self.secondary_robot_teleop_desired_state = utils.robot_state()
        self.third_robot_teleop_desired_state = utils.robot_state()
        self.fourth_robot_teleop_desired_state = utils.robot_state()
        self.last_recorded_robot_state = utils.robot_state()

        self.cv_bridge = CvBridge()

    def simulation_callback(self):
        start = time.perf_counter()
        sim_ground_truth_msg = SimGroundTruth()
        sim_ground_truth_msg.header.stamp = self.clock_msg.clock
        sim_ground_truth_msg.header.frame_id = "map"

        primary_robot_state = None
        if self.control_mode == "programmatic":
            primary_robot_state = self.programmatic_desired_robot_state
        elif self.control_mode == "teleop":
            primary_robot_state = self.teleop_desired_robot_state
        else:
            RuntimeError(f"invalid control_mode. {self.control_mode}")

        t1 = time.perf_counter()
        obs = self.simulation.step(
            primary_robot_state=primary_robot_state,
            secondary_robot_state=self.secondary_robot_teleop_desired_state,
            third_robot_state=self.third_robot_teleop_desired_state,
            fourth_robot_state=self.fourth_robot_teleop_desired_state,
        )
        t2 = time.perf_counter()
        sim_ground_truth_msg.step_sim = (t2 - t1) * 1000

        robot_state_position: Tensor = obs["agent"]["infantry-0"]["qpos"]

        # TODO FIX ME THIS IS INCORRECT (i forgot why it was incorrect lol, maybe coordinate frame?)
        robot_state_velocity: Tensor = obs["agent"]["infantry-0"]["qvel"]

        self.last_recorded_robot_state = utils.robot_state(
            x_vel=robot_state_velocity[0][0].item(),
            y_vel=robot_state_velocity[0][1].item(),
            angular_vel=robot_state_velocity[0][2].item(),
            yaw=robot_state_position[0][3].item(),
            pitch=robot_state_position[0][4].item(),
        )

        if self.get_parameter("enable_cv_cam").get_parameter_value().bool_value:
            t1 = time.perf_counter()
            rgb_tensor = obs["sensor_data"]["cv_camera_0"]["rgb"]
            rgb_tensor: torch.Tensor
            rgb_array = rgb_tensor[0].numpy(force=True)

            if rgb_array.shape[0] == 3:  # If first dimension is 3, it's (C, H, W)
                rgb_array = rgb_array.transpose(1, 2, 0)  # Convert to (H, W, C)

            # Ensure uint8 type for cv_bridge
            rgb_array = rgb_array.astype("uint8")

            img_msg = self.cv_bridge.cv2_to_imgmsg(rgb_array, encoding="rgb8")
            img_msg.header.stamp = self.clock_msg.clock
            t2 = time.perf_counter()
            sim_ground_truth_msg.cv_process_img = (t2 - t1) * 1000

            t1 = time.perf_counter()
            self.image_pub.publish(img_msg)
            t2 = time.perf_counter()
            sim_ground_truth_msg.cv_pub = (t2 - t1) * 1000

        if self.get_parameter("enable_lidar").get_parameter_value().bool_value:
            t1 = time.perf_counter()
            pointcloud = utils.sensor_data_to_pointcloud(obs)

            xyzw = pointcloud["xyzw"]
            # xyzw = xyzw.squeeze(0)
            xyzw = xyzw[0]
            valid_mask = xyzw[:, 3] == 1
            points = xyzw[valid_mask, :3]
            msg = self.points_to_ros_pointcloud2(points)

            self.pointcloud_pub.publish(msg)
            t2 = time.perf_counter()
            sim_ground_truth_msg.pointcloud = (t2 - t1) * 1000

        # sync ros to sim time
        sim_timestamp = obs["extra"]["sim_timestamp"]
        self.clock_msg.clock.sec = int(sim_timestamp)
        # nanosec only contains the fractional part of the timestamp
        self.clock_msg.clock.nanosec = int((sim_timestamp % 1) * 1e9)
        self.ros_clock_pub.publish(self.clock_msg)

        primary_robot_msg = RobotGroundTruth()
        utils.populate_robot_ground_truth_msg(
            msg=primary_robot_msg,
            obs=obs["extra"]["primary_robot"],
        )
        secondary_robot_msg = RobotGroundTruth()
        utils.populate_robot_ground_truth_msg(
            msg=secondary_robot_msg,
            obs=obs["extra"]["secondary_robot"],
        )
        third_robot_msg = RobotGroundTruth()
        utils.populate_robot_ground_truth_msg(
            msg=third_robot_msg,
            obs=obs["extra"]["third_robot"],
        )
        fourth_robot_msg = RobotGroundTruth()
        utils.populate_robot_ground_truth_msg(
            msg=fourth_robot_msg,
            obs=obs["extra"]["fourth_robot"],
        )
        sim_ground_truth_msg.primary_robot = primary_robot_msg
        sim_ground_truth_msg.secondary_robot = secondary_robot_msg
        sim_ground_truth_msg.third_robot = third_robot_msg
        sim_ground_truth_msg.fourth_robot = fourth_robot_msg

        end = time.perf_counter()
        sim_ground_truth_msg.total_time = (end - start) * 1000
        sim_ground_truth_msg.theoretical_fps = 1 / (end - start)
        self.ground_truth_pub.publish(sim_ground_truth_msg)

        # Publish embedded robot state topic
        robot_state_msg = EmbeddedRobotState()
        robot_state_msg.header.stamp = sim_ground_truth_msg.header.stamp
        robot_state_msg.yaw = robot_state_position[0][3].item()
        robot_state_msg.pitch = robot_state_position[0][4].item()
        # TODO fix this apparently robot_state_velocity is broken??
        robot_state_msg.pitch_vel = 0.0
        robot_state_msg.yaw_vel = 0.0
        self.robot_state_pub.publish(robot_state_msg)

        # Publish lidar pose as odometry
        # pose is in "map" frame; twist is also expressed in "map" (world) frame
        lidar_odom_msg = Odometry()
        lidar_odom_msg.header.stamp = sim_ground_truth_msg.header.stamp
        lidar_odom_msg.header.frame_id = "map"
        lidar_odom_msg.child_frame_id = "map"
        utils.populate_pose_msg_from_batched_pose_tensor(
            lidar_odom_msg.pose.pose, obs["extra"]["primary_robot"]["lidar_pose"]
        )
        utils.populate_twist_msg_from_batched_velocity_tensors(
            lidar_odom_msg.twist.twist,
            obs["extra"]["primary_robot"]["lidar_linear_velocity"],
            obs["extra"]["primary_robot"]["lidar_angular_velocity"],
        )
        self.lidar_pose_pub.publish(lidar_odom_msg)

        # TODO dont make this hard coded
        ref_msg = RefData()
        ref_msg.header.stamp = sim_ground_truth_msg.header.stamp
        ref_msg.team_color = ref_msg.COLOR_BLUE if self.blue else ref_msg.COLOR_RED
        ref_msg.game_state = self.get_parameter("game_state").value
        ref_msg.robot_hp = self.get_parameter("robot_hp").value

        self.ref_data_pub.publish(ref_msg)

        # lidar odometry service
        lidar_odometry_pos = Pose()
        utils.populate_pose_msg_from_batched_pose_tensor(
            lidar_odometry_pos, obs["extra"]["primary_robot"]["lidar_pose"]
        )
        self.lidar_pose_queue.append((sim_timestamp, lidar_odometry_pos))
        # remove elements older than 100ms
        while self.lidar_pose_queue[0][0] < sim_timestamp - 0.1:
            self.lidar_pose_queue.pop(0)

    def read_robot_state(self, request, response):
        # TODO make a buffer and respond based off the time in the request
        # TOOD add angular vel and pitch and yaw vel
        response.x_vel = self.last_recorded_robot_state.x_vel
        response.y_vel = self.last_recorded_robot_state.y_vel
        response.pitch = -self.last_recorded_robot_state.pitch
        response.yaw = self.last_recorded_robot_state.yaw
        response.pitch_vel = 0.0
        response.yaw_vel = 0.0
        response.success = True

        return response

    def write_robot_state(self, request, response):
        self.control_mode = "programmatic"
        self.programmatic_desired_robot_state = utils.robot_state(
            # pitch is negated so negative pitch means down
            -request.pitch,
            request.yaw,
            # we scale by max because maniskill normalizes velocity action space between -1,1
            request.x_vel / constants.MAX_TRANSLATION_VEL_M_S,
            request.y_vel / constants.MAX_TRANSLATION_VEL_M_S,
            request.angular_vel / constants.MAX_ANGULAR_VEL_RADS_S,
        )

        response.success = True
        return response

    def write_gimbal_callback(self, msg: WriteGimbal):
        self.control_mode = "programmatic"
        self.programmatic_desired_robot_state.pitch = -msg.pitch
        self.programmatic_desired_robot_state.yaw = msg.yaw

    # TODO this is monkey tier janky please fix this at some point
    def lidar_odometry_service(self, request, response):
        request_time = request.request_time.sec + request.request_time.nanosec / 1e9
        # curr_time = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] / 1e9
        # self.get_logger().info(f"requested {request_time} | curr: {curr_time}")

        closest = self.lidar_pose_queue[0]
        for i in self.lidar_pose_queue:
            if abs(i[0] - request_time) < abs(closest[0] - request_time):
                closest = i

        response.pose = closest[1]
        response.success = True
        return response

    # TODO this is jank need to rewrite the whole programmatic vs teleop system
    def write_chassis_callback(self, msg):
        msg: WriteChassis
        self.control_mode = "programmatic"
        self.programmatic_desired_robot_state.x_vel = msg.x_vel
        self.programmatic_desired_robot_state.y_vel = msg.y_vel
        self.programmatic_desired_robot_state.angular_vel = msg.angular_vel

    # TODO refactor the whole teleop vs "programmatic" input stuff
    # everything should publish to the same topic (WriteChassis and WriteGimbal)
    # we can have a different topic for "debug" simulation controls such as controlling 2-3 non primary robots
    def primary_robot_teleop_callback(self, msg):
        received_state = utils.robot_state(
            # pitch is negated so negative pitch means down
            pitch=-msg.pitch,
            yaw=msg.yaw,
            x_vel=msg.x_vel / constants.MAX_TRANSLATION_VEL_M_S,
            y_vel=msg.y_vel / constants.MAX_TRANSLATION_VEL_M_S,
            angular_vel=msg.angular_vel / constants.MAX_ANGULAR_VEL_RADS_S,
        )

        if received_state != self.teleop_desired_robot_state:
            self.control_mode = "teleop"
            self.teleop_desired_robot_state = received_state

    def secondary_robot_teleop_callback(self, msg):
        self.secondary_robot_teleop_desired_state = utils.robot_state(
            pitch=0,
            yaw=0,
            x_vel=msg.x_vel / constants.MAX_TRANSLATION_VEL_M_S,
            y_vel=msg.y_vel / constants.MAX_TRANSLATION_VEL_M_S,
            angular_vel=msg.angular_vel / constants.MAX_ANGULAR_VEL_RADS_S,
        )

    def third_robot_teleop_callback(self, msg):
        self.third_robot_teleop_desired_state = utils.robot_state(
            pitch=0,
            yaw=0,
            x_vel=msg.x_vel / constants.MAX_TRANSLATION_VEL_M_S,
            y_vel=msg.y_vel / constants.MAX_TRANSLATION_VEL_M_S,
            angular_vel=msg.angular_vel / constants.MAX_ANGULAR_VEL_RADS_S,
        )

    def fourth_robot_teleop_callback(self, msg):
        self.fourth_robot_teleop_desired_state = utils.robot_state(
            pitch=0,
            yaw=0,
            x_vel=msg.x_vel / constants.MAX_TRANSLATION_VEL_M_S,
            y_vel=msg.y_vel / constants.MAX_TRANSLATION_VEL_M_S,
            angular_vel=msg.angular_vel / constants.MAX_ANGULAR_VEL_RADS_S,
        )

    def points_to_ros_pointcloud2(self, points):

        header = Header()
        # TODO FIX ME (i think this is saying use sim time?)
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        field_x = PointField()
        field_x.name = "x"
        field_x.offset = 0
        field_x.datatype = PointField.FLOAT32
        field_x.count = 1

        field_y = PointField()
        field_y.name = "y"
        field_y.offset = 4
        field_y.datatype = PointField.FLOAT32
        field_y.count = 1

        field_z = PointField()
        field_z.name = "z"
        field_z.offset = 8
        field_z.datatype = PointField.FLOAT32
        field_z.count = 1

        fields = [field_x, field_y, field_z]
        point_iter = points.cpu().numpy()

        pc2_msg = point_cloud2.create_cloud(header, fields, point_iter)
        return pc2_msg

    def gp(self, ros_param: str):
        return self.get_parameter(ros_param).get_parameter_value()
