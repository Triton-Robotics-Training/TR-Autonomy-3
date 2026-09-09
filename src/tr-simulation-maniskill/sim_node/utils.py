from typing import Dict

import numpy as np
import sapien.physx as physx
import torch

from mani_skill.sensors.base_sensor import BaseSensor, BaseSensorConfig
from mani_skill.sensors.camera import Camera
from mani_skill.utils import common

from tr_messages.msg import RobotGroundTruth, SimGroundTruth
from geometry_msgs.msg import Pose, Twist


class robot_state:
    def __init__(
        self,
        pitch: float = 0,
        yaw: float = 0,
        x_vel: float = 0,
        y_vel: float = 0,
        angular_vel: float = 0,
    ) -> None:
        self.pitch = pitch
        self.yaw = yaw
        self.x_vel = x_vel
        self.y_vel = y_vel
        self.angular_vel = angular_vel

    def __eq__(self, value: object) -> bool:
        return (
            self.pitch == value.pitch
            and self.yaw == value.yaw
            and self.x_vel == value.x_vel
            and self.y_vel == value.y_vel
            and self.angular_vel == value.angular_vel
        )

    def __str__(self) -> str:
        return (
            f"{self.x_vel}, {self.y_vel}, {self.angular_vel}, {self.pitch}, {self.yaw}"
        )


def sensor_data_to_pointcloud(observation: Dict):
    """convert all camera data in sensor to pointcloud data"""
    sensor_data = observation["sensor_data"]
    camera_params = observation["sensor_param"]
    pointcloud_obs = dict()

    for cam_uid, images in sensor_data.items():
        if "lidar" in cam_uid:
            cam_pcd = {}
            # TODO: double check if the .clone()s are necessary
            # Each pixel is (x, y, z, actor_id) in OpenGL camera space
            # actor_id = 0 for the background
            images: Dict[str, torch.Tensor]
            position = images["position"].clone()
            segmentation = images["segmentation"].clone()
            position = position.float()
            position[..., :3] = (
                position[..., :3] / 1000.0
            )  # convert the raw depth from millimeters to meters

            # Convert to world space
            cam2world = camera_params[cam_uid]["cam2world_gl"].to(position.device)
            xyzw = torch.cat([position, segmentation != 0], dim=-1).reshape(
                position.shape[0], -1, 4
            ) @ cam2world.transpose(1, 2)
            cam_pcd["xyzw"] = xyzw

            # Extra keys
            if "rgb" in images:
                rgb = images["rgb"][..., :3].clone()
                cam_pcd["rgb"] = rgb.reshape(rgb.shape[0], -1, 3)
            if "segmentation" in images:
                cam_pcd["segmentation"] = segmentation.reshape(
                    segmentation.shape[0], -1, 1
                )

            pointcloud_obs[cam_uid] = cam_pcd

    pointcloud_obs = common.merge_dicts(pointcloud_obs.values())
    for key, value in pointcloud_obs.items():
        pointcloud_obs[key] = torch.concat(value, axis=1)

    return pointcloud_obs


# obs should be the dict of the specific robot from extra obs that this msg belongs to
# ex: obs = obs["extra"]["primary_robot"]
def populate_robot_ground_truth_msg(msg: RobotGroundTruth, obs: dict) -> None:
    populate_pose_msg_from_batched_pose_tensor(msg.chassis_pose, obs["chassis_pose"])
    populate_pose_msg_from_batched_pose_tensor(msg.turret_pose, obs["turret_pose"])
    populate_pose_msg_from_batched_pose_tensor(msg.camera_pose, obs["camera_pose"])
    populate_pose_msg_from_batched_pose_tensor(msg.lidar_pose, obs["lidar_pose"])

    msg.armor_panel_poses = []
    # for panel in obs["panel_poses"]:
    for i in range(4):
        p = Pose()
        # send batched i-th panel
        populate_pose_msg_from_batched_pose_tensor(p, obs["panel_poses"][:, i])
        msg.armor_panel_poses.append(p)


def populate_pose_msg_from_batched_pose_tensor(
    msg: Pose, poses_tensor: torch.Tensor
) -> None:
    # for now we just get data from the 1st env for ros
    msg.position.x = poses_tensor[0][0].item()
    msg.position.y = poses_tensor[0][1].item()
    msg.position.z = poses_tensor[0][2].item()

    msg.orientation.w = poses_tensor[0][3].item()
    msg.orientation.x = poses_tensor[0][4].item()
    msg.orientation.y = poses_tensor[0][5].item()
    msg.orientation.z = poses_tensor[0][6].item()


def populate_twist_msg_from_batched_velocity_tensors(
    msg: Twist,
    linear_velocity: torch.Tensor,
    angular_velocity: torch.Tensor,
) -> None:
    # Velocities from SAPIEN are in world frame (shape [N, 3]).
    # We publish them as-is; the Odometry child_frame_id must be set to "map"
    # so that the expressed frame matches per the nav_msgs/Odometry convention.
    msg.linear.x = linear_velocity[0][0].item()
    msg.linear.y = linear_velocity[0][1].item()
    msg.linear.z = linear_velocity[0][2].item()

    msg.angular.x = angular_velocity[0][0].item()
    msg.angular.y = angular_velocity[0][1].item()
    msg.angular.z = angular_velocity[0][2].item()
