from mani_skill.envs.scene import ManiSkillScene
from mani_skill.utils.structs.pose import Pose
import sapien
import numpy as np
from mani_skill.agents.base_agent import BaseAgent, Keyframe
from mani_skill.agents.controllers import *
from mani_skill.agents.registration import register_agent
from mani_skill.sensors.camera import CameraConfig
from torch import Tensor
from ament_index_python import get_package_share_directory
import os
from sim_node import constants
from mani_skill.utils.geometry.rotation_conversions import (
    euler_angles_to_matrix,
    matrix_to_quaternion,
)
import torch

package_dir = get_package_share_directory("sim_node")

armor_panel_gltf_path_dict = {
    "infantry": package_dir
    + "/resource/models/individual_armor_panels/infantry_armor_panel.gltf",
    "hero": package_dir
    + "/resource/models/individual_armor_panels/hero_armor_panel.gltf",
    "sentry": package_dir
    + "/resource/models/individual_armor_panels/sentry_armor_panel.gltf",
}

# this hardcodes the robot panel
# current_robot = "infantry"


@register_agent()
class EllipseRobot(BaseAgent):
    uid = "ellipse_robot"

    def __init__(
        self,
        robot_name: str,
        scene: ManiSkillScene,
        control_freq: int,
        control_mode: str | None = None,
        agent_idx: str | None = None,
        initial_pose: Pose | Pose | None = None,
        build_separate: bool = False,
        major_axis: float = 0.4,
        minor_axis: float = 0.2,
        keyframe: str = None,
    ):

        self.keyframe = keyframe

        self.major_axis = major_axis
        self.minor_axis = minor_axis

        self.current_robot = robot_name

        super().__init__(
            scene, control_freq, control_mode, agent_idx, initial_pose, build_separate
        )

    default_pos = sapien.Pose(p=[0, 0, 0.25], q=[1, 0, 0, 0])
    default_pos.set_rpy([0, 0, np.deg2rad(45)])

    aiming_target_pos = sapien.Pose(p=[2.5, -1.85, 0.25], q=[1, 0, 0, 0])

    # distinct spawn poses so the third/fourth robots don't overlap the
    # secondary aiming target (or each other) on reset
    third_target_pos = sapien.Pose(p=[1.85, -2.5, 0.25], q=[1, 0, 0, 0])

    fourth_target_pos = sapien.Pose(p=[0.0, -1.8, 0.25], q=[1, 0, 0, 0])

    # Parked well clear of the field, for scenarios that want the arena empty.
    # The root link is fixed, so it stays put rather than falling off the floor.
    out_of_field_pos = sapien.Pose(p=[20.0, 20.0, 0.25], q=[1, 0, 0, 0])

    keyframes = dict(
        default=Keyframe(pose=default_pos),
        aiming_target=Keyframe(pose=aiming_target_pos),
        third_target=Keyframe(pose=third_target_pos),
        fourth_target=Keyframe(pose=fourth_target_pos),
        out_of_field=Keyframe(pose=out_of_field_pos),
    )

    base_joint_names = [
        f"chassis_x_axis",
        f"chassis_y_axis",
        f"chassis_revolute",
    ]
    # Dummy joints — accepted in action space to match infantry robot interface but physically inert
    yaw_joint_names = ["yaw"]
    pitch_joint_names = [f"pitch"]

    def reset(self, init_qpos: Tensor = None):
        super().reset(init_qpos)

        keyframe_str = self.keyframe if self.keyframe is not None else "default"
        keyframe = self.keyframes[keyframe_str]

        self.robot.set_pose(keyframe.pose)

    def _after_init(self):
        super()._after_init()

    def _load_articulation(self, initial_pose=None):
        name = self.uid

        builder: sapien.ArticulationBuilder = self.scene.create_articulation_builder()

        base = builder.create_link_builder()
        base.set_name(f"{name}_base")

        x_slide = builder.create_link_builder(parent=base)
        x_slide.set_name(f"{name}_x_slide")
        x_slide.set_joint_name(f"chassis_x_axis")
        x_slide.set_joint_properties(
            type="prismatic",
            limits=[[-np.inf, np.inf]],
            pose_in_parent=sapien.Pose(),
            pose_in_child=sapien.Pose(),
        )

        y_slide = builder.create_link_builder(parent=x_slide)
        y_slide.set_name(f"{name}_y_slide")
        y_slide.set_joint_name(f"chassis_y_axis")
        y_slide.set_joint_properties(
            type="prismatic",
            limits=[[-np.inf, np.inf]],
            pose_in_parent=sapien.Pose(
                q=[0.7071068, 0, 0, 0.7071068]
            ),  # joint X → world Y
            pose_in_child=sapien.Pose(q=[0.7071068, 0, 0, 0.7071068]),
        )

        center = builder.create_link_builder(parent=y_slide)
        center.set_name(f"{name}_center")
        center.set_joint_name(f"chassis_revolute")
        center.set_joint_properties(
            type="revolute",
            limits=[[-np.inf, np.inf]],
            pose_in_parent=sapien.Pose(
                q=[0.7071068, 0, -0.7071068, 0]
            ),  # joint X → world +Z (CCW), matching PDBaseVelController's R_z(+ori)
            pose_in_child=sapien.Pose(q=[0.7071068, 0, -0.7071068, 0]),
            friction=0.0,
            damping=0.1,
        )
        center_marker = sapien.render.RenderMaterial()
        center_marker.set_base_color([0.3, 0.3, 0.3, 1])
        center.add_box_visual(
            half_size=[self.major_axis - 0.02, self.minor_axis - 0.02, 0.05],
            material=center_marker,
        )

        for i, theta in enumerate([0, np.pi / 2, np.pi, 3 * np.pi / 2]):
            x = self.major_axis * np.cos(theta)
            y = self.minor_axis * np.sin(theta)

            frame_q = matrix_to_quaternion(
                euler_angles_to_matrix(
                    torch.tensor(
                        [
                            [
                                np.deg2rad(15) * np.sin(theta),
                                -np.deg2rad(15) * np.cos(theta),
                                theta,
                            ]
                        ],
                        dtype=torch.float32,
                    ),
                    convention="XYZ",
                )
            )[0].numpy()

            panel_link = builder.create_link_builder(parent=center)
            panel_link.set_name(f"{name}_panel_{i}")
            panel_link.set_joint_name(f"panel_fixed_{i}")
            panel_link.set_joint_properties(
                type="fixed",
                limits=[],
                pose_in_parent=sapien.Pose(p=[x, y, 0], q=frame_q),
                pose_in_child=sapien.Pose(),
            )

            visual_q = matrix_to_quaternion(
                euler_angles_to_matrix(
                    torch.tensor([[0, np.pi / 2, np.pi / 2]], dtype=torch.float32),
                    convention="XYZ",
                )
            )[0].numpy()
            panel_link.add_visual_from_file(
                filename=armor_panel_gltf_path_dict[self.current_robot],
                pose=sapien.Pose(q=visual_q),
            )

        # Dummy yaw and pitch links — physically inert (force_limit=0) but give the
        # same 5-DOF action space as InfantryRobot so callers don't need special-casing.
        yaw_dummy = builder.create_link_builder(parent=center)
        yaw_dummy.set_name(f"{name}_yaw_link")
        yaw_dummy.set_joint_name(f"yaw")
        yaw_dummy.set_joint_properties(
            type="revolute",
            limits=[[-np.inf, np.inf]],
            pose_in_parent=sapien.Pose(),
            pose_in_child=sapien.Pose(),
            friction=0.0,
            damping=0.0,
        )

        pitch_dummy = builder.create_link_builder(parent=yaw_dummy)
        pitch_dummy.set_name(f"{name}_pitch_link")
        pitch_dummy.set_joint_name(f"pitch")
        pitch_dummy.set_joint_properties(
            type="revolute",
            limits=[[-np.inf, np.inf]],
            pose_in_parent=sapien.Pose(),
            pose_in_child=sapien.Pose(),
            friction=0.0,
            damping=0.0,
        )

        pose = initial_pose if initial_pose is not None else sapien.Pose(p=[0, 0, 2])
        builder.set_initial_pose(pose)
        robot_name = (
            f"{name}-agent-{self._agent_idx}" if self._agent_idx is not None else name
        )
        self.robot = builder.build(fix_root_link=True, name=robot_name)

        for link in self.robot.links:
            if link.name == f"{name}_center" or "yaw" in link.name or "pitch" in link.name:
                continue
            if not link.render_shapes:
                continue
            parts = list(link.render_shapes[0])[0].parts
            # TODO this is hard coded blue
            if self.current_robot == "hero":
                temp = parts[1]
                parts[1] = parts[0]
                parts[0] = parts[2]
                parts[2] = temp

            parts[0].material.set_base_color([0, 0, 1, 1])  # light bars -> blue
            parts[0].material.set_emission([0, 0, 1, 500])  # light bar glow

            parts[1].material.set_base_color([1, 1, 1, 1])  # symbol     -> white
            parts[2].material.set_base_color([0, 0, 0, 1])  # main plate -> black

    @property
    def _controller_configs(self):
        base_pd_joint_vel = PDBaseVelControllerConfig(
            self.base_joint_names,
            lower=[
                -constants.MAX_TRANSLATION_VEL_M_S,
                -constants.MAX_TRANSLATION_VEL_M_S,
                -constants.MAX_ANGULAR_VEL_RADS_S,
            ],
            upper=[
                constants.MAX_TRANSLATION_VEL_M_S,
                constants.MAX_TRANSLATION_VEL_M_S,
                constants.MAX_ANGULAR_VEL_RADS_S,
            ],
            damping=1000,
            force_limit=500,
        )

        # Dummy controllers — consume yaw/pitch action dims but apply zero force,
        # keeping the action space identical to InfantryRobot.
        yaw_dummy = PDJointPosControllerConfig(
            self.yaw_joint_names,
            lower=None,
            upper=None,
            stiffness=0,
            damping=0,
            force_limit=0,
            normalize_action=False,
        )

        pitch_dummy = PDJointPosControllerConfig(
            self.pitch_joint_names,
            lower=None,
            upper=None,
            stiffness=0,
            damping=0,
            force_limit=0,
            normalize_action=False,
        )

        controller_configs = dict(
            pd_standard=dict(
                base=base_pd_joint_vel,
                yaw=yaw_dummy,
                pitch=pitch_dummy,
            )
        )

        return deepcopy_dict(controller_configs)

    def get_armor_panel_poses(self) -> torch.Tensor:
        name = self.uid
        panel_links = [self.robot.links_map[f"{name}_panel_{i}"] for i in range(4)]
        # Each link.pose.raw_pose is (b, 7); stack to (4, b, 7) then permute to (b, 4, 7)
        panels_stacked = torch.stack([link.pose.raw_pose for link in panel_links])
        return panels_stacked.permute(1, 0, 2)

    def get_ground_truth_obs(self) -> dict:
        # The ellipse robot has no turret, camera, or lidar — mock them as chassis pose
        chassis_pose: Pose = self.robot.links_map[f"{self.uid}_center"].pose
        return dict(
            chassis_pose=chassis_pose.raw_pose,
            turret_pose=chassis_pose.raw_pose,
            camera_pose=chassis_pose.raw_pose,
            lidar_pose=chassis_pose.raw_pose,
            panel_poses=self.get_armor_panel_poses(),
        )
