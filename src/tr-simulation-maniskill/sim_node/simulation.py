import gymnasium as gym
import mani_skill.envs
from mani_skill.envs.sapien_env import BaseEnv
from mani_skill.agents.base_agent import BaseAgent
from mani_skill.agents.multi_agent import MultiAgent
from sim_node import ARC2026_env
import numpy as np
from sim_node import utils
from mani_skill.utils.structs import SimConfig
import torch


# TODO: rename this to something like task could then also use this to determine different environments
SPAWN_SCENARIO_KEYFRAME_MAPPING: dict = dict(
    center_1v1=dict(
        primary_robot="default",
        secondary_robot="aiming_target",
        third_robot="third_target",
        fourth_robot="fourth_target",
    ),
    # Ego robot turned to face the aiming target, and the two robots this
    # scenario does not use parked off the field so they stay out of frame.
    cv_training=dict(
        primary_robot="facing_aiming_target",
        secondary_robot="aiming_target",
        third_robot="out_of_field",
        fourth_robot="out_of_field",
    ),
    navigate_from_spawn=dict(
        primary_robot="blue_spawn",
        secondary_robot="out_of_field",
        third_robot="third_target",
        fourth_robot="fourth_target",
    ),
)


class Simulation:

    def __init__(
        self,
        seed,
        cpu_sim,
        human_gui,
        sim_config: SimConfig,
        arc2026_env_config: ARC2026_env.ARC2026EnvConfig,
    ):
        self.human_gui_enabled = human_gui

        # arc2026 env specific config
        self.arc2026_env_config = arc2026_env_config

        self.env: BaseEnv = gym.make(
            "ARC2026",
            render_mode=("human" if self.human_gui_enabled else None),
            reward_mode="sparse",
            obs_mode="state_dict+rgb+segmentation+position",
            sim_config=sim_config,
            sim_backend="physx_cpu" if cpu_sim else "auto",
            render_backend="sapien_cpu" if cpu_sim else "sapien_cuda",
            # num_envs=2,
            # parallel_in_single_scene=True,
            arc2026_env_config=arc2026_env_config,
        )

        self.base_env: BaseEnv = self.env
        # TODO dont make this none make it the initial reset obs instead
        self.past_obs = None

        obs, _ = self.env.reset(seed=seed)

    def step(
        self,
        primary_robot_state: utils.robot_state,
        secondary_robot_state: utils.robot_state,
        third_robot_state: utils.robot_state,
        fourth_robot_state: utils.robot_state,
    ):
        action = {
            "infantry-0": self.build_robot_action("infantry-0", primary_robot_state),
            "ellipse_robot-1": self.build_robot_action(
                "ellipse_robot-1", secondary_robot_state
            ),
            "ellipse_robot-2": self.build_robot_action(
                "ellipse_robot-2", third_robot_state
            ),
            "ellipse_robot-3": self.build_robot_action(
                "ellipse_robot-3", fourth_robot_state
            ),
        }

        obs, reward, terminated, truncated, info = self.env.step(action=action)
        if self.human_gui_enabled:
            self.env.render()

        self.past_obs = obs
        return obs

    def build_robot_action(self, robot: str, robot_state: utils.robot_state):
        # calculate world relative robot velocity
        if self.past_obs is not None:
            world_relative_vel = self.head_to_world_vel(
                robot,
                robot_state.x_vel,
                robot_state.y_vel,
            )
        else:
            world_relative_vel = np.array([0, 0])

        robot_action = torch.tensor(
            [
                world_relative_vel[0],
                world_relative_vel[1],
                robot_state.angular_vel,
                robot_state.yaw,
                robot_state.pitch,
            ],
            dtype=torch.float32,
        )

        return robot_action.unsqueeze(0).expand(self.env.unwrapped.num_envs, -1)

    def shutdown(self):
        self.env.close()

    def head_to_world_vel(self, robot: str, x_vel, y_vel):
        yaw_rads = self.past_obs["agent"][robot]["qpos"][0][3].item()
        rotation_matrix = np.array(
            [
                [np.cos(yaw_rads), -np.sin(yaw_rads)],
                [np.sin(yaw_rads), np.cos(yaw_rads)],
            ]
        )

        head_relative_vel = np.array([x_vel, y_vel])
        world_relative_vel = rotation_matrix @ head_relative_vel

        return world_relative_vel
