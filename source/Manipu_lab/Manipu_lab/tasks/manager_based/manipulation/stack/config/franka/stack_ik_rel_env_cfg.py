# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.controllers.config.rmp_flow import FRANKA_RMPFLOW_CFG
from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg

from Manipu_lab.tasks.manager_based.manipulation.stack.mdp.actions.rmpflow_actions_cfg import RMPFlowActionCfg
#from isaaclab.envs.mdp.actions.rmpflow_actions_cfg import RMPFlowActionCfg

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.utils import configclass

from Manipu_lab.tasks.manager_based.manipulation.stack import mdp
from Manipu_lab.tasks.manager_based.manipulation.stack.mdp import franka_stack_events

from . import stack_joint_pos_env_cfg

##
# Pre-defined configs
##
from Manipu_lab.assets.franka import (  # isort: skip
    FRANKA_PANDA_HIGH_PD_CFG,
    FRANKA_ROBOTIQ_GRIPPER_CFG,
)


@configclass
class EventCfgFrankaRobotiqGripper:
    """Configuration for events."""

    init_franka_arm_pose = EventTerm(
        func=franka_stack_events.set_default_joint_pose,
        mode="reset",
        params={
            # ["left_inner_knuckle_joint", "right_inner_knuckle_joint"] are excluded from closed-loop articulation
            # 'finger_joint', 'right_outer_knuckle_joint', 'left_outer_finger_joint', 'right_outer_finger_joint', 'left_inner_finger_joint', 'right_inner_finger_joint', 'left_inner_finger_knuckle_joint', 'right_inner_finger_knuckle_joint']
            # "default_pose": [0.0444, -0.1894, -0.1107, -2.5148, 0.0044, 2.3775, 0.6952,
            #                  0.785, 0.785, 0.0, 0.0, -0.785, -0.785, 0.0, 0.0],
            "default_pose": [
                0.0,
                0.0,
                0.0,
                -1.57,
                0.0,
                1.57,
                0.0,  # home_pose from customer
                0.0,
                0.0,
                0.0,
                0.0,
                -0.785,
                -0.785,
                0.0,
                0.0,
            ],
        },
    )

    randomize_franka_joint_state = EventTerm(
        func=franka_stack_events.randomize_joint_by_gaussian_offset,
        mode="reset",
        params={
            "mean": 0.0,
            "std": 0.02,
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )

    randomize_cube_positions = EventTerm(
        func=franka_stack_events.randomize_object_pose,
        mode="reset",
        params={
            "pose_range": {"x": (0.4, 0.6), "y": (-0.10, 0.10), "z": (0.0203, 0.0203), "yaw": (-1.0, 1, 0)},
            "min_separation": 0.1,
            "asset_cfgs": [SceneEntityCfg("cube_1"), SceneEntityCfg("cube_2"), SceneEntityCfg("cube_3")],
        },
    )


class FrankaRobotiqGripperCubeStackEnvCfg(stack_joint_pos_env_cfg.FrankaCubeStackEnvCfg):

    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # Set events
        self.events = EventCfgFrankaRobotiqGripper()

        # Set Franka as robot
        self.scene.robot = FRANKA_PANDA_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for the specific robot type (franka)
        self.actions.arm_action = mdp.DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["panda_joint.*"],
            body_name="panda_hand",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls"),
            scale=1.0,
            body_offset=mdp.DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.145]),
        )

        """
        You need to specify all the active joint position targets for both close and open cmd.
        Not including mimic_joints, you do not need to give cmd_target for mimic_joints.
            - "right_outer_knuckle_joint" is the mimic_joint for "finger_joint"
            - left_inner_finger_joint": -0.785, "right_inner_finger_joint": 0.785, position_target is set as official USD recommended
        """
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["finger_joint", ".*_inner_finger_joint", ".*_inner_finger_knuckle_joint", ".*_outer_.*_joint"],
            close_command_expr={
                "finger_joint": 0.785,
                "left_inner_finger_joint": -0.785,
                "right_inner_finger_joint": 0.785,
            },
            open_command_expr={
                "finger_joint": 0.0,
                "left_inner_finger_joint": -0.785,
                "right_inner_finger_joint": 0.785,
            },
        )

        self.gripper_joint_names = ["finger_joint", "right_outer_knuckle_joint"]
        self.gripper_open_val = 0.0
        self.gripper_threshold = 0.1

        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/panda_link0",
            debug_vis=True,
            visualizer_cfg=self.marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/panda_hand",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=[0.0, 0.0, 0.145],
                    ),
                ),
            ],
        )

        #

        # Set the simulation parameters
        self.sim.dt = 1 / 60
        self.sim.render_interval = 1

        self.decimation = 3
        self.episode_length_s = 20.0


@configclass
class FrankaCubeStackEnvCfg(stack_joint_pos_env_cfg.FrankaCubeStackEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set Franka as robot
        # We switch here to a stiffer PD controller for IK tracking to be better.
        self.scene.robot = FRANKA_PANDA_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for the specific robot type (franka)
        self.actions.arm_action = mdp.DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["panda_joint.*"],
            body_name="panda_hand",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls"),
            scale=0.5,
            body_offset=mdp.DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.107]),
        )

        # Set the simulation parameters
        self.sim.dt = 1 / 60
        self.sim.render_interval = 1

        self.decimation = 3
        self.episode_length_s = 20.0


@configclass
class RmpFlowFrankaCubeStackEnvCfg(stack_joint_pos_env_cfg.FrankaCubeStackEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set Franka as robot
        # We switch here to a stiffer PD controller for IK tracking to be better.
        self.scene.robot = FRANKA_PANDA_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        self.actions.arm_action = RMPFlowActionCfg(
            asset_name="robot",
            joint_names=["panda_joint.*"],
            body_name="panda_hand",
            controller=FRANKA_RMPFLOW_CFG,
            scale=1.0,
            body_offset=mdp.DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.0]),
            articulation_prim_expr="/World/envs/env_.*/Robot",
        )

        # Set the simulation parameters
        self.sim.dt = 1 / 60
        self.sim.render_interval = 1

        self.decimation = 3
        self.episode_length_s = 20.0
