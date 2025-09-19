# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import os

##
# Pre-defined configs
##
from isaaclab.controllers.config.rmp_flow import PIPER_RMPFLOW_CFG
from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
#from isaaclab.envs.mdp.actions.rmpflow_actions_cfg import RMPFlowActionCfg
from Manipu_lab.tasks.manager_based.manipulation.stack.mdp.actions.rmpflow_actions_cfg import RMPFlowActionCfg

from isaaclab.utils import configclass

from .stack_joint_pos_env_cfg import PiperCubeStackEnvCfg as JointPosPiperCubeStackEnvCfg

from Manipu_lab.assets.piper import PIPER_HIGH_PD_CFG  # isort: skip


@configclass
class PiperCubeStackEnvCfg(JointPosPiperCubeStackEnvCfg):
    """Stack-3-cubes task driven by the 6-DoF Piper arm with Differential Inverse Kinematics task-space control."""

    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        self.scene.robot = PIPER_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        use_relative_mode_env = os.getenv("USE_RELATIVE_MODE", "False")
        self.use_relative_mode = use_relative_mode_env.lower() in ["true", "1", "t"]

        # Set task-space actions for the specific robot type (piper)
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["joint[1-6]"],
            body_name="gripper_base",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls"),
            scale=1.0,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.1358]),
        )


@configclass
class RmpFlowPiperCubeStackEnvCfg(PiperCubeStackEnvCfg):
    """Stack-3-cubes task driven by the 6-DoF Piper arm with RmpFlow task-space control."""

    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        self.actions.arm_action = RMPFlowActionCfg(
            asset_name="robot",
            joint_names=["joint[1-6]"],
            body_name="gripper_base",
            controller=PIPER_RMPFLOW_CFG,
            scale=1.0,
            body_offset=RMPFlowActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.1358]),
            articulation_prim_expr="/World/envs/env_.*/Robot",
            use_relative_mode=self.use_relative_mode,
        )
