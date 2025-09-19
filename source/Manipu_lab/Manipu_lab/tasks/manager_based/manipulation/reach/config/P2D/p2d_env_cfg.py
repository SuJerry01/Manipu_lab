# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Taehyeong Kim

# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

import Manipu_lab.tasks.manager_based.manipulation.reach.mdp as mdp
from Manipu_lab.tasks.manager_based.manipulation.reach.dual_reach_env_cfg import ReachEnvCfg
from isaaclab.managers import SceneEntityCfg

from Manipu_lab.assets.P2D import P2D_CFG  # isort: skip


@configclass
class P2DReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Assign robot asset
        self.scene.robot = P2D_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Joint reset configuration
        self.events.reset_robot_joints.params["position_range"] = (0.0, 0.0)

        # Reward configuration: set correct end-effector body
        ee_link_l = "gripper_l_tip"
        self.rewards.end_effector_position_tracking_left.params["asset_cfg"].body_names = [ee_link_l]
        self.rewards.end_effector_position_tracking_fine_grained_left.params["asset_cfg"].body_names = [ee_link_l]
        self.rewards.end_effector_orientation_tracking_left.params["asset_cfg"].body_names = [ee_link_l]

        ee_link_r = "gripper_r_tip"
        self.rewards.end_effector_position_tracking_right.params["asset_cfg"].body_names = [ee_link_r]
        self.rewards.end_effector_position_tracking_fine_grained_right.params["asset_cfg"].body_names = [ee_link_r]
        self.rewards.end_effector_orientation_tracking_right.params["asset_cfg"].body_names = [ee_link_r]

        # Action configuration (base, arm_l, arm_r)
        self.actions.base_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["base_joint"],
            scale=0.3,
        )
        self.actions.arm_l_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["arm_l_joint[1-6]"],
            scale=0.4,
        )
        self.actions.arm_r_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["arm_r_joint[1-6]"],
            scale=0.4,
        )
        
        # Observation policy configuration
        self.observations.policy.joint_pos.params["asset_cfg"] = SceneEntityCfg(
            name="robot", joint_names=["arm_l_joint[1-6]", "arm_r_joint[1-6]", "base_joint"]
        )
        self.observations.policy.joint_vel.params["asset_cfg"] = SceneEntityCfg(
            name="robot", joint_names=["arm_l_joint[1-6]", "arm_r_joint[1-6]", "base_joint"]
        )

        self.commands.ee_pose_l.body_name = ee_link_l
        self.commands.ee_pose_r.body_name = ee_link_r


@configclass
class P2DReachEnvCfg_PLAY(P2DReachEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False





