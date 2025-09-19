# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause


import os

from isaacsim.core.utils.extensions import get_extension_path_from_name

from isaaclab.controllers.rmp_flow import RmpFlowControllerCfg

# Note: RMP-Flow config files for supported robots are stored in the motion_generation extension
_RMP_CONFIG_DIR = os.path.join(
    get_extension_path_from_name("isaacsim.robot_motion.motion_generation"), "motion_policy_configs"
)

# Path to current directory
_CUR_DIR = os.path.dirname(os.path.realpath(__file__))

_MOTION_CONFIG_DIR = os.path.join(_CUR_DIR, "motion_policy_configs")

FRANKA_RMPFLOW_CFG = RmpFlowControllerCfg(
    config_file=os.path.join(_RMP_CONFIG_DIR, "franka", "rmpflow", "franka_rmpflow_common.yaml"),
    urdf_file=os.path.join(_CUR_DIR, "data", "lula_franka_gen.urdf"),
    collision_file=os.path.join(_RMP_CONFIG_DIR, "franka", "rmpflow", "robot_descriptor.yaml"),
    frame_name="panda_hand",
    evaluations_per_frame=5,
    ignore_robot_state_updates=True,
)
"""Configuration of RMPFlow for Franka arm (default from `isaacsim.robot_motion.motion_generation`)."""


UR10_RMPFLOW_CFG = RmpFlowControllerCfg(
    config_file=os.path.join(_RMP_CONFIG_DIR, "ur10", "rmpflow", "ur10_rmpflow_config.yaml"),
    urdf_file=os.path.join(_RMP_CONFIG_DIR, "ur10", "ur10_robot.urdf"),
    collision_file=os.path.join(_RMP_CONFIG_DIR, "ur10", "rmpflow", "ur10_robot_description.yaml"),
    frame_name="ee_link",
    evaluations_per_frame=5,
    ignore_robot_state_updates=True,
)
"""Configuration of RMPFlow for UR10 arm (default from `isaacsim.robot_motion.motion_generation`)."""

PIPER_RMPFLOW_CFG = RmpFlowControllerCfg(
    config_file=os.path.join(_MOTION_CONFIG_DIR, "piper", "rmpflow", "piper_rmpflow_config.yaml"),
    urdf_file=os.path.join(_MOTION_CONFIG_DIR, "piper", "piper_description.urdf"),
    collision_file=os.path.join(_MOTION_CONFIG_DIR, "piper", "rmpflow", "piper_robot_description.yaml"),
    frame_name="gripper_center",
    evaluations_per_frame=5,
    ignore_robot_state_updates=True,
)
"""Configuration of RMPFlow for Piper arm."""

GALBOT_LEFT_ARM_RMPFLOW_CFG = RmpFlowControllerCfg(
    config_file=os.path.join(
        _MOTION_CONFIG_DIR, "galbot_one_charlie", "rmpflow", "galbot_one_charlie_left_arm_rmpflow_config_cortex.yaml"
    ),
    urdf_file=os.path.join(_MOTION_CONFIG_DIR, "galbot_one_charlie", "galbot_one_charlie.urdf"),
    collision_file=os.path.join(
        _MOTION_CONFIG_DIR, "galbot_one_charlie", "rmpflow", "galbot_one_charlie_left_arm_gripper.yaml"
    ),
    frame_name="left_gripper_tcp_link",
    evaluations_per_frame=5,
    ignore_robot_state_updates=True,
)

GALBOT_RIGHT_ARM_RMPFLOW_CFG = RmpFlowControllerCfg(
    config_file=os.path.join(
        _MOTION_CONFIG_DIR, "galbot_one_charlie", "rmpflow", "galbot_one_charlie_right_arm_rmpflow_config_cortex.yaml"
    ),
    urdf_file=os.path.join(_MOTION_CONFIG_DIR, "galbot_one_charlie", "galbot_one_charlie.urdf"),
    collision_file=os.path.join(
        _MOTION_CONFIG_DIR, "galbot_one_charlie", "rmpflow", "galbot_one_charlie_right_arm_suction.yaml"
    ),
    frame_name="right_suction_cup_tcp_link",
    evaluations_per_frame=5,
    ignore_robot_state_updates=True,
)

"""Configuration of RMPFlow for Galbot humanoid."""

AGIBOT_LEFT_ARM_RMPFLOW_CFG = RmpFlowControllerCfg(
    config_file=os.path.join(_MOTION_CONFIG_DIR, "agibot", "rmpflow", "agibot_left_arm_rmpflow_config_cortex.yaml"),
    urdf_file=os.path.join(_MOTION_CONFIG_DIR, "agibot", "agibot.urdf"),
    collision_file=os.path.join(_MOTION_CONFIG_DIR, "agibot", "rmpflow", "agibot_left_arm_gripper.yaml"),
    frame_name="gripper_center",
    evaluations_per_frame=5,
    ignore_robot_state_updates=True,
)

AGIBOT_RIGHT_ARM_RMPFLOW_REPLAY_CFG = RmpFlowControllerCfg(
    config_file=os.path.join(_MOTION_CONFIG_DIR, "agibot", "rmpflow", "agibot_right_arm_rmpflow_config_cortex.yaml"),
    urdf_file=os.path.join(_MOTION_CONFIG_DIR, "agibot", "agibot.urdf"),
    collision_file=os.path.join(_MOTION_CONFIG_DIR, "agibot", "rmpflow", "agibot_right_arm_gripper_replay.yaml"),
    frame_name="right_gripper_center",
    evaluations_per_frame=5,
    ignore_robot_state_updates=True,
)

AGIBOT_RIGHT_ARM_BODY_RMPFLOW_REPLAY_CFG = RmpFlowControllerCfg(
    config_file=os.path.join(
        _MOTION_CONFIG_DIR, "agibot", "rmpflow", "agibot_right_arm_body_rmpflow_config_cortex.yaml"
    ),
    urdf_file=os.path.join(_MOTION_CONFIG_DIR, "agibot", "agibot.urdf"),
    collision_file=os.path.join(_MOTION_CONFIG_DIR, "agibot", "rmpflow", "agibot_right_arm_body_replay.yaml"),
    frame_name="right_gripper_center",
    evaluations_per_frame=5,
    ignore_robot_state_updates=True,
)

AGIBOT_RIGHT_ARM_RMPFLOW_TELEOP_CFG = RmpFlowControllerCfg(
    config_file=os.path.join(_MOTION_CONFIG_DIR, "agibot", "rmpflow", "agibot_right_arm_rmpflow_config_cortex.yaml"),
    urdf_file=os.path.join(_MOTION_CONFIG_DIR, "agibot", "agibot.urdf"),
    collision_file=os.path.join(_MOTION_CONFIG_DIR, "agibot", "rmpflow", "agibot_right_arm_gripper_teleop.yaml"),
    frame_name="right_gripper_center",
    evaluations_per_frame=5,
    ignore_robot_state_updates=True,
)

"""Configuration of RMPFlow for Agibot humanoid."""
