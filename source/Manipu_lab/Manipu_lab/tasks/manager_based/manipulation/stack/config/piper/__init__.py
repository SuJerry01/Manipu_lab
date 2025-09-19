# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

##
# Register Gym environments.
##

gym.register(
    id="Isaac-Stack-Cube-Piper-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.stack_joint_pos_env_cfg:PiperCubeStackEnvCfg",
    },
    disable_env_checker=True,
)


gym.register(
    id="Isaac-Stack-Cube-Piper-IK-Rel-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.stack_ik_rel_env_cfg:PiperCubeStackEnvCfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Stack-Cube-Piper-RmpFlow-Rel-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.stack_ik_rel_env_cfg:RmpFlowPiperCubeStackEnvCfg",
    },
    disable_env_checker=True,
)
