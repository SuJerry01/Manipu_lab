# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Piper manipulator robot.

The following configurations are available:

* :obj:`PIPER_CFG`: 6‑DoF arm with a simple 2‑DoF parallel gripper

"""

from Manipu_lab.assets import ISAACLAB_ASSETS_DATA_DIR

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

# -----------------------------------------------------------------------------
# Piper configuration
# -----------------------------------------------------------------------------

PIPER_CFG = ArticulationCfg(
    # ------------------------------------------------------------------
    # Spawn parameters
    # ------------------------------------------------------------------
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/piper_description/piper_description/piper.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    # ------------------------------------------------------------------
    # Initial state
    # ------------------------------------------------------------------
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            # arm
            "joint1": 0.0,
            "joint2": 1.8,
            "joint3": -1.5,
            "joint4": 0.0,
            "joint5": 1.0,
            "joint6": 0.0,
            # gripper
            "joint7": 0.035,  # left finger (open)
            "joint8": -0.035,  # right finger (open)
        },
        pos=(0.0, 0.0, 0.0),
    ),
    # ------------------------------------------------------------------
    # Actuator groups
    # ------------------------------------------------------------------
    actuators={
        # 6-DoF arm with joint-specific gains
        "piper_arm": ImplicitActuatorCfg(
            joint_names_expr=["joint[1-6]"],
            effort_limit_sim=100.0,
            velocity_limit_sim=5.0,
            stiffness={
                "joint1": 70.0,
                "joint2": 90.0,
                "joint3": 110.0,
                "joint4": 90.0,
                "joint5": 80.0,
                "joint6": 60.0,
            },
            damping={
                "joint1": 30.0,
                "joint2": 30.0,
                "joint3": 50.0,
                "joint4": 45.0,
                "joint5": 45.0,
                "joint6": 45.0,
            },
        ),
        # Gripper stays as one group
        "piper_gripper": ImplicitActuatorCfg(
            joint_names_expr=["joint[7-8]"],
            effort_limit_sim=80.0,
            velocity_limit_sim=1.0,
            stiffness=1200,
            damping=450,
            friction=1.5,
        ),
    },
    # Use the full joint limits specified in USD / URDF
    soft_joint_pos_limit_factor=1.0,
)

PIPER_HIGH_PD_CFG = PIPER_CFG.copy()
PIPER_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
PIPER_HIGH_PD_CFG.actuators["piper_arm"].stiffness = 400.0
PIPER_HIGH_PD_CFG.actuators["piper_arm"].damping = 100.0


"""Default configuration for the Piper 6‑DoF manipulator with 2‑finger gripper."""
