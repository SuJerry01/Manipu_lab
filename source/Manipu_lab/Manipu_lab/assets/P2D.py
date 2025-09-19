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

from isaaclab.sim import UsdFileCfg, RigidBodyPropertiesCfg, ArticulationRootPropertiesCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg

from Manipu_lab.assets import ISAACLAB_ASSETS_DATA_DIR

P2D_CFG = ArticulationCfg(
    spawn=UsdFileCfg(
        usd_path=f"{ISAACLAB_ASSETS_DATA_DIR}/robots/P2D/P2D.usd",
        rigid_props=RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=2,
        ),
        activate_contact_sensors=True,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            # Base joints
            "base_joint": 0.0,
            
            # Left arm joints
            **{f"arm_l_joint{i+1}": 0.0 for i in range(6)},
            # Right arm joints
            **{f"arm_r_joint{i+1}": 0.0 for i in range(6)},

            # Left and right gripper joints
            **{f"gripper_l_joint{i+1}": 0.0 for i in range(2)},
            **{f"gripper_r_joint{i+1}": 0.0 for i in range(2)},

            # Head joints
            **{f"head_joint{i+1}": 0.0 for i in range(2)},
        },
    ),
    actuators={
        # Actuator for base joint
        "base": ImplicitActuatorCfg(
            joint_names_expr=["base_joint"],
            velocity_limit_sim=2.0,
            effort_limit_sim=500.0,
            stiffness=5000.0,
            damping=500.0,
        ),

        # Actuators for both arms
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                "arm_l_joint[1-6]",
                "arm_r_joint[1-6]",
            ],
            velocity_limit_sim=8.0,
            effort_limit_sim=800.0,
            stiffness=300.0,
            damping=60.0,
        ),

        # Actuators for grippers
        "grippers": ImplicitActuatorCfg(
            joint_names_expr=[
                "gripper_l_joint[1-2]",
                "gripper_r_joint[1-2]",
            ],
            velocity_limit_sim=4.0,
            effort_limit_sim=4000.0,
            stiffness=80000.0,
            damping=80.0,
        ),

        # Actuators for head joints
        "head": ImplicitActuatorCfg(
            joint_names_expr=["head_joint1", "head_joint2"],
            velocity_limit_sim=4.0,
            effort_limit_sim=250.0,
            stiffness=150.0,
            damping=40.0,
        ),
    }
)

