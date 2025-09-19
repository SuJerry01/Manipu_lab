# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause


from isaaclab.assets import RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from Manipu_lab.tasks.manager_based.manipulation.stack import mdp
from Manipu_lab.tasks.manager_based.manipulation.stack.mdp import franka_stack_events
from Manipu_lab.tasks.manager_based.manipulation.stack.stack_env_cfg import ObservationsCfg, StackEnvCfg

# ------------------------------------------------------------
#  Pre-defined configs
# ------------------------------------------------------------
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from Manipu_lab.assets.piper import PIPER_CFG  # isort: skip


@configclass
class EventCfgPiper:
    """Configuration for events."""

    init_piper_arm_pose = EventTerm(
        func=franka_stack_events.set_default_joint_pose,
        mode="reset",
        params={
            "default_pose": [
                0.0,
                1.8,
                -1.5,
                0.0,
                1.0,
                0.0,
                0.035,
                -0.035,
            ]
        },
    )

    randomize_piper_joint_state = EventTerm(
        func=franka_stack_events.randomize_galbot_joint_by_gaussian_offset,
        mode="reset",  # for reset init_pose of robot
        params={
            "mean": 0.0,
            "std": (
                0.0
            ),  # No error is tolerant for Rmpflow close-loop Rel ik control, otherwise no teleop control will result in robot motion
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )

    randomize_cube_positions = EventTerm(
        func=franka_stack_events.randomize_object_pose,
        mode="reset",
        params={
            "pose_range": {"x": (0.2, 0.4), "y": (-0.10, 0.10), "z": (0.0203, 0.0203), "yaw": (-1.0, 1.0, 0.0)},
            "min_separation": 0.1,
            "asset_cfgs": [SceneEntityCfg("cube_1"), SceneEntityCfg("cube_2"), SceneEntityCfg("cube_3")],
        },
    )


@configclass
class ObservationPiperStackEnvCfg:

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group with state values."""

        actions = ObsTerm(func=mdp.last_action)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)

        object = ObsTerm(func=mdp.object_obs)
        cube_positions = ObsTerm(func=mdp.cube_positions_in_world_frame)
        cube_orientations = ObsTerm(func=mdp.cube_orientations_in_world_frame)
        # get eef_pose(N, 7) in robot base frame
        eef_pose = ObsTerm(func=mdp.ee_frame_pos)
        gripper_pos = ObsTerm(
            func=mdp.gripper_pos,
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class SubtaskCfg(ObservationsCfg.SubtaskCfg):
        """Observations for subtask group."""

        grasp_1 = ObsTerm(
            func=mdp.object_grasped,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "ee_frame_cfg": SceneEntityCfg("ee_frame"),
                "object_cfg": SceneEntityCfg("cube_2"),
            },
        )
        stack_1 = ObsTerm(
            func=mdp.object_stacked,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "upper_object_cfg": SceneEntityCfg("cube_2"),
                "lower_object_cfg": SceneEntityCfg("cube_1"),
            },
        )
        grasp_2 = ObsTerm(
            func=mdp.object_grasped,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "ee_frame_cfg": SceneEntityCfg("ee_frame"),
                "object_cfg": SceneEntityCfg("cube_3"),
            },
        )

        def __post_init__(self):
            super().__post_init__()

    subtask_terms: SubtaskCfg = SubtaskCfg()
    policy: PolicyCfg = PolicyCfg()


# ======================================================================
#  Main training config
# ======================================================================


@configclass
class PiperCubeStackEnvCfg(StackEnvCfg):
    """Stack-3-cubes task driven by the 6-DoF Piper arm + 2-finger gripper."""

    cube_scale = (1.0, 1.0, 1.0)
    cube_properties = RigidBodyPropertiesCfg(
        solver_position_iteration_count=16,
        solver_velocity_iteration_count=1,
        max_angular_velocity=1000.0,
        max_linear_velocity=1000.0,
        max_depenetration_velocity=5.0,
        disable_gravity=False,
    )
    # ---------------  EE frame --------------- #
    marker_cfg = FRAME_MARKER_CFG.copy()
    marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
    marker_cfg.prim_path = "/Visuals/FrameTransformer"

    def __post_init__(self):
        super().__post_init__()

        # ---------------  Events --------------- #
        self.events = EventCfgPiper()
        self.observations = ObservationPiperStackEnvCfg()

        # ---------------  Robot --------------- #
        self.scene.robot = PIPER_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # arm: joints 1-6  ·  gripper: prismatic joints 7 & 8
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["joint[1-6]"],
            scale=1,
            use_default_offset=True,
        )
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["joint[7-8]"],
            # match open/close limits from PIPER_CFG.init_state
            open_command_expr={"joint7": 0.035, "joint8": -0.035},
            close_command_expr={"joint7": 0.0, "joint8": 0.0},
        )
        self.gripper_joint_names = ["joint[7-8]"]
        self.gripper_open_val = 0.035
        self.gripper_threshold = 0.005

        # Set each stacking cube deterministically
        self.scene.cube_1 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cube_1",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.4, 0.0, 0.0203], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/blue_block.usd",
                scale=self.cube_scale,
                rigid_props=self.cube_properties,
            ),
        )
        self.scene.cube_2 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cube_2",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.55, 0.05, 0.0203], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/red_block.usd",
                scale=self.cube_scale,
                rigid_props=self.cube_properties,
            ),
        )
        self.scene.cube_3 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cube_3",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.60, -0.1, 0.0203], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/green_block.usd",
                scale=self.cube_scale,
                rigid_props=self.cube_properties,
            ),
        )

        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base_link",
            debug_vis=True,
            visualizer_cfg=self.marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/gripper_base",
                    name="end_effector",
                    offset=OffsetCfg(pos=[0.0, 0.0, 0.1358]),
                ),
            ],
        )
