# Copyright (c) 2024-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.managers.recorder_manager import RecorderManagerBaseCfg, RecorderTerm, RecorderTermCfg
from isaaclab.utils import configclass

from . import recorders

##
# State recorders.
##


@configclass
class InitialStateRecorderCfg(RecorderTermCfg):
    """Configuration for the initial state recorder term."""

    class_type: type[RecorderTerm] = recorders.InitialStateRecorder


@configclass
class PostStepStatesRecorderCfg(RecorderTermCfg):
    """Configuration for the step state recorder term."""

    class_type: type[RecorderTerm] = recorders.PostStepStatesRecorder


@configclass
class PreStepActionsRecorderCfg(RecorderTermCfg):
    """Configuration for the step action recorder term."""

    class_type: type[RecorderTerm] = recorders.PreStepActionsRecorder


@configclass
class PreStepFlatPolicyObservationsRecorderCfg(RecorderTermCfg):
    """Configuration for the step policy observation recorder term."""

    class_type: type[RecorderTerm] = recorders.PreStepFlatPolicyObservationsRecorder


@configclass
class PostStepAbsEEFPoseBinaryGripperActionsRecorderCfg(RecorderTermCfg):
    """Configuration for the step absolute eef pose + binary gripper actions recorder term."""

    class_type: type[RecorderTerm] = recorders.PostStepAbsEEFPoseBinaryGripperActionsRecorder


@configclass
class PostStepAbsEEFPoseAbsGripperActionsRecorderCfg(RecorderTermCfg):
    """Configuration for the step absolute eef pose + absolute gripper actions recorder term."""

    class_type: type[RecorderTerm] = recorders.PostStepAbsEEFPoseAbsGripperActionsRecorder


##
# Recorder manager configurations.
##


@configclass
class ActionStateRecorderManagerCfg(RecorderManagerBaseCfg):
    """Recorder configurations for recording actions and states."""

    record_initial_state = InitialStateRecorderCfg()
    record_post_step_states = PostStepStatesRecorderCfg()
    record_pre_step_actions = PreStepActionsRecorderCfg()
    record_pre_step_flat_policy_observations = PreStepFlatPolicyObservationsRecorderCfg()


##
# Recorder manager configurations for Mimic workflow.
# Actions: absolute_eef_pose: (N, 7) + binary_gripper_action: (N, 1)
##


@configclass
class AbsEEFPoseBinaryGripperActionStateRecorderManagerCfg(RecorderManagerBaseCfg):
    """Recorder configurations for recording actions and states."""

    record_initial_state = InitialStateRecorderCfg()
    record_post_step_states = PostStepStatesRecorderCfg()
    record_post_step_abs_eef_pose_binary_gripper_actions = PostStepAbsEEFPoseBinaryGripperActionsRecorderCfg()
    record_pre_step_flat_policy_observations = PreStepFlatPolicyObservationsRecorderCfg()


##
# Recorder manager configurations for Mimic workflow.
# Actions: absolute_eef_pose: (N, 7) + absolute_gripper_action: (N, 1)
##
@configclass
class AbsEEFPoseAbsGripperActionStateRecorderManagerCfg(RecorderManagerBaseCfg):
    """Recorder configurations for recording actions and states."""

    record_initial_state = InitialStateRecorderCfg()
    record_post_step_states = PostStepStatesRecorderCfg()
    record_post_step_abs_eef_pose_abs_gripper_actions = PostStepAbsEEFPoseAbsGripperActionsRecorderCfg()
    record_pre_step_flat_policy_observations = PreStepFlatPolicyObservationsRecorderCfg()
