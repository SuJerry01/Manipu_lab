# Copyright (c) 2024-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from collections.abc import Sequence

from isaaclab.managers.recorder_manager import RecorderTerm


class InitialStateRecorder(RecorderTerm):
    """Recorder term that records the initial state of the environment after reset."""

    def record_post_reset(self, env_ids: Sequence[int] | None):
        def extract_env_ids_values(value):
            nonlocal env_ids
            if isinstance(value, dict):
                return {k: extract_env_ids_values(v) for k, v in value.items()}
            return value[env_ids]

        return "initial_state", extract_env_ids_values(self._env.scene.get_state(is_relative=True))


class PostStepStatesRecorder(RecorderTerm):
    """Recorder term that records the state of the environment at the end of each step."""

    def record_post_step(self):
        return "states", self._env.scene.get_state(is_relative=True)


class PostStepAbsEEFPoseBinaryGripperActionsRecorder(RecorderTerm):
    """Recorder term that records the absolute eef_pose actions of the environment at the end of each step."""

    def record_post_step(self):
        ## FIXME: record current eef_pose as action_target may bring a little bit of delay
        ## this is the eef_pose in world frame
        eef_pose = self._env.obs_buf["policy"]["eef_pose"]

        if "Suction" in self._env.unwrapped.cfg.env_name:
            gripper_bool_action = torch.tensor(
                [not cup.is_closed() for cup in self._env._suction_cup], dtype=torch.bool
            ).view(-1, 1)
            gripper_value_action = torch.where(gripper_bool_action, torch.tensor(1.0), torch.tensor(-1.0)).to(
                self._env.device
            )
            final_action = torch.cat([eef_pose, gripper_value_action], dim=-1)
            return "actions", final_action
        else:
            gripper_value_action = self._env.action_manager.action[:, -1:]
            final_action = torch.cat([eef_pose, gripper_value_action], dim=-1)
            return "actions", final_action


class PostStepAbsEEFPoseAbsGripperActionsRecorder(RecorderTerm):
    """Recorder term that records the absolute eef_pose actions of the environment at the end of each step."""

    def record_post_step(self):
        ## FIXME: record current eef_pose as action_target may bring a little bit of delay
        ## this is the eef_pose in world frame
        eef_pose = self._env.obs_buf["policy"]["eef_pose"]
        gripper_pos = self._env.obs_buf["policy"]["gripper_pos"]

        if "Suction" in self._env.unwrapped.cfg.env_name:
            gripper_bool_action = torch.tensor(
                [not cup.is_closed() for cup in self._env._suction_cup], dtype=torch.bool
            ).view(-1, 1)
            gripper_value_action = torch.where(gripper_bool_action, torch.tensor(1.0), torch.tensor(-1.0)).to(
                self._env.device
            )
            final_action = torch.cat([eef_pose, gripper_value_action], dim=-1)
            return "actions", final_action
        else:
            absolute_gripper_pos = gripper_pos[:, 0].view(-1, 1)
            final_action = torch.cat([eef_pose, absolute_gripper_pos], dim=-1)
            return "actions", final_action


class PreStepActionsRecorder(RecorderTerm):
    """Recorder term that records the actions in the beginning of each step."""

    def record_pre_step(self):
        if "Suction" in self._env.unwrapped.cfg.env_name:
            gripper_bool_action = torch.tensor(
                [not cup.is_closed() for cup in self._env._suction_cup], dtype=torch.bool
            ).view(-1, 1)
            gripper_value_action = torch.where(gripper_bool_action, torch.tensor(1.0), torch.tensor(-1.0)).to(
                self._env.device
            )
            final_action = torch.cat([self._env.action_manager.action, gripper_value_action], dim=-1)
            return "actions", final_action
        else:
            return "actions", self._env.action_manager.action


class PreStepFlatPolicyObservationsRecorder(RecorderTerm):
    """Recorder term that records the policy group observations in each step."""

    def record_pre_step(self):
        return "obs", self._env.obs_buf["policy"]
