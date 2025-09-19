# Copyright (c) 2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Pink IK controller implementation for IsaacLab.

This module provides integration between Pink inverse kinematics solver and IsaacLab.
Pink is a differentiable inverse kinematics solver framework that provides task-space control capabilities.
"""

import numpy as np
import torch

from pink import solve_ik
from pink.configuration import Configuration
from pinocchio.robot_wrapper import RobotWrapper

from .pink_ik_cfg import PinkIKControllerCfg


class PinkIKController:
    """Integration of Pink IK controller with Isaac Lab.

    The Pink IK controller is available at: https://github.com/stephane-caron/pink
    """

    def __init__(self, cfg: PinkIKControllerCfg, device: str):
        """Initialize the Pink IK Controller.

        Args:
            cfg: The configuration for the controller.
            device: The device to use for computations (e.g., 'cuda:0').
        """
        # Initialize the robot model from URDF and mesh files
        self.robot_wrapper = RobotWrapper.BuildFromURDF(cfg.urdf_path, cfg.mesh_path, root_joint=None)
        self.pink_configuration = Configuration(
            self.robot_wrapper.model, self.robot_wrapper.data, self.robot_wrapper.q0
        )

        # Set the default targets for each task from the configuration
        for task in cfg.variable_input_tasks:
            task.set_target_from_configuration(self.pink_configuration)
        for task in cfg.fixed_input_tasks:
            task.set_target_from_configuration(self.pink_configuration)

        # Map joint names from Isaac Lab to Pink's joint conventions
        pink_joint_names = self.robot_wrapper.model.names.tolist()[1:]  # Skip the root and universal joints
        isaac_lab_joint_names = cfg.joint_names

        # Create reordering arrays for joint indices
        self.isaac_lab_to_pink_ordering = [isaac_lab_joint_names.index(pink_joint) for pink_joint in pink_joint_names]
        self.pink_to_isaac_lab_ordering = [
            pink_joint_names.index(isaac_lab_joint) for isaac_lab_joint in isaac_lab_joint_names
        ]

        self.cfg = cfg
        self.device = device

    """
    Operations.
    """

    def reorder_array(self, input_array: list[float], reordering_array: list[int]) -> list[float]:
        """Reorder the input array based on the provided ordering.

        Args:
            input_array: The array to reorder.
            reordering_array: The indices to use for reordering.

        Returns:
            Reordered array.
        """
        return [input_array[i] for i in reordering_array]

    def initialize(self):
        """Initialize the internals of the controller.

        This method is called during setup but before the first compute call.
        """
        pass

    def check_joint_limits(
        self,
        target_joint_pos: torch.Tensor,
    ) -> torch.Tensor:
        target_joint_pos = np.array(target_joint_pos)

        lower_limits = self.robot_wrapper.model.lowerPositionLimit
        upper_limits = self.robot_wrapper.model.upperPositionLimit

        # ----------- Check whether the joint position has exceedded the limit -----------
        over_lower = target_joint_pos < lower_limits
        over_upper = target_joint_pos > upper_limits
        has_over_limit = over_lower.any() or over_upper.any()

        if has_over_limit and self.cfg.show_ik_warnings:
            print("\033[93m[Warning] target_joint_pos exceeds joint limits!\033[0m")
            """
            if over_lower.any():
                print("\033[93m[Warning] Below lower limits at indices:\033[0m",
                      list(np.where(over_lower)[0]),
                      [self.isaaclab_joint_names[i] for i in np.where(over_lower)[0]])
            if over_upper.any():
                print("\033[93m[Warning] Above upper limits at indices:\033[0m",
                      list(np.where(over_upper)[0]),
                      [self.isaaclab_joint_names[i] for i in np.where(over_upper)[0]])
            """

        clamped_joint_pos = np.clip(target_joint_pos, lower_limits, upper_limits)
        return clamped_joint_pos

    def compute(
        self,
        curr_joint_pos: np.ndarray,
        dt: float,
    ) -> torch.Tensor:
        """Compute the target joint positions based on current state and tasks.

        Args:
            curr_joint_pos: The current joint positions.
            dt: The time step for computing joint position changes.

        Returns:
            The target joint positions as a tensor.
        """
        # Initialize joint positions for Pink, including the root and universal joints
        if "g1_29dof_rev_1_0_with_inspire" in self.robot_wrapper.model.name:
            # Check and clamp the joint limits for Unitree G1,
            curr_joint_positions_pink = np.array(self.reorder_array(curr_joint_pos, self.isaac_lab_to_pink_ordering))
            joint_positions_pink = self.check_joint_limits(curr_joint_positions_pink)
        else:
            # Fourier
            joint_positions_pink = np.array(self.reorder_array(curr_joint_pos, self.isaac_lab_to_pink_ordering))

        # Update Pink's robot configuration with the current joint positions
        self.pink_configuration.update(joint_positions_pink)

        # pink.solve_ik can raise an exception if the solver fails
        try:
            velocity = solve_ik(
                self.pink_configuration, self.cfg.variable_input_tasks + self.cfg.fixed_input_tasks, dt, solver="osqp"
            )
            Delta_q = velocity * dt
        except (AssertionError, Exception):
            # Print warning and return the current joint positions as the target
            # Not using omni.log since its not available in CI during docs build
            if self.cfg.show_ik_warnings:
                print(
                    "Warning: IK quadratic solver could not find a solution! Did not update the target joint positions."
                )
            return torch.tensor(curr_joint_pos, device=self.device, dtype=torch.float32)

        # Discard the first 6 values (for root and universal joints)
        pink_joint_angle_changes = Delta_q

        # Reorder the joint angle changes back to Isaac Lab conventions
        joint_vel_isaac_lab = torch.tensor(
            self.reorder_array(pink_joint_angle_changes, self.pink_to_isaac_lab_ordering),
            device=self.device,
            dtype=torch.float,
        )

        # Add the velocity changes to the current joint positions to get the target joint positions
        target_joint_pos = torch.add(
            joint_vel_isaac_lab, torch.tensor(curr_joint_pos, device=self.device, dtype=torch.float32)
        )

        return target_joint_pos
