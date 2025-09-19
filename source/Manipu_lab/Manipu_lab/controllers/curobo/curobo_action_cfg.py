# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""CuRobo action configurations for Isaac Lab."""

from __future__ import annotations

import torch
from dataclasses import MISSING
from typing import Any, Dict, List, Optional

from isaaclab.managers.action_manager import ActionTerm, ActionTermCfg
from isaaclab.utils import configclass
from isaaclab.assets import Articulation
from isaaclab.envs import ManagerBasedEnv

from .curobo_controller import CuRoboController, CuRoboControllerConfig


@configclass
class CuRoboMotionPlanningActionCfg(ActionTermCfg):
    """CuRobo运动规划Action配置类
    
    这个配置类定义了如何使用CuRobo进行GPU加速的运动规划控制。
    """
    
    class_type: type[ActionTerm] = None  # 将在下面设置
    
    # === 基础配置 ===
    joint_names: list[str] = MISSING
    """控制的关节名称列表，例如: ["joint[1-6]"]"""
    
    body_name: str = MISSING
    """末端执行器body名称，例如: "gripper_base" """
    
    # === CuRobo配置文件路径 ===
    robot_config_path: str = MISSING
    """CuRobo机器人配置文件路径，例如: "path/to/piper_curobo.yaml" """
    
    world_config_path: str | None = None
    """世界环境配置文件路径，用于碰撞检测"""
    
    # === CuRobo性能参数 ===
    num_seeds: int = 32
    """并行规划种子数，更多种子 = 更高成功率但更慢"""
    
    max_attempts: int = 3
    """最大规划尝试次数"""
    
    planning_time_budget: float = 0.01
    """规划时间预算(秒)，默认10ms"""
    
    interpolation_dt: float = 0.02
    """轨迹插值时间步长"""
    
    # === 功能开关 ===
    enable_collision_avoidance: bool = True
    """是否启用碰撞避免"""
    
    enable_graph_cache: bool = True
    """是否启用图缓存加速"""
    
    enable_optimization: bool = True
    """是否启用轨迹优化"""
    
    fallback_to_ik: bool = True
    """规划失败时是否回退到简单IK"""
    
    # === 动作空间配置 ===
    use_quaternion: bool = True
    """是否使用四元数表示旋转(7维)，False则使用欧拉角(6维)"""
    
    scale: float = 1.0
    """动作缩放因子"""


class CuRoboMotionPlanningAction(ActionTerm):
    """CuRobo运动规划Action实现类
    
    这个类实现了CuRobo GPU加速运动规划在Isaac Lab中的集成。
    将用户的目标位姿通过CuRobo规划成smooth的轨迹。
    """
    
    cfg: CuRoboMotionPlanningActionCfg
    
    def __init__(self, cfg: CuRoboMotionPlanningActionCfg, env: ManagerBasedEnv):
        """初始化CuRobo Motion Planning Action
        
        Args:
            cfg: CuRobo配置
            env: Isaac Lab环境
        """
        super().__init__(cfg, env)
        
        # 获取机器人资产
        self._robot: Articulation = env.scene[cfg.asset_name]
        
        # 获取关节索引
        self._joint_ids, self._joint_names = self._robot.find_joints(cfg.joint_names)
        self._num_joints = len(self._joint_ids)
        
        # 初始化CuRobo控制器
        print(f"[CuRobo] Initializing controller for {cfg.asset_name}...")
        controller_config = CuRoboControllerConfig(
            robot_config_path=cfg.robot_config_path,
            world_config_path=cfg.world_config_path,
            num_seeds=cfg.num_seeds,
            max_attempts=cfg.max_attempts,
            planning_time_budget=cfg.planning_time_budget,
            interpolation_dt=cfg.interpolation_dt,
            enable_graph=cfg.enable_graph_cache,
            enable_opt=cfg.enable_optimization,
        )
        
        self.curobo_controller = CuRoboController(controller_config)
        print(f"[CuRobo] Controller initialized successfully!")
        
        # 设置IK后备方案
        if cfg.fallback_to_ik:
            self._setup_ik_fallback()
    
    def _setup_ik_fallback(self):
        """设置IK后备方案，当CuRobo规划失败时使用"""
        try:
            from isaaclab.controllers.differential_ik import DifferentialIKController
            from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
            
            ik_config = DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=False,
                ik_method="dls"
            )
            
            self.ik_fallback = DifferentialIKController(
                ik_config, 
                joint_names=self.cfg.joint_names,
                body_name=self.cfg.body_name
            )
            print("[CuRobo] IK fallback initialized")
        except ImportError:
            print("[CuRobo] Warning: IK fallback not available")
            self.ik_fallback = None
    
    @property
    def action_dim(self) -> int:
        """动作维度
        
        Returns:
            如果use_quaternion=True: 7 (x,y,z,qw,qx,qy,qz)
            如果use_quaternion=False: 6 (x,y,z,rx,ry,rz)
        """
        return 7 if self.cfg.use_quaternion else 6
    
    def apply(self, actions: torch.Tensor) -> None:
        """应用CuRobo运动规划
        
        Args:
            actions: [num_envs, action_dim] 目标位姿
                    如果action_dim=7: (x,y,z,qw,qx,qy,qz)
                    如果action_dim=6: (x,y,z,rx,ry,rz)
        """
        # 缩放动作
        scaled_actions = actions * self.cfg.scale
        
        # 获取当前机器人状态
        current_joint_pos = self._robot.data.joint_pos[:, self._joint_ids]
        
        # CuRobo运动规划
        try:
            planning_results = self.curobo_controller.plan_motion(
                current_joint_state=current_joint_pos,
                target_pose=scaled_actions
            )
            
            # 处理规划结果
            joint_targets = []
            success_count = 0
            
            for i, result in enumerate(planning_results):
                if result['success']:
                    # 使用CuRobo规划结果
                    joint_targets.append(result['next_joint_state'])
                    success_count += 1
                else:
                    # 规划失败，使用IK后备方案
                    if self.cfg.fallback_to_ik and hasattr(self, 'ik_fallback') and self.ik_fallback is not None:
                        ik_result = self._ik_fallback_solve(scaled_actions[i], current_joint_pos[i])
                        joint_targets.append(ik_result)
                    else:
                        # 保持当前位置
                        joint_targets.append(current_joint_pos[i])
            
            # 打印成功率统计
            if len(planning_results) > 0:
                success_rate = success_count / len(planning_results)
                if success_rate < 0.8:  # 成功率低于80%时打印警告
                    print(f"[CuRobo] Planning success rate: {success_rate:.2%}")
            
            # 设置关节目标
            joint_targets_tensor = torch.stack(joint_targets)
            self._robot.set_joint_position_target(
                joint_targets_tensor, 
                joint_ids=self._joint_ids
            )
            
        except Exception as e:
            print(f"[CuRobo] Error in motion planning: {e}")
            # 发生错误时保持当前位置
            self._robot.set_joint_position_target(
                current_joint_pos, 
                joint_ids=self._joint_ids
            )
    
    def _ik_fallback_solve(self, target_pose: torch.Tensor, current_joints: torch.Tensor) -> torch.Tensor:
        """IK后备求解
        
        Args:
            target_pose: 目标位姿
            current_joints: 当前关节角度
            
        Returns:
            目标关节角度
        """
        if hasattr(self, 'ik_fallback') and self.ik_fallback is not None:
            try:
                # 使用IK控制器求解
                ik_command = target_pose.unsqueeze(0)  # 添加batch维度
                joint_targets = self.ik_fallback.compute(
                    ik_command,
                    current_joints.unsqueeze(0)
                )
                return joint_targets.squeeze(0)
            except Exception as e:
                print(f"[CuRobo] IK fallback failed: {e}")
                return current_joints
        else:
            return current_joints
    
    def reset(self, env_ids: torch.Tensor | None = None) -> None:
        """重置Action状态
        
        Args:
            env_ids: 要重置的环境ID，None表示重置所有环境
        """
        self.curobo_controller.reset()


# 设置class_type
CuRoboMotionPlanningActionCfg.class_type = CuRoboMotionPlanningAction
