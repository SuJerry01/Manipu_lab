# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""CuRobo controller wrapper for Isaac Lab integration."""

from __future__ import annotations

import torch
from typing import Optional, Dict, Any, List
from dataclasses import dataclass


@dataclass
class CuRoboControllerConfig:
    """CuRobo控制器配置类"""
    
    # === 文件路径 ===
    robot_config_path: str
    """机器人配置文件路径"""
    
    world_config_path: Optional[str] = None
    """世界环境配置文件路径"""
    
    motion_gen_config_path: Optional[str] = None
    """运动生成配置文件路径"""
    
    # === GPU和设备配置 ===
    device: str = "cuda:0"
    """GPU设备"""
    
    dtype: str = "float32"
    """数据类型"""
    
    # === 性能参数 ===
    batch_size: int = 1
    """批处理大小"""
    
    num_seeds: int = 32
    """并行规划种子数"""
    
    # === 优化参数 ===
    max_attempts: int = 5
    """最大规划尝试次数"""
    
    enable_graph: bool = True
    """是否启用图缓存"""
    
    enable_opt: bool = True
    """是否启用优化"""
    
    # === 时间参数 ===
    interpolation_dt: float = 0.02
    """轨迹插值时间步长"""
    
    planning_time_budget: float = 0.01
    """规划时间预算(秒)"""


class CuRoboController:
    """CuRobo GPU加速运动控制器包装类
    
    这个类封装了CuRobo的MotionGen功能，提供与Isaac Lab兼容的接口。
    """
    
    def __init__(self, config: CuRoboControllerConfig):
        """初始化CuRobo控制器
        
        Args:
            config: CuRobo控制器配置
        """
        self.config = config
        self._motion_gen = None
        self._is_initialized = False
        
        # 延迟初始化，避免导入问题
        self._lazy_init()
    
    def _lazy_init(self):
        """延迟初始化CuRobo组件"""
        try:
            self._setup_motion_generator()
            self._warmup_gpu()
            self._is_initialized = True
            print("[CuRobo] Controller initialized successfully!")
        except ImportError as e:
            print(f"[CuRobo] Import error: {e}")
            print("[CuRobo] Please install CuRobo: pip install curobo[all]")
            self._is_initialized = False
        except Exception as e:
            print(f"[CuRobo] Initialization error: {e}")
            self._is_initialized = False
    
    def _setup_motion_generator(self):
        """初始化CuRobo运动生成器"""
        # 导入CuRobo模块
        from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
        from curobo.types.base import TensorDeviceType
        from curobo.types.math import Pose
        from curobo.types.robot import RobotConfig
        from curobo.geom.types import WorldConfig
        
        # 设置tensor参数
        tensor_args = TensorDeviceType(device=self.config.device, dtype=getattr(torch, self.config.dtype))
        
        # 加载机器人配置
        robot_config = RobotConfig.load_from_config(
            self.config.robot_config_path,
            tensor_args
        )
        
        # 加载世界配置
        world_config = None
        if self.config.world_config_path:
            world_config = WorldConfig.load_from_config(
                self.config.world_config_path
            )
        
        # 创建运动生成配置
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_config,
            world_config,
            tensor_args=tensor_args,
            num_seeds=self.config.num_seeds,
            interpolation_dt=self.config.interpolation_dt
        )
        
        # 初始化运动生成器
        self._motion_gen = MotionGen(motion_gen_config)
        
        # 存储相关类型以便后续使用
        self._Pose = Pose
        self._tensor_args = tensor_args
    
    def _warmup_gpu(self):
        """GPU预热，减少首次调用延迟"""
        if not self._is_initialized and self._motion_gen is None:
            return
            
        print("[CuRobo] Warming up GPU kernels...")
        
        try:
            # 创建虚拟目标进行预热
            dummy_pose = self._Pose.from_list([0.5, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0])
            dummy_start_state = self._motion_gen.get_active_js().unsqueeze(0)
            
            # 执行几次预热规划
            for i in range(5):
                result = self._motion_gen.plan_single(
                    dummy_start_state,
                    dummy_pose,
                    enable_graph=self.config.enable_graph,
                    enable_opt=self.config.enable_opt,
                    max_attempts=1
                )
            
            print("[CuRobo] GPU warmup completed!")
        except Exception as e:
            print(f"[CuRobo] Warmup warning: {e}")
    
    def plan_motion(
        self, 
        current_joint_state: torch.Tensor,
        target_pose: torch.Tensor,
        **kwargs
    ) -> List[Dict[str, Any]]:
        """规划从当前关节状态到目标位姿的运动
        
        Args:
            current_joint_state: [batch, n_joints] 当前关节状态
            target_pose: [batch, 6/7] 目标位姿
                        6维: (x,y,z,rx,ry,rz)
                        7维: (x,y,z,qw,qx,qy,qz)
            
        Returns:
            规划结果列表，每个环境一个结果字典:
            {
                'success': bool,
                'trajectory': tensor or None,
                'next_joint_state': tensor,
                'planning_time': float,
                'trajectory_length': int
            }
        """
        if not self._is_initialized or self._motion_gen is None:
            # CuRobo未初始化，返回失败结果
            batch_size = current_joint_state.shape[0]
            return [
                {
                    'success': False,
                    'trajectory': None,
                    'next_joint_state': current_joint_state[i],
                    'planning_time': 0.0,
                    'trajectory_length': 0
                }
                for i in range(batch_size)
            ]
        
        batch_size = current_joint_state.shape[0]
        
        # 转换目标位姿格式
        if target_pose.shape[-1] == 6:  # [x,y,z,rx,ry,rz]
            target_pose = self._convert_euler_to_quat(target_pose)
        
        results = []
        
        for i in range(batch_size):
            try:
                # 单个环境的规划
                pose = self._Pose.from_list(target_pose[i].tolist())
                start_state = current_joint_state[i].unsqueeze(0)
                
                # CuRobo运动规划
                result = self._motion_gen.plan_single(
                    start_state,
                    pose,
                    enable_graph=self.config.enable_graph,
                    enable_opt=self.config.enable_opt,
                    max_attempts=self.config.max_attempts
                )
                
                if result.success:
                    # 获取插值轨迹
                    interpolated_plan = result.get_interpolated_plan()
                    trajectory = interpolated_plan.position
                    
                    results.append({
                        'success': True,
                        'trajectory': trajectory,
                        'next_joint_state': trajectory[1] if len(trajectory) > 1 else trajectory[0],
                        'planning_time': result.solve_time,
                        'trajectory_length': len(trajectory)
                    })
                else:
                    # 规划失败，返回当前状态
                    results.append({
                        'success': False,
                        'trajectory': None,
                        'next_joint_state': current_joint_state[i],
                        'planning_time': 0.0,
                        'trajectory_length': 0
                    })
                    
            except Exception as e:
                print(f"[CuRobo] Planning error for env {i}: {e}")
                results.append({
                    'success': False,
                    'trajectory': None,
                    'next_joint_state': current_joint_state[i],
                    'planning_time': 0.0,
                    'trajectory_length': 0
                })
        
        return results
    
    def _convert_euler_to_quat(self, euler_pose: torch.Tensor) -> torch.Tensor:
        """将欧拉角位姿转换为四元数位姿
        
        Args:
            euler_pose: [batch, 6] (x,y,z,rx,ry,rz)
            
        Returns:
            quat_pose: [batch, 7] (x,y,z,qw,qx,qy,qz)
        """
        try:
            from curobo.util.torch_utils import quat_from_euler_xyz
            
            pos = euler_pose[..., :3]
            euler = euler_pose[..., 3:6]
            quat = quat_from_euler_xyz(euler)
            
            # 重新排列为 [x,y,z,qw,qx,qy,qz]
            pose_quat = torch.cat([pos, quat], dim=-1)
            return pose_quat
        except ImportError:
            # 如果CuRobo不可用，使用简单的恒等四元数
            batch_size = euler_pose.shape[0]
            pos = euler_pose[..., :3]
            quat = torch.tensor([1.0, 0.0, 0.0, 0.0], device=euler_pose.device).repeat(batch_size, 1)
            return torch.cat([pos, quat], dim=-1)
    
    def update_world(self, world_config: Dict[str, Any]):
        """动态更新世界环境配置
        
        Args:
            world_config: 世界配置字典
        """
        if not self._is_initialized or self._motion_gen is None:
            print("[CuRobo] Cannot update world: controller not initialized")
            return
            
        try:
            from curobo.geom.types import WorldConfig
            world_cfg = WorldConfig.from_dict(world_config)
            self._motion_gen.update_world(world_cfg)
            print("[CuRobo] World configuration updated")
        except Exception as e:
            print(f"[CuRobo] Failed to update world: {e}")
    
    def reset(self):
        """重置控制器状态"""
        # CuRobo内部会处理状态重置
        # 这里可以添加额外的重置逻辑如果需要
        pass
    
    @property
    def is_initialized(self) -> bool:
        """检查控制器是否已初始化"""
        return self._is_initialized
