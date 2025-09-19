# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

# Import CuRobo action configuration
from Manipu_lab.controllers.curobo_action_cfg import CuRoboMotionPlanningActionCfg

# Import CuRobo config utilities
from Manipu_lab.assets.configs import get_config_paths, get_curobo_params

# Import base configuration
from . import joint_pos_env_cfg

##
# Pre-defined configs
##
from Manipu_lab.assets.piper import PIPER_CFG  # isort: skip


@configclass
class PiperReachCuRoboEnvCfg(joint_pos_env_cfg.PiperReachEnvCfg):
    """使用CuRobo控制器的Piper reach任务配置
    
    这个配置使用CuRobo GPU加速运动规划替代传统的关节控制。
    CuRobo提供：
    - 实时轨迹优化
    - 碰撞避免
    - 高质量smooth轨迹
    - GPU并行加速
    """
    
    def __post_init__(self):
        # 调用父类初始化
        super().__post_init__()
        
        # 设置机器人配置
        self.scene.robot = PIPER_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        
        # 获取CuRobo配置路径和参数
        config_paths = get_config_paths("piper")
        curobo_params = get_curobo_params("training")  # 使用训练预设
        
        # 配置CuRobo Motion Planning Action
        self.actions.arm_action = CuRoboMotionPlanningActionCfg(
            asset_name="robot",
            joint_names=["joint[1-6]"],
            body_name="gripper_base",
            
            # === CuRobo配置文件路径 (通过工具函数获取) ===
            robot_config_path=config_paths["robot_config_path"],
            world_config_path=config_paths["world_config_path"],
            
            # === CuRobo性能参数 (使用预设，可覆盖) ===
            num_seeds=64,                    # 覆盖预设，增加种子数提高成功率
            max_attempts=curobo_params["max_attempts"],
            planning_time_budget=0.008,      # 覆盖预设，8ms时间预算保证实时性
            interpolation_dt=0.02,           # 50Hz插值频率
            
            # === 功能配置 ===
            enable_collision_avoidance=True,  # 启用碰撞避免
            enable_graph_cache=True,          # 启用图缓存加速
            enable_optimization=curobo_params["enable_optimization"],
            fallback_to_ik=True,             # 失败时回退到IK
            
            # === 动作空间配置 ===
            use_quaternion=True,             # 使用四元数(7维)
            scale=1.0,                       # 动作缩放
        )
        
        # === 观测空间调整 ===
        # CuRobo直接从位姿到关节空间，不需要关节信息作为观测
        # 可以移除关节位置和速度观测来减小观测空间
        # self.observations.policy.joint_pos = None
        # self.observations.policy.joint_vel = None
        
        # === 奖励函数调整 ===
        # CuRobo提供更平滑的轨迹，可以适当调整奖励权重
        # 增加对位置精度的奖励
        if hasattr(self.rewards, 'end_effector_position_tracking'):
            self.rewards.end_effector_position_tracking.weight = 2.0
            
        # 减少对关节速度的惩罚（CuRobo轨迹更平滑）
        if hasattr(self.rewards, 'joint_vel'):
            self.rewards.joint_vel.weight = 0.05
            
        # === 训练参数优化 ===
        # CuRobo提供更稳定的控制，可以增加学习率
        # 这需要在agent配置中设置


@configclass
class PiperReachCuRoboEnvCfg_PLAY(PiperReachCuRoboEnvCfg):
    """CuRobo Piper Reach Play配置"""
    
    def __post_init__(self):
        # 调用父类初始化
        super().__post_init__()
        
        # === Play模式优化 ===
        # 减少环境数量以便观察
        self.scene.num_envs = 16
        self.scene.env_spacing = 2.5
        
        # 禁用观测噪声
        self.observations.policy.enable_corruption = False
        
        # === CuRobo Play模式优化 ===
        # 使用高质量预设参数
        play_params = get_curobo_params("high_quality")
        self.actions.arm_action.num_seeds = play_params["num_seeds"]
        self.actions.arm_action.planning_time_budget = play_params["planning_time_budget"]
        self.actions.arm_action.max_attempts = play_params["max_attempts"]


# 高质量模式配置（用于最终评估）
@configclass  
class PiperReachCuRoboEnvCfg_HIGHQUALITY(PiperReachCuRoboEnvCfg):
    """高质量CuRobo配置，牺牲速度换取最佳规划质量"""
    
    def __post_init__(self):
        super().__post_init__()
        
        # 使用生产级预设参数
        production_params = get_curobo_params("production")
        self.actions.arm_action.num_seeds = production_params["num_seeds"]
        self.actions.arm_action.planning_time_budget = production_params["planning_time_budget"]
        self.actions.arm_action.max_attempts = production_params["max_attempts"]
        
        # 减少环境数量以处理计算负载
        self.scene.num_envs = 256


# 快速模式配置（用于快速原型开发）
@configclass
class PiperReachCuRoboEnvCfg_FAST(PiperReachCuRoboEnvCfg):
    """快速CuRobo配置，牺牲质量换取速度"""
    
    def __post_init__(self):
        super().__post_init__()
        
        # 使用开发预设参数
        dev_params = get_curobo_params("development")
        self.actions.arm_action.num_seeds = dev_params["num_seeds"]
        self.actions.arm_action.planning_time_budget = dev_params["planning_time_budget"]
        self.actions.arm_action.max_attempts = dev_params["max_attempts"]
        self.actions.arm_action.enable_optimization = dev_params["enable_optimization"]
