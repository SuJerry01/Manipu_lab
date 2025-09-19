# Isaac Lab 機器人訓練與執行指令手冊

本文檔包含 Isaac Lab 中所有支援機器人的訓練和執行指令。

## 支援的機器人

- **Franka Panda**: 7-DoF 機械臂配備 Panda 夾爪
- **Piper**: 6-DoF 機械臂配備 2-DoF 平行夾爪
- **OMY**: 6-DoF 機械臂配備 4-DoF 夾爪
- **P2D**: Piper雙臂機器人
- **FFW-BG2**: 雙臂機器人
- **FFW-SG2**: 雙臂機器人（含移動底座）
- **UR10**: Universal Robots  機械臂

## 支援的訓練框架

- **RSL-RL**
- **RL-Games**
- **Stable-Baselines3 (SB3)**
- **SKRL**

## 可用的任務環境

### 1. 到達任務 (Reach Tasks) ~ 6 embodiment

#### Franka Panda ~ 1

**訓練指令:**
```bash
# RSL-RL
python scripts/rsl_rl/train.py --task Isaac-Reach-Franka-v0 --headless

# RL-Games
python scripts/rl_games/train.py --task Isaac-Reach-Franka-v0 --headless

# Stable-Baselines3
python scripts/sb3/train.py --task Isaac-Reach-Franka-v0 --headless

# SKRL
python scripts/skrl/train.py --task Isaac-Reach-Franka-v0 --headless
```

**執行指令:**
```bash
# RSL-RL
python scripts/rsl_rl/play.py --task Isaac-Reach-Franka-v0 --num_envs 32 

# RL-Games
python scripts/rl_games/play.py --task Isaac-Reach-Franka-v0 --num_envs 32 

# Stable-Baselines3
python scripts/sb3/play.py --task Isaac-Reach-Franka-v0 --num_envs 32 

# SKRL
python scripts/skrl/play.py --task Isaac-Reach-Franka-v0 --num_envs 32 
```

**可用環境變體:**
- `Isaac-Reach-Franka-v0` (關節位置控制)
- `Isaac-Reach-Franka-IK-Abs-v0` (逆運動學絕對位姿控制)
- `Isaac-Reach-Franka-IK-Rel-v0` (逆運動學相對位姿控制)
- `Isaac-Reach-Franka-OSC-v0` (操作空間控制)

#### Piper ~ 2


**訓練指令:**
```bash
# RSL-RL
python scripts/rsl_rl/train.py --task Isaac-Reach-Piper-v0 --headless

# RL-Games
python scripts/rl_games/train.py --task Isaac-Reach-Piper-v0 --headless

# Stable-Baselines3
python scripts/sb3/train.py --task Isaac-Reach-Piper-v0 --headless

# SKRL
python scripts/skrl/train.py --task Isaac-Reach-Piper-v0 --headless
```

**執行指令:**
```bash
python scripts/rsl_rl/play.py --task Isaac-Reach-Piper-v0 --num_envs 32 
```

**可用環境變體:**
- `Isaac-Reach-Piper-v0` (關節位置控制)
- `Isaac-Reach-Piper-IK-Abs-v0` (逆運動學絕對位姿控制)
- `Isaac-Reach-Piper-IK-Rel-v0` (逆運動學相對位姿控制)
- `Isaac-Reach-Piper-OSC-v0` (操作空間控制)

#### OMY ~ 3

**訓練指令:**
```bash
# RSL-RL
python scripts/rsl_rl/train.py --task Isaac-Reach-OMY-v0 --headless

# RL-Games
python scripts/rl_games/train.py --task Isaac-Reach-OMY-v0 --headless
```

**執行指令:**
```bash
python scripts/rsl_rl/play.py --task Isaac-Reach-OMY-v0 --num_envs 32 
```

#### P2D ~ 4 (配置中)

**訓練指令:**
```bash
# RSL-RL
python scripts/rsl_rl/train.py --task Isaac-Reach-P2D-v0 --headless

# RL-Games
python scripts/rl_games/train.py --task Isaac-Reach-P2D-v0 --headless
```

**執行指令:**
```bash
python scripts/rsl_rl/play.py --task Isaac-Reach-P2D-v0 --num_envs 32 
```

#### FFW-BG2 ~ 5

**訓練指令:**
```bash
# RSL-RL
python scripts/rsl_rl/train.py --task Isaac-Reach-FFW-BG2-v0 --headless

# RL-Games
python scripts/rl_games/train.py --task Isaac-Reach-FFW-BG2-v0 --headless
```

**執行指令:**
```bash
python scripts/rsl_rl/play.py --task Isaac-Reach-FFW-BG2-v0 --num_envs 32 
```

#### UR10 ~ 6

**訓練指令:**
```bash
# RSL-RL
python scripts/rsl_rl/train.py --task Isaac-Reach-UR10-v0 --headless

# RL-Games
python scripts/rl_games/train.py --task Isaac-Reach-UR10-v0 --headless

# SKRL
python scripts/skrl/train.py --task Isaac-Reach-UR10-v0 --headless
```

**執行指令:**
```bash
python scripts/rsl_rl/play.py --task Isaac-Reach-UR10-v0 --num_envs 32 
```

### 2. 抓取任務 (Lift Tasks) ~ 3 embodiment

#### Franka Panda ~ 1

**訓練指令:**
```bash
# RSL-RL
python scripts/rsl_rl/train.py --task Isaac-Lift-Cube-Franka-v0 --headless

# RL-Games
python scripts/rl_games/train.py --task Isaac-Lift-Cube-Franka-v0 --headless

# Stable-Baselines3
python scripts/sb3/train.py --task Isaac-Lift-Cube-Franka-v0 --headless

# SKRL
python scripts/skrl/train.py --task Isaac-Lift-Cube-Franka-v0 --headless
```

**執行指令:**
```bash
# RSL-RL
python scripts/rsl_rl/play.py --task Isaac-Lift-Cube-Franka-v0 --num_envs 32 

# RL-Games
python scripts/rl_games/play.py --task Isaac-Lift-Cube-Franka-v0 --num_envs 32 
```

**可用環境變體:**
- `Isaac-Lift-Cube-Franka-v0` (關節位置控制)
- `Isaac-Lift-Cube-Franka-Play-v0` (遊戲模式)
- `Isaac-Lift-Cube-Franka-IK-Abs-v0` (逆運動學絕對位姿控制)
- `Isaac-Lift-Cube-Franka-IK-Rel-v0` (逆運動學相對位姿控制)
- `Isaac-Lift-Teddy-Bear-Franka-IK-Abs-v0` (抓取泰迪熊)

#### Piper ~ 2

**訓練指令:**
```bash
# RSL-RL
python scripts/rsl_rl/train.py --task Isaac-Lift-Cube-Piper-v0 --headless

# RL-Games
python scripts/rl_games/train.py --task Isaac-Lift-Cube-Piper-v0 --headless

# Stable-Baselines3
python scripts/sb3/train.py --task Isaac-Lift-Cube-Piper-v0 --headless

# SKRL
python scripts/skrl/train.py --task Isaac-Lift-Cube-Piper-v0 --headless
```

**執行指令:**
```bash
python scripts/rsl_rl/play.py --task Isaac-Lift-Cube-Piper-v0 --num_envs 32 
```

**可用環境變體:**
- `Isaac-Lift-Cube-Piper-v0` (關節位置控制)
- `Isaac-Lift-Cube-Piper-Play-v0` (遊戲模式)
- `Isaac-Lift-Cube-Piper-IK-Abs-v0` (逆運動學絕對位姿控制)
- `Isaac-Lift-Cube-Piper-IK-Rel-v0` (逆運動學相對位姿控制)
- `Isaac-Lift-Teddy-Bear-Piper-IK-Abs-v0` (抓取泰迪熊)

#### OMY ~ 3

**訓練指令:**
```bash
# RSL-RL
python scripts/rsl_rl/train.py --task Isaac-Lift-Cube-OMY-v0 --headless

# RL-Games
python scripts/rl_games/train.py --task Isaac-Lift-Cube-OMY-v0 --headless

# SKRL
python scripts/skrl/train.py --task Isaac-Lift-Cube-OMY-v0 --headless
```

**執行指令:**
```bash
python scripts/rsl_rl/play.py --task Isaac-Lift-Cube-OMY-v0 --num_envs 32 
```

### 3. 開櫃任務 (Cabinet Tasks) ~ 3 embodiment

#### Franka Panda ~ 1

**訓練指令:**
```bash
# RSL-RL
python scripts/rsl_rl/train.py --task Isaac-Open-Drawer-Franka-v0 --headless

# RL-Games
python scripts/rl_games/train.py --task Isaac-Open-Drawer-Franka-v0 --headless

# SKRL
python scripts/skrl/train.py --task Isaac-Open-Drawer-Franka-v0 --headless
```

**執行指令:**
```bash
python scripts/rsl_rl/play.py --task Isaac-Open-Drawer-Franka-v0 --num_envs 32 
```

**可用環境變體:**
- `Isaac-Open-Drawer-Franka-v0` (關節位置控制)
- `Isaac-Open-Drawer-Franka-Play-v0` (遊戲模式)
- `Isaac-Open-Drawer-Franka-IK-Abs-v0` (逆運動學絕對位姿控制)
- `Isaac-Open-Drawer-Franka-IK-Rel-v0` (逆運動學相對位姿控制)

#### Piper ~ 2

**訓練指令:**
```bash
# RSL-RL
python scripts/rsl_rl/train.py --task Isaac-Open-Drawer-Piper-v0 --headless

# RL-Games
python scripts/rl_games/train.py --task Isaac-Open-Drawer-Piper-v0 --headless

# SKRL
python scripts/skrl/train.py --task Isaac-Open-Drawer-Piper-v0 --headless
```

**執行指令:**
```bash
python scripts/rsl_rl/play.py --task Isaac-Open-Drawer-Piper-v0 --num_envs 32 
```

#### OMY ~ 3 

**訓練指令:**
```bash
# RSL-RL
python scripts/rsl_rl/train.py --task Isaac-Open-Drawer-OMY-v0 --headless --num_envs=512

# RL-Games
python scripts/rl_games/train.py --task Isaac-Open-Drawer-OMY-v0 --headless
```

**執行指令:**
```bash
python scripts/rsl_rl/play.py --task Isaac-Open-Drawer-OMY-v0 --num_envs 32 
```

### 4. 堆疊任務 ( IM/BC必須在Linux下 ) ~3 embodiment 

#### Franka Panda ~ 1

**訓練指令:**
```bash
# RSL-RL
python scripts/robomimic/train.py --task Isaac-Stack-Cube-Franka-v0 --headless

# RL-Games
python scripts/rl_robomimic/games/train.py --task Isaac-Stack-Cube-Franka-v0 --headless
```

**執行指令:**
```bash
python scripts/robomimic/play.py --task Isaac-Stack-Cube-Franka-v0 --num_envs 32 
```

**可用環境變體:**
- `Isaac-Stack-Cube-Franka-v0` (關節位置控制)
- `Isaac-Stack-Cube-Franka-IK-Abs-v0` (逆運動學絕對位姿控制)
- `Isaac-Stack-Cube-Franka-IK-Rel-v0` (逆運動學相對位姿控制)
- `Isaac-Stack-Cube-Franka-IK-Rel-Blueprint-v0` (Blueprint 模式)
- `Isaac-Stack-Cube-Franka-IK-Rel-Visuomotor-v0` (視覺運動模式)

#### Piper ~ 2 

**訓練指令:**
```bash
# RSL-RL
python scripts/robomimic/train.py --task Isaac-Stack-Cube-Piper-v0 --headless

# RL-Games
python scripts/robomimic/train.py --task Isaac-Stack-Cube-Piper-v0 --headless
```

**執行指令:**
```bash
python scripts/robomimic/play.py --task Isaac-Stack-Cube-Piper-v0 --num_envs 32 
```

**可用環境變體:**
- `Isaac-Stack-Cube-Piper-v0` (關節位置控制)
- `Isaac-Stack-Cube-Piper-IK-Rel-v0` (逆運動學相對位姿控制)

#### OMY ~ 3

**訓練指令:**
```bash
# RSL-RL
python scripts/robomimic/train.py --task Isaac-Stack-Cube-OMY-v0 --headless

# RL-Games
python scripts/robomimic/train.py --task Isaac-Stack-Cube-OMY-v0 --headless
```

**執行指令:**
```bash
python scripts/robomimic/play.py --task Isaac-Stack-Cube-OMY-v0 --num_envs 32 
```

## 常用參數說明

### 通用參數
- `--task`: 任務環境名稱
- `--num_envs`: 並行環境數量 (可選，系統會使用預設值)
- `--seed`: 隨機種子
- `--checkpoint`: 檢查點路徑 (僅用於執行)
- `--video`: 錄製影片 (訓練時)
- `--headless`: 無渲染模式運行

### RSL-RL 特定參數
- `--max_iterations`: 最大迭代次數
- `--distributed`: 分散式訓練

### RL-Games 特定參數
- `--sigma`: 策略初始標準差
- `--wandb-project-name`: Weights & Biases 專案名稱
- `--track`: 啟用 W&B 追蹤

### SKRL 特定參數
- `--agent`: 智慧體配置入口點 (如 skrl_ppo_cfg_entry_point, skrl_mappo_cfg_entry_point)

## 範例用法

### 1. 訓練 Franka 機器人到達任務
```bash
python scripts/rsl_rl/train.py \
    --task Isaac-Reach-Franka-v0 \
    --max_iterations 1000 \
    --seed 42 \
    --headless
```

### 2. 執行預訓練的 Piper 抓取模型
```bash
python scripts/rsl_rl/play.py \
    --task Isaac-Lift-Cube-Piper-Play-v0 \
    --num_envs 32 \
    --checkpoint /path/to/model.pt
```

### 3. 使用 Weights & Biases 追蹤訓練
```bash
python scripts/rl_games/train.py \
    --task Isaac-Stack-Cube-Franka-v0 \
    --track \
    --wandb-project-name "isaac-lab-manipulation" \
    --headless
```

### 4. 錄製訓練影片
```bash
python scripts/rsl_rl/train.py \
    --task Isaac-Open-Drawer-Piper-v0 \
    --video \
    --video_length 200 \
    --video_interval 1000 \
    --headless
```

## 檢查點和日誌

### 檢查點位置
- **RSL-RL**: `logs/rsl_rl/{task_name}/{experiment_name}/model_*.pt`
- **RL-Games**: `logs/rl_games/{task_name}/nn/*.pth`
- **SB3**: `logs/sb3/{task_name}/*.zip`
- **SKRL**: `logs/skrl/{task_name}/checkpoints/*.pt`

### 日誌位置
- **TensorBoard 日誌**: `logs/{framework}/{task_name}/summaries/`
- **Weights & Biases**: 自動上傳到 W&B 雲端

## 故障排除

### 常見問題

1. **記憶體不足錯誤**
   - 減少 `--num_envs` 參數
   - 使用較小的批次大小

2. **GPU 記憶體不足**
   - 降低環境數量
   - 啟用 `--headless` 模式

3. **無法找到環境**
   - 確認環境名稱拼寫正確
   - 檢查擴展是否正確安裝

4. **檢查點載入失敗**
   - 確認檢查點路徑正確
   - 檢查模型架構相容性

### 效能最佳化

1. **訓練速度最佳化**
   - 使用多個 GPU 進行分散式訓練
   - 調整 `num_envs` 以平衡記憶體和速度
   - 啟用無頭模式

2. **穩定性改善**
   - 使用適當的學習率
   - 調整獎勵縮放因子
   - 設定適當的隨機種子

---

