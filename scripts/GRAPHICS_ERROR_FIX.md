# DriverShaderCacheManager 錯誤修復指南

## 問題描述

您遇到的錯誤訊息：
```
2025-09-16T06:44:53Z [39,791ms] [Error] [gpu.foundation.plugin] DriverShaderCacheManager::init() called with a different graphics interface, without a shutdown()!
```

這是一個常見的 Omniverse Isaac Sim 圖形驅動程式問題，通常發生在圖形介面切換或多次初始化時。

## 解決方案

### 方法 1: 使用 Python 修復腳本（推薦）

在您的 Python 腳本最開始處（在任何 Omniverse 或 Isaac Lab 模組導入之前）添加：

```python
import sys
sys.path.append('scripts')  # 如果 scripts 目錄不在 Python 路徑中
from fix_graphics_error import setup_graphics_fix
setup_graphics_fix()
```

### 方法 2: 手動設置環境變數

在運行 Python 腳本之前，設置以下環境變數：

```bash
# Windows (PowerShell)
$env:OMNI_KIT_ACCEPT_EULA="YES"
$env:OMNI_KIT_ALLOW_ROOT="1"
$env:OMNI_RENDERER="rtx"
$env:OMNI_DISABLE_GPU_CACHE="1"
$env:OMNI_DISABLE_SHADER_CACHE="1"
$env:OMNI_FORCE_GRAPHICS_INTERFACE="1"
$env:OMNI_SINGLE_GRAPHICS_CONTEXT="1"
$env:OMNI_DISABLE_GPU_VALIDATION="1"
$env:OMNI_DISABLE_GRAPHICS_DEBUG="1"
$env:OMNI_DISABLE_MULTI_GPU="1"
$env:OMNI_DISABLE_GPU_MEMORY_POOL="1"

# Windows (Command Prompt)
set OMNI_KIT_ACCEPT_EULA=YES
set OMNI_KIT_ALLOW_ROOT=1
set OMNI_RENDERER=rtx
set OMNI_DISABLE_GPU_CACHE=1
set OMNI_DISABLE_SHADER_CACHE=1
set OMNI_FORCE_GRAPHICS_INTERFACE=1
set OMNI_SINGLE_GRAPHICS_CONTEXT=1
set OMNI_DISABLE_GPU_VALIDATION=1
set OMNI_DISABLE_GRAPHICS_DEBUG=1
set OMNI_DISABLE_MULTI_GPU=1
set OMNI_DISABLE_GPU_MEMORY_POOL=1
```

### 方法 3: 使用批次檔案（Windows）

運行 `scripts/fix_graphics_error.bat` 來設置環境變數，然後運行您的 Python 腳本。

## 環境變數說明

| 變數名 | 作用 |
|--------|------|
| `OMNI_RENDERER=rtx` | 強制使用 RTX 渲染器，避免 API 切換 |
| `OMNI_DISABLE_GPU_CACHE=1` | 禁用 GPU 快取，防止 shader cache 衝突 |
| `OMNI_DISABLE_SHADER_CACHE=1` | 禁用 shader 快取 |
| `OMNI_FORCE_GRAPHICS_INTERFACE=1` | 強制使用單一圖形介面 |
| `OMNI_SINGLE_GRAPHICS_CONTEXT=1` | 強制使用單一圖形上下文 |
| `OMNI_DISABLE_GPU_VALIDATION=1` | 禁用 GPU 驗證以提升穩定性 |
| `OMNI_DISABLE_GRAPHICS_DEBUG=1` | 禁用圖形除錯模式 |
| `OMNI_DISABLE_MULTI_GPU=1` | 禁用多 GPU 支援 |
| `OMNI_DISABLE_GPU_MEMORY_POOL=1` | 禁用 GPU 記憶體池 |

## 其他建議

1. **更新驅動程式**: 確保您的 NVIDIA 驅動程式是最新版本
2. **重啟應用程式**: 在設置環境變數後重啟您的應用程式
3. **檢查相容性**: 確保您的 Omniverse Isaac Sim 版本與驅動程式相容
4. **使用 headless 模式**: 如果可能，使用 `headless=True` 來避免圖形介面問題

## 測試修復

運行以下命令來測試修復是否有效：

```bash
python scripts/fix_graphics_error.py
```

然後運行您的 Isaac Lab 腳本，應該不會再出現 DriverShaderCacheManager 錯誤。

## 如果問題仍然存在

1. 檢查 NVIDIA 驅動程式版本
2. 嘗試回滾到較舊的驅動程式版本
3. 聯繫 NVIDIA 或 Omniverse 技術支援
4. 檢查系統日誌以獲取更多錯誤資訊

