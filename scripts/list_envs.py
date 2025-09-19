# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Script to print all the available environments in Isaac Lab.

The script iterates over all registered environments and stores the details in a table.
It prints the name of the environment, the entry point and the config file.

All the environments are registered in the `Manipu_lab` extension. They start
with `Isaac` in their name.
"""

"""Launch Isaac Sim Simulator first."""

import os
# 設置圖形相關環境變數以避免 API 切換問題
os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"
os.environ["OMNI_KIT_ALLOW_ROOT"] = "1"
# 強制使用 RTX 渲染器避免 API 切換
os.environ["OMNI_RENDERER"] = "rtx"
# 禁用圖形驅動程式快取以避免 shader cache 錯誤
os.environ["OMNI_DISABLE_GPU_CACHE"] = "1"
# 強制使用單一圖形介面
os.environ["OMNI_FORCE_GRAPHICS_INTERFACE"] = "1"

from isaaclab.app import AppLauncher

# launch omniverse app with specific graphics settings
app_launcher = AppLauncher(
    headless=True,
    experience="",  # 使用空字符串避免載入不必要的擴展
)
simulation_app = app_launcher.app


"""Rest everything follows."""

import gymnasium as gym
from prettytable import PrettyTable

# 只導入我們的擴展，避免重複註冊 Isaac Lab 環境
import Manipu_lab.tasks  # noqa: F401

# 清除可能的重複註冊
import warnings
warnings.filterwarnings("ignore", message=".*Overriding environment.*already in registry.*")


def main():
    """Print all environments registered in `Manipu_lab` extension."""
    # print all the available environments
    table = PrettyTable(["S. No.", "Task Name", "Entry Point", "Config"])
    table.title = "Available Environments in Isaac Lab"
    # set alignment of table columns
    table.align["Task Name"] = "l"
    table.align["Entry Point"] = "l"
    table.align["Config"] = "l"

    # count of environments
    index = 0
    # acquire all Isaac environments names
    for task_spec in gym.registry.values():
        if "Template-" in task_spec.id:
            # add details to table
            table.add_row([index + 1, task_spec.id, task_spec.entry_point, task_spec.kwargs["env_cfg_entry_point"]])
            # increment count
            index += 1

    print(table)


if __name__ == "__main__":
    try:
        # run the main function
        main()
    except Exception as e:
        raise e
    finally:
        # close the app
        simulation_app.close()
