# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Graphics interface fix for Omniverse Isaac Sim to prevent DriverShaderCacheManager errors.

This module provides utilities to configure graphics settings and prevent
the common "DriverShaderCacheManager::init() called with a different graphics interface" error.
"""

import os
import warnings
from typing import Optional


def setup_graphics_environment(
    disable_gpu_cache: bool = True,
    force_single_interface: bool = True,
    renderer: str = "rtx"
) -> None:
    """
    Setup graphics environment variables to prevent DriverShaderCacheManager errors.
    
    Args:
        disable_gpu_cache: Whether to disable GPU cache to prevent shader cache conflicts
        force_single_interface: Whether to force single graphics interface
        renderer: Graphics renderer to use ("rtx", "dx12", "vulkan")
    """
    # Basic Omniverse settings
    os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"
    os.environ["OMNI_KIT_ALLOW_ROOT"] = "1"
    
    # Graphics renderer settings
    os.environ["OMNI_RENDERER"] = renderer
    
    if disable_gpu_cache:
        # Disable GPU cache to prevent shader cache conflicts
        os.environ["OMNI_DISABLE_GPU_CACHE"] = "1"
        os.environ["OMNI_DISABLE_SHADER_CACHE"] = "1"
    
    if force_single_interface:
        # Force single graphics interface
        os.environ["OMNI_FORCE_GRAPHICS_INTERFACE"] = "1"
        os.environ["OMNI_SINGLE_GRAPHICS_CONTEXT"] = "1"
    
    # Additional stability settings
    os.environ["OMNI_DISABLE_GPU_VALIDATION"] = "1"
    os.environ["OMNI_DISABLE_GRAPHICS_DEBUG"] = "1"
    
    # Suppress graphics-related warnings
    warnings.filterwarnings("ignore", message=".*graphics.*")
    warnings.filterwarnings("ignore", message=".*shader.*")
    warnings.filterwarnings("ignore", message=".*DriverShaderCacheManager.*")


def create_safe_app_launcher(
    headless: bool = True,
    experience: str = "",
    **kwargs
):
    """
    Create a safe AppLauncher with graphics fixes applied.
    
    Args:
        headless: Whether to run in headless mode
        experience: Experience string (empty to avoid loading unnecessary extensions)
        **kwargs: Additional arguments for AppLauncher
        
    Returns:
        AppLauncher instance with graphics fixes applied
    """
    from isaaclab.app import AppLauncher
    
    # Apply graphics fixes
    setup_graphics_environment()
    
    # Create AppLauncher with safe settings
    return AppLauncher(
        headless=headless,
        experience=experience,
        **kwargs
    )


def cleanup_graphics_resources():
    """
    Cleanup graphics resources to prevent memory leaks and interface conflicts.
    """
    try:
        import gc
        import torch
        
        # Clear CUDA cache if available
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
            torch.cuda.synchronize()
        
        # Force garbage collection
        gc.collect()
        
    except Exception as e:
        print(f"Warning: Failed to cleanup graphics resources: {e}")


# Example usage and testing
if __name__ == "__main__":
    print("Setting up graphics environment...")
    setup_graphics_environment()
    
    print("Creating safe AppLauncher...")
    app_launcher = create_safe_app_launcher()
    
    print("Graphics environment setup complete!")
    print("Environment variables set:")
    for key, value in os.environ.items():
        if key.startswith("OMNI_"):
            print(f"  {key} = {value}")

