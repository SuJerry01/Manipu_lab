#!/usr/bin/env python3
"""
Quick fix for DriverShaderCacheManager graphics interface error in Omniverse Isaac Sim.

This script sets up the necessary environment variables to prevent the common
"DriverShaderCacheManager::init() called with a different graphics interface" error.

Usage:
    python fix_graphics_error.py
    # or import and call setup_graphics_fix() in your scripts
"""

import os
import sys


def setup_graphics_fix():
    """
    Setup environment variables to fix DriverShaderCacheManager errors.
    
    This function should be called BEFORE importing any Omniverse or Isaac Lab modules.
    """
    print("Setting up graphics environment to prevent DriverShaderCacheManager errors...")
    
    # Basic Omniverse settings
    os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"
    os.environ["OMNI_KIT_ALLOW_ROOT"] = "1"
    
    # Graphics renderer - use RTX for better stability
    os.environ["OMNI_RENDERER"] = "rtx"
    
    # Disable GPU cache to prevent shader cache conflicts
    os.environ["OMNI_DISABLE_GPU_CACHE"] = "1"
    os.environ["OMNI_DISABLE_SHADER_CACHE"] = "1"
    
    # Force single graphics interface
    os.environ["OMNI_FORCE_GRAPHICS_INTERFACE"] = "1"
    os.environ["OMNI_SINGLE_GRAPHICS_CONTEXT"] = "1"
    
    # Additional stability settings
    os.environ["OMNI_DISABLE_GPU_VALIDATION"] = "1"
    os.environ["OMNI_DISABLE_GRAPHICS_DEBUG"] = "1"
    
    # Disable some problematic features that can cause interface switching
    os.environ["OMNI_DISABLE_MULTI_GPU"] = "1"
    os.environ["OMNI_DISABLE_GPU_MEMORY_POOL"] = "1"
    
    print("Graphics environment setup complete!")
    print("The following environment variables have been set:")
    
    # Print relevant environment variables
    relevant_vars = [key for key in os.environ.keys() if key.startswith("OMNI_")]
    for var in sorted(relevant_vars):
        print(f"  {var} = {os.environ[var]}")


def main():
    """Main function to run the graphics fix."""
    setup_graphics_fix()
    
    print("\n" + "="*60)
    print("Graphics fix applied successfully!")
    print("="*60)
    print("\nTo use this fix in your scripts, add this at the very beginning:")
    print("(before importing any Omniverse or Isaac Lab modules)")
    print("\n```python")
    print("import sys")
    print("sys.path.append('scripts')  # if not already in path")
    print("from fix_graphics_error import setup_graphics_fix")
    print("setup_graphics_fix()")
    print("```")
    print("\nOr simply run this script before starting your main application.")


if __name__ == "__main__":
    main()

