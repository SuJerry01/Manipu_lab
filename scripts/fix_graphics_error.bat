@echo off
echo Setting up graphics environment to prevent DriverShaderCacheManager errors...

REM Set environment variables to fix graphics interface issues
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

echo Graphics environment variables set successfully!
echo.
echo You can now run your Isaac Lab scripts without the DriverShaderCacheManager error.
echo.
echo To make these settings permanent, you can:
echo 1. Add these environment variables to your system environment variables
echo 2. Or run this batch file before starting your Python scripts
echo.
pause

