# Activate the conda env that holds Gazebo Harmonic (RoboStack), for Windows,
# and point Gazebo at this folder's models and worlds.
# Usage:  . .\gz-env.ps1
# Same env as Ster-Vis's sim/gazebo/activate-ros2gz.ps1. Set STER_VIS_CONDA to
# your conda root and STER_VIS_GZ_ENV to the env name if they differ.

$condaRoot = if ($env:STER_VIS_CONDA) { $env:STER_VIS_CONDA } else { 'C:\ProgramData\miniforge3' }
$envName = if ($env:STER_VIS_GZ_ENV) { $env:STER_VIS_GZ_ENV } else { 'ros2gz' }
$condaHook = Join-Path $condaRoot 'shell\condabin\conda-hook.ps1'
if (-not (Test-Path $condaHook)) { throw "conda hook not found at $condaHook - set STER_VIS_CONDA" }
. $condaHook
# RoboStack's ros-workspace hook calls a local_setup.ps1 that it does not
# ship; the error is harmless, so do not let it abort activation.
$prev = $ErrorActionPreference; $ErrorActionPreference = 'Continue'
conda activate $envName 2>$null
$ErrorActionPreference = $prev

# PowerShell activation skips the env's .bat hooks, and one of them tells
# gz-rendering where OGRE's shaders live. Without it cameras render nothing.
$env:OGRE_RESOURCE_PATH = "$env:CONDA_PREFIX\Library\bin"
$env:OGRE2_RESOURCE_PATH = "$env:CONDA_PREFIX\Library\bin\OGRE-Next"
$env:SDF_PATH = "$env:CONDA_PREFIX\Library\share\sdformat13\1.10"

$env:PYTHONUNBUFFERED = '1'
$env:GZ_IP = '127.0.0.1'
$env:GZ_VERSION = 'harmonic'
$sep = [IO.Path]::PathSeparator
$env:GZ_SIM_RESOURCE_PATH = (Join-Path $PSScriptRoot 'models') + $sep + (Join-Path $PSScriptRoot 'worlds') + $sep + $env:GZ_SIM_RESOURCE_PATH

# `python -m amr run --sim` straight from this checkout, and Ster-Vis from its
# checkout beside it (or set STER_VIS_SRC; or pip install ster-vis instead).
$repo = Split-Path $PSScriptRoot -Parent
$sterVis = if ($env:STER_VIS_SRC) { $env:STER_VIS_SRC } else {
    @('Stereo-vision', 'Ster-Vis') | ForEach-Object { Join-Path (Split-Path $repo -Parent) "$_\src" } |
        Where-Object { Test-Path (Join-Path $_ 'stereo_vision') } | Select-Object -First 1
}
$env:PYTHONPATH = (@((Join-Path $repo 'src'), $sterVis, $env:PYTHONPATH) | Where-Object { $_ }) -join $sep
