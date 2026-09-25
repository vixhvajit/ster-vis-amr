# Run the Gazebo server on the AMR test world (Windows).
# Windows only runs the server (no gz GUI, gazebosim/gz-sim#168); look at the
# robot through check_model.py's pictures, or the /view/* camera topics.
#
# Usage:  .\run-gz.ps1            # runs until Ctrl+C; start-gz.ps1 runs it hidden
param([string]$World = 'amr_test.sdf')
$ErrorActionPreference = 'Stop'
. $PSScriptRoot\gz-env.ps1

$worldPath = Join-Path $PSScriptRoot "worlds\$World"
if (-not (Test-Path $worldPath)) {
    python (Join-Path $PSScriptRoot 'build_model.py')
}

$bin = "$env:CONDA_PREFIX\Library\bin"
# gz only loads gz-sim8-gz.dll with cwd = Library\bin; the world must be absolute.
Push-Location $bin
try {
    & (Join-Path $bin 'gz.bat') sim -s -r -v 3 $worldPath
} finally {
    Pop-Location
}
