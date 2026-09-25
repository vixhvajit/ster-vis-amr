# Open the AMR test world in Gazebo with the GUI (Windows).
# Starts the server in the background, then the GUI client in its own window.
# Usage:  .\open-gz.ps1        # close with .\stop-gz.ps1 (and close the window)
param([string]$World = 'amr_test.sdf')
. $PSScriptRoot\gz-env.ps1
& $PSScriptRoot\start-gz.ps1 -World $World
Start-Sleep -Seconds 12        # let the server load the world first

$bin = "$env:CONDA_PREFIX\Library\bin"
Start-Process -FilePath "$bin\gz.bat" -ArgumentList 'sim', '-g' -WorkingDirectory $bin -WindowStyle Hidden
Start-Sleep -Seconds 15
# point the GUI camera at the robot
Push-Location $bin
& .\gz.bat service -s /gui/move_to --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 --req 'data: "amr"' 2>$null | Out-Null
Pop-Location
Write-Host "Gazebo GUI open on $World"
