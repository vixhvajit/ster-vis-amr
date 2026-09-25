# Start the Gazebo server in the background (hidden), logging to output\gz.*.
# Usage:  .\start-gz.ps1 ; python check_model.py ; .\stop-gz.ps1
param([string]$World = 'amr_test.sdf')
$out = Join-Path $PSScriptRoot 'output'
New-Item -ItemType Directory -Force $out | Out-Null
$p = Start-Process powershell -PassThru -WindowStyle Hidden `
    -ArgumentList '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', (Join-Path $PSScriptRoot 'run-gz.ps1'), '-World', $World `
    -RedirectStandardOutput "$out\gz.log" -RedirectStandardError "$out\gz.err"
Write-Host "Gazebo server starting on $World (pid $($p.Id)); logs in output\gz.log / gz.err"
