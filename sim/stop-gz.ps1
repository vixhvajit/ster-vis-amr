# Stop the Gazebo server started by run-gz.ps1, the GUI, check_model.py and
# `amr run --sim`, whichever are running.
Get-CimInstance Win32_Process |
    Where-Object { $_.CommandLine -match '\\worlds\\amr_\w+\.sdf|sim\\run-gz\.ps1|check_model\.py|-m amr run| sim -g' } |
    ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
Write-Host "Gazebo stopped."
