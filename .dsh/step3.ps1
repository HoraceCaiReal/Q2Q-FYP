# Reproducible entry point from the Windows workspace.
param(
    [ValidateSet('Deploy', 'Start', 'Test', 'Status', 'Stop')]
    [string]$Action = 'Status',
    [switch]$Gui,
    [switch]$KinematicCarry,
    [ValidateSet('q2q_warehouse', 'q2q_warehouse_grasp50', 'q2q_warehouse_grasp80')]
    [string]$World = 'q2q_warehouse',
    [ValidateSet('mesh', 'convex')]
    [string]$GripperCollision = 'mesh',
    [ValidateRange(0.05,1.0)]
    [double]$PositionGain = 0.1,
    [ValidateSet('mimic','commanded','suction')]
    [string]$GripperMode = 'mimic'
)
$ErrorActionPreference = 'Stop'
$key = Join-Path $env:USERPROFILE '.ssh\id_ed25519'
$known = Join-Path $PSScriptRoot 'known_hosts'
$common = @('-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=accept-new',
            '-o', "UserKnownHostsFile=$known", '-o', 'ConnectTimeout=8')
$sshOptions = @('-i', $key, '-p', '2222') + $common
$scpOptions = @('-i', $key, '-P', '2222') + $common

if ($Action -eq 'Deploy') {
    # Reuse the current PowerShell executable (works in PS5 and PS7).
    $taskShell = (Get-Process -Id $PID).Path
    & $taskShell -NoProfile -File (Join-Path $PSScriptRoot 'sync.ps1')
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & ssh @sshOptions ros@127.0.0.1 'mkdir -p /home/ros/.q2q'
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    foreach ($name in @('step3_env.sh', 'step3_control.py', 'fastdds_udp.xml')) {
        & scp @scpOptions (Join-Path $PSScriptRoot $name) 'ros@127.0.0.1:/home/ros/.q2q/'
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    }
    Write-Output 'DEPLOY_OK (this does not start or pass the test)'
    exit 0
}
$verb = $Action.ToLowerInvariant()
$extra = if ($Gui -and $verb -eq 'start') { ' --gui' } else { '' }
if ($verb -eq 'start') {
    $gainText = $PositionGain.ToString([System.Globalization.CultureInfo]::InvariantCulture)
    $extra += " --world $World --gripper-collision $GripperCollision --position-gain $gainText --gripper-mode $GripperMode"
    if ($KinematicCarry) { $extra += ' --kinematic-carry' }
}
& ssh @sshOptions ros@127.0.0.1 "bash -c 'source /home/ros/.q2q/step3_env.sh && python3 /home/ros/.q2q/step3_control.py $verb$extra'"
exit $LASTEXITCODE
