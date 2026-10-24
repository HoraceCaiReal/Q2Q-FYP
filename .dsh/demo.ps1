# demo.ps1 - one-command defense demo.
#
#   & .\.dsh\demo.ps1
#
# What it does:
#   1) VM: stop old stack, clean residual, start headless (50 mm + kinematic carry)
#   2) VM: wait until 4 controllers + nav action are actually up
#   3) VM: attach a Gazebo GUI *client* (ogre engine) and RViz to the running server
#   4) VM: run one full chain (one START)
#   5) host: screenshot on every state change into docs/evaluations/artifacts/<date>/demo-<stamp>/
#
# Exit 0 only when the chain reached DONE. LAUNCHED is not a PASS.
# ASCII-only on purpose (PS 5.1 mis-decodes BOM-less UTF-8 scripts).
param(
    [int]$TimeoutSec = 660,
    [int]$PollSec = 10
)
$ErrorActionPreference = 'Stop'
$key = Join-Path $env:USERPROFILE '.ssh\id_ed25519'
$known = Join-Path $PSScriptRoot 'known_hosts'
$common = @('-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=accept-new',
            '-o', "UserKnownHostsFile=$known", '-o', 'ConnectTimeout=15')
$vbox = 'C:\Program Files\Oracle\VirtualBox\VBoxManage.exe'
$vm = 'UBUNTU22.04'
$out = Join-Path (Split-Path $PSScriptRoot -Parent) `
    ("docs\evaluations\artifacts\{0}\demo-{1}" -f (Get-Date -Format 'yyyy-MM-dd'), (Get-Date -Format 'HHmmss'))
New-Item -ItemType Directory -Force -Path $out | Out-Null

function Remote([string]$script) {
    & ssh -i $key -p 2222 @common ros@127.0.0.1 $script
}

foreach ($name in @('demo_run.sh', 'launch_trial.sh', 'step7_trial.py')) {
    & scp -i $key -P 2222 @common (Join-Path $PSScriptRoot $name) "ros@127.0.0.1:/home/ros/.q2q/$name"
    if ($LASTEXITCODE -ne 0) { Write-Output "SCP_FAIL $name"; exit 5 }
}

Write-Output '--- bringing up stack + GUI client + chain ---'
$bringup = Remote 'bash /home/ros/.q2q/demo_run.sh'
$bringup | ForEach-Object { Write-Output "  $_" }
if (-not ($bringup | Where-Object { $_ -match '^DEMO_LAUNCHED' })) {
    Write-Output 'DEMO_BRINGUP_FAILED'
    exit 6
}
$run = (($bringup | Where-Object { $_ -match '^RUN=' } | Select-Object -Last 1) -replace '^RUN=', '').Trim()
$log = (($bringup | Where-Object { $_ -match '^TRIAL_LOG=' } | Select-Object -Last 1) -replace '^TRIAL_LOG=', '').Trim()
Write-Output "RUN=$run"
Write-Output "TRIAL_LOG=$log"

$seen = @{}
$final = ''
$deadline = (Get-Date).AddSeconds($TimeoutSec)
while ((Get-Date) -lt $deadline) {
    Start-Sleep -Seconds $PollSec
    $line = (Remote "grep -E ^TRANS $log | tail -1") -join ''
    $line = $line.Trim()
    if (-not $line) { continue }
    $parts = $line -split '\s+'
    if ($parts.Count -lt 3) { continue }
    $state = $parts[2]
    if (-not $seen.ContainsKey($state)) {
        $seen[$state] = $true
        $png = Join-Path $out ("{0:D2}_{1}.png" -f $seen.Count, $state)
        & $vbox controlvm $vm screenshotpng $png 2>$null | Out-Null
        Write-Output ("  STATE {0,-12} -> {1}" -f $state, (Split-Path $png -Leaf))
    }
    if ($state -eq 'DONE') { $final = 'DONE'; break }
    if ($state -eq 'FAILED') { $final = 'FAILED'; break }
}

Remote "grep -E ^TRANS $log | tail -14" | ForEach-Object { Write-Output "  $_" }
Remote "grep -E ^SUMMARY $log | tail -1" | ForEach-Object { Write-Output "  $_" }
Write-Output "SCREENSHOTS=$out"
if ($final -ne 'DONE') { Write-Output 'DEMO_RESULT=NOT_DONE'; exit 1 }
Write-Output 'DEMO_RESULT=DONE'
exit 0
