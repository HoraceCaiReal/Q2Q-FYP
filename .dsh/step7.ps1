# step7.ps1 - run one full step-7 chain trial on an already Start-ed stack.
#
# Usage:  & .\.dsh\step7.ps1                 # arrive_x 0.80 (default)
#         & .\.dsh\step7.ps1 -ArriveX 0.78
#
# Prereq: .dsh\step3.ps1 -Action Start -World q2q_warehouse_grasp50
# ASCII-only on purpose (PS 5.1 mis-decodes BOM-less UTF-8 scripts).
param([double]$ArriveX = 0.80, [double]$TrialTimeout = 1500)
$ErrorActionPreference = 'Stop'
$key = Join-Path $env:USERPROFILE '.ssh\id_ed25519'
$known = Join-Path $PSScriptRoot 'known_hosts'
$common = @('-o','BatchMode=yes','-o','StrictHostKeyChecking=accept-new','-o',"UserKnownHostsFile=$known")
& scp -i $key -P 2222 @common (Join-Path $PSScriptRoot 'step7_trial.py') 'ros@127.0.0.1:/home/ros/.q2q/step7_trial.py'
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& scp -i $key -P 2222 @common (Join-Path $PSScriptRoot 'launch_trial.sh') 'ros@127.0.0.1:/home/ros/.q2q/launch_trial.sh'
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$x = $ArriveX.ToString([System.Globalization.CultureInfo]::InvariantCulture)
$budget = $TrialTimeout.ToString([System.Globalization.CultureInfo]::InvariantCulture)
& ssh -i $key -p 2222 @common ros@127.0.0.1 "bash /home/ros/.q2q/launch_trial.sh --arrive-x $x --trial-timeout $budget"
exit $LASTEXITCODE
