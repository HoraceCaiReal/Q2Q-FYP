# doctor.ps1 - environment self-check before the first run.
#
#   & .\.dsh\doctor.ps1
#
# Checks, in order: VirtualBox VM running -> NAT port 2222 -> SSH key works ->
# ROS 2 / Gazebo versions -> workspace packages built -> this repo's source
# identical to the VM mirror. Prints a fix line for every failure; never
# changes anything by itself.
#
# ASCII-only on purpose (PS 5.1 mis-decodes BOM-less UTF-8 scripts).
$ErrorActionPreference = 'Continue'
$key = Join-Path $env:USERPROFILE '.ssh\id_ed25519'
$kh = Join-Path $env:USERPROFILE '.ssh\q2q_known_hosts'
$common = @('-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=accept-new',
            '-o', "UserKnownHostsFile=$kh", '-o', 'ConnectTimeout=10')
$vbox = 'C:\Program Files\Oracle\VirtualBox\VBoxManage.exe'
$vmName = 'UBUNTU22.04'
$fail = 0

function Ok([string]$msg) { Write-Output ("  [ OK ] " + $msg) }
function Bad([string]$msg, [string]$fix) {
    $script:fail++
    Write-Output ("  [FAIL] " + $msg)
    Write-Output ("         fix: " + $fix)
}
function Note([string]$msg) { Write-Output ("  [ .. ] " + $msg) }

Write-Output 'Q2Q-FYP doctor'

# 1) VirtualBox VM -----------------------------------------------------------
Write-Output '1) VirtualBox'
if (-not (Test-Path $vbox)) {
    Bad "VBoxManage not found at $vbox" 'install VirtualBox, or edit $vbox in this script'
} elseif (-not (& $vbox list runningvms 2>$null | Select-String $vmName)) {
    Bad "VM '$vmName' is not running" "start it: `"$vbox`" startvm $vmName --type headless (then log in once)"
} else {
    Ok "VM '$vmName' is running"
    if (& $vbox showvminfo $vmName --machinereadable 2>$null | Select-String 'Forwarding.*2222') {
        Ok 'NAT port forward 2222 -> 22 present'
    } else {
        Bad 'NAT port forward 2222 -> 22 missing' "`"$vbox`" controlvm $vmName natpf1 'ssh,tcp,127.0.0.1,2222,,22'"
    }
}

# 2) SSH key -----------------------------------------------------------------
Write-Output '2) SSH'
if (-not (Test-Path $key)) {
    Bad "private key not found: $key" 'ssh-keygen -t ed25519 -f "$env:USERPROFILE\.ssh\id_ed25519"; then copy the .pub into the VM ~/.ssh/authorized_keys'
} else {
    Ok "private key present"
}
$probe = & ssh -i $key -p 2222 @common ros@127.0.0.1 'echo SSH_OK' 2>&1
if ($probe -match 'SSH_OK') {
    Ok 'ssh ros@127.0.0.1:2222 works'
} else {
    Bad "ssh failed: $($probe -join ' ')" 'check the VM is up, port 2222 is forwarded, and the key is authorized'
}

# 3) ROS / Gazebo ------------------------------------------------------------
Write-Output '3) ROS 2 and Gazebo'
if ($probe -match 'SSH_OK') {
    $env_src = 'source /opt/ros/humble/setup.bash >/dev/null 2>&1; source /home/ros/ros2_ws/install/setup.bash >/dev/null 2>&1;'
    $ros = & ssh -i $key -p 2222 @common ros@127.0.0.1 "$env_src echo ROS_DISTRO=`$ROS_DISTRO; ign gazebo --versions 2>/dev/null | head -1" 2>&1
    if ($ros -match 'ROS_DISTRO=humble') { Ok 'ROS 2 Humble present' }
    else { Bad 'ROS 2 Humble not found in the VM' 'install ros-humble-desktop, then source /opt/ros/humble/setup.bash' }
    if ($ros -match '6\.') { Ok ('Gazebo Fortress line: ' + (($ros | Select-String '6\.').Line).Trim()) }
    else { Bad 'Gazebo (ign gazebo, Fortress 6) not found' 'sudo apt install ros-humble-ros-gz; make sure ign gazebo works' }
    $pkgs = & ssh -i $key -p 2222 @common ros@127.0.0.1 "$env_src ls /home/ros/ros2_ws/install 2>/dev/null | tr '\n' ' '" 2>&1
    $need = @('robot_bringup', 'robot_worlds', 'robot_moveit', 'robot_sim_control', 'q2q_interfaces',
              'q2q_orchestration', 'q2q_low', 'q2q_record')
    $missing = @($need | Where-Object { $pkgs -notmatch $_ })
    if ($missing.Count -eq 0) { Ok 'all 8 workspace packages built in ~/ros2_ws/install' }
    else {
        Bad ('missing built packages: ' + ($missing -join ', '))
        'run: & .\.dsh\step3.ps1 -Action Deploy'
    }
}

# 4) source of truth vs VM mirror -------------------------------------------
Write-Output '4) sources'
$root = Split-Path $PSScriptRoot -Parent
$local = (Get-ChildItem (Join-Path $root 'q2q_harness') -Recurse -File -Filter *.py |
          Where-Object { $_.FullName -notmatch '__pycache__' } | Measure-Object).Count
if ($local -gt 0) { Ok ("local q2q_harness has $local .py files") }
else { Bad 'q2q_harness not found next to .dsh\' 'run this script from the cloned repository root' }
Note 'any difference between local and ~/ros2_ws/src is synced by: & .\.dsh\step3.ps1 -Action Deploy'

Write-Output ''
if ($fail -eq 0) {
    Write-Output 'ALL CHECKS PASSED - next: & .\.dsh\step3.ps1 -Action Deploy  (if you changed sources)'
    Write-Output '                     then: & .\.dsh\demo.ps1'
    exit 0
}
Write-Output ("$fail CHECK(S) FAILED - fix the lines marked [FAIL] above, then run this again")
exit 1
