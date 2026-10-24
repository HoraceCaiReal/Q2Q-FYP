# sync.ps1 - push Windows-side ROS2 packages into the VM, syntax-check, rebuild.
#
# Usage (from anywhere):
#   pwsh -File .dsh\sync.ps1                      # push every package under q2q_harness\sim
#   pwsh -File .dsh\sync.ps1 -Pkg robot_bringup   # push one package
#   pwsh -File .dsh\sync.ps1 -Pkg robot_bringup -NoBuild
#
# Layout (Windows side = single source of truth, mirrors VM ~/ros2_ws/src):
#   E:\<workspace>\q2q_harness\sim\<Pkg>\    <- one directory per ROS2 package (has package.xml)
#
# Exit codes: 2 = VM off, 4 = local package missing, 5 = scp failed,
#             6 = syntax error (build skipped), 7 = build failed, 0 = ok
#
# NOTE: ASCII-only on purpose. Windows PowerShell 5.1 mis-decodes a BOM-less
#       UTF-8 script, and non-ASCII here breaks parsing.

param(
    [string]$Pkg = '',
    [switch]$NoBuild
)

$vb   = 'C:\Program Files\Oracle\VirtualBox\VBoxManage.exe'
$vm   = 'UBUNTU22.04'
$key  = "$env:USERPROFILE\.ssh\id_ed25519"
$kh   = Join-Path $PSScriptRoot 'known_hosts'
$common = @('-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=accept-new',
            '-o', "UserKnownHostsFile=$kh", '-o', 'ConnectTimeout=8')
$sshOpts = @('-i', $key, '-p', '2222') + $common
$scpOpts = @('-i', $key, '-P', '2222') + $common

$root   = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
# ROS2 packages may live under any layer (project convention). 2026-10-24: mid/ and record/
# got their first packages, so scanning only sim/ would silently skip them.
$srcRoots = @('sim', 'mid', 'record', 'low', 'top') |
    ForEach-Object { Join-Path $root ('q2q_harness\' + $_) } |
    Where-Object { Test-Path $_ }
if (-not $srcRoots) { Write-Output "SRC_ROOT_NOT_FOUND under q2q_harness"; exit 4 }

# --- 0) pick packages -------------------------------------------------------
if ($Pkg) {
    $pkgs = @($srcRoots | ForEach-Object { Get-Item (Join-Path $_ $Pkg) -ErrorAction SilentlyContinue } |
              Where-Object { $_ -and (Test-Path (Join-Path $_.FullName 'package.xml')) })
    if (-not $pkgs) {
        Write-Output "LOCAL_NOT_FOUND: $Pkg (needs package.xml under sim/mid/record/low/top)"; exit 4
    }
} else {
    $pkgs = @($srcRoots | ForEach-Object {
                  Get-ChildItem -Path $_ -Directory -ErrorAction SilentlyContinue |
                  Where-Object { Test-Path (Join-Path $_.FullName 'package.xml') } })
    if ($pkgs.Count -eq 0) { Write-Output "NO_PACKAGE under q2q_harness/{sim,mid,record,low,top}"; exit 4 }
}
$names = @($pkgs | ForEach-Object { $_.Name })
Write-Output ("PACKAGES=" + ($names -join ','))

# --- 0.1) VM up + tunnel present -------------------------------------------
if (-not (& $vb list runningvms | Select-String $vm)) {
    Write-Output 'VM_STATE=off  -> start the VM and log in to the desktop'
    exit 2
}
if (-not (& $vb showvminfo $vm --machinereadable 2>$null | Select-String 'Forwarding.*2222')) {
    & $vb controlvm $vm natpf1 'ssh,tcp,127.0.0.1,2222,,22' | Out-Null
    Write-Output 'PORTFORWARD=re-added'
}

# --- 0.5) local syntax pre-check -------------------------------------------
$checker = Join-Path $PSScriptRoot '_tools\pycheck.py'
$localPy = $pkgs | ForEach-Object {
    Get-ChildItem -Path $_.FullName -Recurse -Filter *.py -File -ErrorAction SilentlyContinue
} | Where-Object { $_.FullName -notmatch '__pycache__' }
$py = Get-Command python -ErrorAction SilentlyContinue
if ($py -and (Test-Path $checker) -and $localPy) {
    Write-Output '--- local syntax pre-check ---'
    $paths = @($localPy | ForEach-Object { $_.FullName })
    $out = & $py.Source $checker @paths 2>&1
    $out | ForEach-Object { Write-Output "  $_" }
    if ($LASTEXITCODE -ne 0) { Write-Output 'PUSH=skipped (fix the syntax error above)'; exit 6 }
} else {
    Write-Output 'LOCAL_SYNTAX=skipped (no local python)'
}

# --- 1) push ---------------------------------------------------------------
$stageRoot = Join-Path $env:TEMP 'q2q_sync'
$stageFull = [System.IO.Path]::GetFullPath($stageRoot)
$expectedStage = [System.IO.Path]::GetFullPath((Join-Path $env:TEMP 'q2q_sync'))
if ($stageFull -ne $expectedStage -or (Split-Path $stageFull -Leaf) -ne 'q2q_sync') {
    throw 'Refusing to clear unexpected staging path'
}
Remove-Item -LiteralPath $stageFull -Recurse -Force -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Path $stageRoot -Force | Out-Null
foreach ($item in $pkgs) {
    Write-Output ("--- push: " + $item.Name + " ---")
    $stage = Join-Path $stageRoot $item.Name
    & robocopy $item.FullName $stage /E /XD __pycache__ build install log /XF *.pyc /NFL /NDL /NJH /NJS /NP | Out-Null
    if ($LASTEXITCODE -ge 8) { Write-Output 'STAGE=fail'; exit 5 }
    scp -r @scpOpts $stage "ros@127.0.0.1:~/ros2_ws/src/" 2>&1 | ForEach-Object { Write-Output "  $_" }
    if ($LASTEXITCODE -ne 0) { Write-Output 'SCP=fail'; exit 5 }
}

# --- 1.5) make scripts/*.py executable -------------------------------------
# Windows has no exec bit; scp lands 644 and ros2 launch then fails with
# "executable 'x.py' not found on the libexec directory".
$chmodCmd = (($names | ForEach-Object { "find ~/ros2_ws/src/$_ -path '*/scripts/*.py' -exec chmod +x {} +" }) -join ' ; ')
$chmodOut = ssh @sshOpts ros@127.0.0.1 $chmodCmd 2>&1
if ($LASTEXITCODE -ne 0) { Write-Output ('CHMOD=fail ' + ($chmodOut -join ' ')); exit 5 }

# --- 2) remote syntax check (python files that exist) ----------------------
# Sent as base64 so PowerShell never re-quotes the inner python -c snippet.
$chkLines = @(
    'cd ~/ros2_ws/src || exit 7',
    'fail=0',
    ('for d in ' + ($names -join ' ') + '; do'),
    '  while IFS= read -r f; do',
    '    python3 -c ''import sys;compile(open(sys.argv[1]).read(),sys.argv[1],"exec")'' "$f" || fail=1',
    '  done < <(find "$d" -name "*.py" -not -path "*__pycache__*")',
    'done',
    'if [ $fail -eq 0 ]; then echo SYNTAX_OK; else echo SYNTAX_FAIL; exit 1; fi'
)
$chkScript = $chkLines -join "`n"
$chkB64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($chkScript))
$res = ssh @sshOpts ros@127.0.0.1 ("echo $chkB64 | base64 -d | bash")
$checkExit = $LASTEXITCODE
$res | ForEach-Object { Write-Output "  $_" }
if ($checkExit -ne 0 -or $res -match 'SYNTAX_FAIL') { Write-Output 'SYNTAX=FAIL -> build skipped'; exit 6 }

if ($NoBuild) { Write-Output 'BUILD=skipped (-NoBuild)'; exit 0 }

# --- 3) build (symlink-install: later config/launch edits need no rebuild) --
Write-Output '--- colcon build --symlink-install ---'
$sel = ($names -join ' ')
$bld = 'set -o pipefail; source /opt/ros/humble/setup.bash; cd ~/ros2_ws && colcon build --symlink-install --packages-select ' +
       $sel + ' 2>&1 | tail -25'
$out = ssh @sshOpts ros@127.0.0.1 $bld
$buildExit = $LASTEXITCODE
$out | ForEach-Object { Write-Output "  $_" }
if ($buildExit -ne 0) { Write-Output 'BUILD=FAIL'; exit 7 }
Write-Output 'BUILD=ok'
exit 0
