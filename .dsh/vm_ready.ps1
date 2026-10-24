# vm_ready.ps1 - wake-up self-check: VM running + NAT port-forward + SSH reachable
# Usage: pwsh -File .dsh\vm_ready.ps1      (run from the workspace root)
# Exit: 0 = all good; 2 = VM not running; 3 = SSH unreachable
#
# NOTE: keep this file ASCII-only. Windows PowerShell 5.1 mis-decodes a
#       BOM-less UTF-8 script, and non-ASCII text here breaks parsing.
#       Host keys persist in the workspace, shared with step3.ps1 and sync.ps1.

$vb   = 'C:\Program Files\Oracle\VirtualBox\VBoxManage.exe'
$vm   = 'UBUNTU22.04'
$key  = "$env:USERPROFILE\.ssh\id_ed25519"
$kh   = Join-Path $PSScriptRoot 'known_hosts'
$opts = @('-i', $key, '-p', '2222', '-o', 'BatchMode=yes',
          '-o', 'StrictHostKeyChecking=accept-new', '-o', "UserKnownHostsFile=$kh",
          '-o', 'ConnectTimeout=8')

Write-Output '--- 1) VM power state ---'
if (-not (& $vb list runningvms | Select-String $vm)) {
    Write-Output 'VM_STATE=off  -> start the VM in VirtualBox and log in to the desktop'
    exit 2
}
Write-Output 'VM_STATE=on'

Write-Output '--- 2) NAT port forwarding ---'
$fwd = & $vb showvminfo $vm --machinereadable 2>$null | Select-String 'Forwarding.*2222'
if (-not $fwd) {
    & $vb controlvm $vm natpf1 'ssh,tcp,127.0.0.1,2222,,22' | Out-Null
    Write-Output 'PORTFORWARD=missing -> re-added automatically'
} else {
    Write-Output "PORTFORWARD=ok  ($fwd)"
}

Write-Output '--- 3) SSH connectivity ---'
$probe = ssh @opts ros@127.0.0.1 'hostname; id -un; uptime -p' 2>&1
$probe | ForEach-Object { Write-Output "  $_" }
if ($LASTEXITCODE -ne 0) {
    Write-Output 'SSH=fail'
    Write-Output 'Hint: check sshd and verify the VM identity before changing a host key'
    exit 3
}
Write-Output 'SSH=ok  -> ready to work'
exit 0
