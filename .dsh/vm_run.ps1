# vm_run.ps1 - run a local .sh script inside the VM through SSH stdin.
#
# Usage:  powershell -NoProfile -ExecutionPolicy Bypass -File .dsh\vm_run.ps1 -Script .dsh\fan_test_basic.sh
#
# Why stdin instead of "ssh host $script": Windows PowerShell 5.1 re-quotes
# arguments passed to native commands and silently strips embedded double
# quotes, which breaks any non-trivial remote shell script. Feeding the file
# to `bash -s` over stdin keeps the script byte-for-byte intact.
#
# ASCII-only on purpose (PS 5.1 mis-decodes BOM-less UTF-8 scripts).

param(
    [Parameter(Mandatory = $true)][string]$Script,
    [string]$ScriptArgs = '',
    [int]$TimeoutSec = 0
)

if (-not (Test-Path $Script)) { Write-Output "SCRIPT_NOT_FOUND: $Script"; exit 4 }

$key = "$env:USERPROFILE\.ssh\id_ed25519"
$kh  = Join-Path $env:TEMP 'dsh_known_hosts'
$opts = @('-i', $key, '-p', '2222', '-o', 'BatchMode=yes',
          '-o', 'StrictHostKeyChecking=accept-new', '-o', "UserKnownHostsFile=$kh",
          '-o', 'ConnectTimeout=8')

# Send the script as base64 on the command line and let the VM decode it.
# Reason: piping through PowerShell appends a CRLF (bash then trips over a
# stray "\r"), and passing the text as an argument loses embedded quotes.
# base64 is pure ASCII, so neither problem can happen.
$body = Get-Content -Raw -Encoding UTF8 $Script
$b64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($body))
ssh @opts ros@127.0.0.1 "echo $b64 | base64 -d | bash -s -- $ScriptArgs"
exit $LASTEXITCODE
