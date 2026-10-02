# Run in an Administrator Windows PowerShell. Preserve the configured time source.
$principal = [Security.Principal.WindowsPrincipal]::new([Security.Principal.WindowsIdentity]::GetCurrent())
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw 'Run this script from an Administrator PowerShell to resynchronize Windows time.'
}
Start-Service W32Time
w32tm /resync /rediscover
if ($LASTEXITCODE -ne 0) { throw 'Windows time resynchronization failed; inspect w32tm /query /status.' }
w32tm /query /status
