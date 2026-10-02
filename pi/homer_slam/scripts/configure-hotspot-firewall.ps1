param([string]$Subnet = '192.168.137.0/24', [switch]$Remove)
# Scoped DDS UDP rules only; no global firewall disable or default-action change.
$principal = [Security.Principal.WindowsPrincipal]::new([Security.Principal.WindowsIdentity]::GetCurrent())
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw 'Run from Administrator PowerShell to configure hotspot DDS firewall rules.'
}
$name = 'Homer-Hotspot-DDS'
$vm = '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'
if ($Remove) {
    Get-NetFirewallRule -Name $name -ErrorAction SilentlyContinue | Remove-NetFirewallRule
    Get-NetFirewallHyperVRule -Name $name -ErrorAction SilentlyContinue | Remove-NetFirewallHyperVRule
    return
}
if (-not (Get-NetFirewallRule -Name $name -ErrorAction SilentlyContinue)) {
    New-NetFirewallRule -Name $name -DisplayName 'Homer hotspot DDS UDP' -Direction Inbound -Action Allow -Protocol UDP -LocalPort 7400-7800 -RemoteAddress $Subnet -Profile Any | Out-Null
}
if (-not (Get-NetFirewallHyperVRule -Name $name -ErrorAction SilentlyContinue)) {
    New-NetFirewallHyperVRule -Name $name -DisplayName 'Homer hotspot DDS UDP' -Direction Inbound -Action Allow -VMCreatorId $vm -Protocol UDP -LocalPorts '7400-7800' -RemoteAddresses $Subnet | Out-Null
}
Write-Output "DDS inbound allowed from $Subnet on ports 7400-7800; domain 0, up to 100 participants."
