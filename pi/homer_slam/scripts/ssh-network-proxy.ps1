param([Parameter(Mandatory=$true)][string]$Address)
$client = [System.Net.Sockets.TcpClient]::new()
try {
    $connect = $client.ConnectAsync($Address, 22)
    if (-not $connect.Wait(8000)) { throw 'SSH peer connection timed out' }
    $network = $client.GetStream()
    $toPi = [Console]::OpenStandardInput().CopyToAsync($network)
    $fromPi = $network.CopyToAsync([Console]::OpenStandardOutput())
    [System.Threading.Tasks.Task]::WaitAny([System.Threading.Tasks.Task[]]@($toPi, $fromPi)) | Out-Null
} finally { $client.Dispose() }
