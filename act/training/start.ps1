$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot
$tools = Join-Path $PSScriptRoot '.tools'
New-Item -ItemType Directory -Force $tools | Out-Null
$uv = Join-Path $tools 'uv.exe'
if (-not (Test-Path $uv)) {
    Write-Host 'Downloading the local Python installer...'
    $download = Join-Path $tools 'uv.zip'
    Invoke-WebRequest 'https://github.com/astral-sh/uv/releases/latest/download/uv-x86_64-pc-windows-msvc.zip' -OutFile $download
    Expand-Archive -Force $download $tools
    Remove-Item $download
}
$env:UV_PYTHON_INSTALL_DIR = Join-Path $tools 'python'
$env:UV_PYTHON_PREFERENCE = 'only-managed'
$env:UV_PYTHON_NO_REGISTRY = '1'
$env:UV_CACHE_DIR = Join-Path $PSScriptRoot '.cache\uv'
$env:PIP_CACHE_DIR = Join-Path $PSScriptRoot '.cache\pip'
$env:TORCH_HOME = Join-Path $PSScriptRoot '.cache\torch'
$env:HF_HOME = Join-Path $PSScriptRoot '.cache\huggingface'
Write-Host 'Preparing local Python 3.11, then starting training...'
& $uv run --no-project --python 3.11 run.py @args
exit $LASTEXITCODE
