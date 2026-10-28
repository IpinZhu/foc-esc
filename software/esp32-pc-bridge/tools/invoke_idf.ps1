[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$ProjectPath,
    [Parameter(Mandatory = $true)]
    [string]$IdfPath,
    [string]$IdfToolsPath,
    [ValidateSet('receiver', 'mock', 'uart', 'pc', 'motor')]
    [string]$Variant = 'receiver',
    [ValidateSet('build', 'flash', 'menuconfig')]
    [string]$Action = 'build',
    [string]$Port
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

try {
    if ($Action -eq 'flash' -and [string]::IsNullOrWhiteSpace($Port)) {
        throw 'Flashing requires an explicit -Port.'
    }
    $env:IDF_PATH = $IdfPath
    if ($IdfToolsPath) { $env:IDF_TOOLS_PATH = $IdfToolsPath }
    $global:LASTEXITCODE = 0
    . (Join-Path $IdfPath 'export.ps1')
    if ($LASTEXITCODE -ne 0) {
        throw "SDK environment initialization failed with exit code $LASTEXITCODE"
    }
    if (-not $env:IDF_PYTHON_ENV_PATH) {
        throw 'SDK Python environment is missing. Run setup.ps1 or install this SDK tools first.'
    }
    $python = Join-Path $env:IDF_PYTHON_ENV_PATH 'Scripts/python.exe'
    if (-not (Test-Path $python)) {
        throw "SDK Python does not exist: $python"
    }
    $defaults = @((Join-Path $ProjectPath 'sdkconfig.defaults'))
    switch ($Variant) {
        'mock' { $defaults += Join-Path $ProjectPath 'sdkconfig.mock.defaults' }
        'uart' { $defaults += Join-Path $ProjectPath 'sdkconfig.uart.defaults' }
        'pc' { $defaults += Join-Path $ProjectPath 'sdkconfig.pc.defaults' }
        'motor' { $defaults += Join-Path $ProjectPath 'sdkconfig.motor.defaults' }
    }
    $build = Join-Path $ProjectPath "build-$Variant"
    $config = Join-Path $ProjectPath "sdkconfig.$Variant"
    $arguments = @((Join-Path $IdfPath 'tools/idf.py'), '-C', $ProjectPath, '-B', $build,
        '-DIDF_TARGET=esp32c3', "-DSDKCONFIG=$config",
        "-DSDKCONFIG_DEFAULTS=$($defaults -join ';')")
    if ($Port) { $arguments += @('-p', $Port) }
    if ($Action -eq 'build') { $arguments += 'build' } else { $arguments += $Action }
    Write-Output "ESP-IDF: $IdfPath"
    Write-Output "Variant: $Variant; action: $Action; build directory: $build"
    & $python @arguments
    if ($LASTEXITCODE -ne 0) {
        throw "idf.py failed with exit code $LASTEXITCODE"
    }
    if ($Action -eq 'build') {
        $binary = Join-Path $build 'foc_pc_bridge.bin'
        if (-not (Test-Path $binary)) {
            throw "Build returned success but firmware was not generated: $binary"
        }
        Write-Output "Firmware: $binary"
    }
} catch {
    Write-Error $_ -ErrorAction Continue
    exit 1
}
