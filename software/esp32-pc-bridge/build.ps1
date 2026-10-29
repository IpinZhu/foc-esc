[CmdletBinding()]
param(
    [ValidateSet('receiver', 'mock', 'uart', 'pc', 'motor')]
    [string]$Variant = 'receiver',
    [ValidateSet('build', 'flash', 'menuconfig', 'test')]
    [string]$Action = 'build',
    [string]$Port,
    [string]$IdfPath,
    [string]$IdfToolsPath,
    [switch]$Help
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

if ($Help) {
    Write-Output @'
Usage from ordinary PowerShell (no ESP-IDF terminal required):
  .\setup.ps1                                  Install SDK once
  .\build.ps1                                  Build native-USB receiver
  .\build.ps1 -Variant mock                    Build fake FOC transmitter
  .\build.ps1 -Variant pc                      Build v2 PC duplex bridge (requires local keys)
  .\build.ps1 -Variant motor                   Build v2 motor UART/radio bridge (requires local keys)
  .\build.ps1 -Variant pc -Action menuconfig    Configure PC peer MAC, channel, PMK and LMK
  .\build.ps1 -Variant motor -Action menuconfig Configure motor peer MAC and matching keys
  .\build.ps1 -Action test                     Run host tests, no SDK needed
  .\build.ps1 -Action menuconfig               Configure receiver
  .\build.ps1 -Action flash -Port COM5          Build and flash receiver
  .\build.ps1 -IdfPath C:\Espressif\esp-idf      Use an existing SDK
Optional -IdfToolsPath selects that SDK's tool cache. Nothing is flashed by default.
Duplex peers must use each other's station MAC, the same channel and locally supplied
16-byte ASCII PMK/LMK. No default control key exists; missing keys fail closed.
Motor UART1 defaults: GPIO4 TX -> STM32 PB11 RX, GPIO5 RX <- STM32 PB10 TX,
115200 8N1, 3.3 V logic, common ground. Confirm the actual board pins before flashing.
'@
    exit 0
}

function Invoke-Checked {
    param([string]$Program, [string[]]$Arguments)
    & $Program @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "$Program failed with exit code $LASTEXITCODE"
    }
}

try {
    if ($Action -eq 'flash' -and [string]::IsNullOrWhiteSpace($Port)) {
        throw 'Flashing requires an explicit -Port COMx. No device was changed.'
    }
    if ($Action -eq 'test') {
        $cmake = (Get-Command cmake -ErrorAction Stop).Source
        $ctest = (Get-Command ctest -ErrorAction Stop).Source
        $hostBuild = Join-Path $PSScriptRoot 'build-host-script'
        $arguments = @('-S', (Join-Path $PSScriptRoot 'test'), '-B', $hostBuild)
        if (Get-Command ninja -ErrorAction SilentlyContinue) {
            $arguments += @('-G', 'Ninja')
        } elseif (Get-Command mingw32-make -ErrorAction SilentlyContinue) {
            $arguments += @('-G', 'MinGW Makefiles')
            $compiler = Get-Command g++ -ErrorAction Stop
            $arguments += "-DCMAKE_CXX_COMPILER=$($compiler.Source)"
        }
        Invoke-Checked $cmake $arguments
        Invoke-Checked $cmake @('--build', $hostBuild)
        Invoke-Checked $ctest @('--test-dir', $hostBuild, '--output-on-failure')
        $python = Join-Path $PSScriptRoot '.venv/Scripts/python.exe'
        if (-not (Test-Path $python)) {
            $basePython = (Get-Command python -ErrorAction Stop).Source
            Invoke-Checked $basePython @('-m', 'venv', (Join-Path $PSScriptRoot '.venv'))
        }
        Invoke-Checked $python @('-I', (Join-Path $PSScriptRoot 'tools/test_verify_usb.py'))
        Invoke-Checked $python @('-I', (Join-Path $PSScriptRoot 'tools/test_build_scripts.py'))
        exit 0
    }

    $localSdk = Join-Path $PSScriptRoot '.tools/esp-idf-v5.5.1'
    if ($IdfPath) {
        $sdk = (Resolve-Path -LiteralPath $IdfPath -ErrorAction Stop).Path
    } else {
        $candidates = @($localSdk, $env:IDF_PATH, 'C:/Espressif/esp-idf',
                        (Join-Path $HOME 'esp/esp-idf'))
        if (Test-Path 'C:/Espressif/frameworks') {
            $candidates += @(Get-ChildItem 'C:/Espressif/frameworks' -Directory -Filter 'esp-idf*' |
                Sort-Object Name -Descending | Select-Object -ExpandProperty FullName)
        }
        $sdk = $null
        foreach ($candidate in $candidates) {
            if ($candidate -and (Test-Path (Join-Path $candidate 'tools/idf.py')) -and
                (Test-Path (Join-Path $candidate 'export.ps1'))) {
                $sdk = (Resolve-Path -LiteralPath $candidate).Path
                break
            }
        }
    }
    if (-not $sdk -or -not (Test-Path (Join-Path $sdk 'tools/idf.py')) -or
        -not (Test-Path (Join-Path $sdk 'export.ps1'))) {
        throw 'ESP-IDF SDK not found. Run .\setup.ps1, or supply -IdfPath C:\path\to\esp-idf.'
    }
    if (-not $IdfToolsPath -and $sdk -eq $localSdk) {
        $IdfToolsPath = Join-Path $PSScriptRoot '.tools/espressif'
    }
    $shell = (Get-Process -Id $PID).Path
    $arguments = @('-NoProfile', '-File', (Join-Path $PSScriptRoot 'tools/invoke_idf.ps1'),
        '-ProjectPath', $PSScriptRoot, '-IdfPath', $sdk, '-Variant', $Variant, '-Action', $Action)
    if ($IdfToolsPath) { $arguments += @('-IdfToolsPath', $IdfToolsPath) }
    if ($Port) { $arguments += @('-Port', $Port) }
    Invoke-Checked $shell $arguments
} catch {
    Write-Error $_ -ErrorAction Continue
    exit 1
}
