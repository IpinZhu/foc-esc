[CmdletBinding()]
param(
    [string]$Python = 'python',
    [switch]$Help
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

if ($Help) {
    Write-Output @'
Usage: .\setup.ps1 [-Python C:\path\to\python.exe]
Installs official ESP-IDF v5.5.1 and ESP32-C3 tools under .tools in this project.
Requires Git, Python 3.9-3.13 and network access. Does not change global PATH.
After installation: .\build.ps1
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

$version = 'v5.5.1'
$sdk = Join-Path $PSScriptRoot ".tools/esp-idf-$version"
$tools = Join-Path $PSScriptRoot '.tools/espressif'
$oldTools = $env:IDF_TOOLS_PATH
$oldIdf = $env:IDF_PATH

try {
    $git = (Get-Command git -ErrorAction Stop).Source
    $pythonExe = (Get-Command $Python -ErrorAction Stop).Source
    New-Item -ItemType Directory -Force (Split-Path $sdk) | Out-Null
    if (-not (Test-Path $sdk)) {
        Write-Output "Downloading official ESP-IDF $version to $sdk"
        Invoke-Checked $git @('-c', 'http.version=HTTP/1.1', 'clone', '--depth', '1', '--branch', $version,
            '--recurse-submodules', '--shallow-submodules',
            'https://github.com/espressif/esp-idf.git', $sdk)
    } else {
        if (-not (Test-Path (Join-Path $sdk '.git'))) {
            throw "Refusing to overwrite existing non-Git directory: $sdk"
        }
        $origin = & $git -C $sdk remote get-url origin
        if ($LASTEXITCODE -ne 0 -or $origin.TrimEnd('/') -ne 'https://github.com/espressif/esp-idf.git') {
            throw "Existing SDK does not have the expected official Git origin: $sdk"
        }
        $tag = & $git -C $sdk describe --tags --exact-match HEAD
        if ($LASTEXITCODE -ne 0 -or $tag -ne $version) {
            throw "Existing SDK is not $version; it will not be changed: $sdk"
        }
        Invoke-Checked $git @('-c', 'http.version=HTTP/1.1', '-C', $sdk, 'submodule', 'update', '--init', '--recursive', '--depth', '1')
    }
    $idfTools = Join-Path $sdk 'tools/idf_tools.py'
    if (-not (Test-Path $idfTools)) {
        throw "Incomplete SDK checkout: $idfTools"
    }
    $env:IDF_PATH = $sdk
    $env:IDF_TOOLS_PATH = $tools
    Write-Output "Installing ESP32-C3 tools into $tools"
    Invoke-Checked $pythonExe @($idfTools, 'install', '--targets=esp32c3')
    Invoke-Checked $pythonExe @($idfTools, 'install-python-env')
    Write-Output 'SDK ready. Run .\build.ps1 or .\build.ps1 -Variant mock'
} catch {
    Write-Error $_ -ErrorAction Continue
    exit 1
} finally {
    $env:IDF_TOOLS_PATH = $oldTools
    $env:IDF_PATH = $oldIdf
}
