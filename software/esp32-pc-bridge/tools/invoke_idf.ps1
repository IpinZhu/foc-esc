# Compatibility entrypoint for callers of the original Windows IDF helper.
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$ProjectPath,
    [Parameter(Mandatory = $true)][string]$IdfPath,
    [string]$IdfToolsPath,
    [ValidateSet('receiver', 'mock', 'uart', 'pc', 'motor')][string]$Variant = 'receiver',
    [ValidateSet('build', 'flash', 'menuconfig')][string]$Action = 'build',
    [string]$Port
)
$arguments = @('--project', $ProjectPath, '--idf-path', $IdfPath,
               '--variant', $Variant, '--action', $Action)
if ($IdfToolsPath) { $arguments += @('--idf-tools-path', $IdfToolsPath) }
if ($Port) { $arguments += @('--port', $Port) }
& (Join-Path $PSScriptRoot 'invoke_driver.ps1') -DriverArguments $arguments
exit $LASTEXITCODE
