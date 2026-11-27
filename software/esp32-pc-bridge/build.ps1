[CmdletBinding()]
param(
    [ValidateSet('receiver', 'mock', 'uart', 'pc', 'motor')]
    [string]$Variant = 'receiver',
    [ValidateSet('build', 'flash', 'menuconfig', 'test', 'version')]
    [string]$Action = 'build',
    [string]$Port,
    [string]$IdfPath,
    [string]$IdfToolsPath,
    [string]$PythonEnv,
    [switch]$Help
)
$arguments = @('--variant', $Variant, '--action', $Action)
if ($Port) { $arguments += @('--port', $Port) }
if ($IdfPath) { $arguments += @('--idf-path', $IdfPath) }
if ($IdfToolsPath) { $arguments += @('--idf-tools-path', $IdfToolsPath) }
if ($PythonEnv) { $arguments += @('--python-env', $PythonEnv) }
if ($Help) { $arguments = @('--help') }
& (Join-Path $PSScriptRoot 'tools/invoke_driver.ps1') -DriverArguments $arguments
exit $LASTEXITCODE
