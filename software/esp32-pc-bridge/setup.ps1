[CmdletBinding()]
param(
    [string]$Python,
    [switch]$Help
)
$arguments = @('--action', 'setup')
if ($Python) { $arguments += @('--python', $Python) }
if ($Help) { $arguments = @('--help') }
& (Join-Path $PSScriptRoot 'tools/invoke_driver.ps1') -DriverArguments $arguments -Python $Python
exit $LASTEXITCODE
