[CmdletBinding()]
param(
    [ValidateSet('encoder', 'hall', 'sensorless')][string]$Sensor = 'encoder',
    [ValidateSet('build', 'test', 'clippy', 'host-clippy', 'fmt', 'flash')][string]$Action = 'build',
    [string]$Probe,
    [int]$Speed,
    [switch]$ConnectUnderReset,
    [switch]$Help
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
try {
    $arguments = @('--sensor', $Sensor, '--action', $Action)
    if ($Probe) { $arguments += @('--probe', $Probe) }
    if ($PSBoundParameters.ContainsKey('Speed')) { $arguments += @('--speed', "$Speed") }
    if ($ConnectUnderReset) { $arguments += '--connect-under-reset' }
    if ($Help) { $arguments = @('--help') }
    $program = $null
    $prefix = @()
    foreach ($name in @('python3', 'python', 'py')) {
        $candidate = Get-Command $name -ErrorAction SilentlyContinue
        if ($candidate) {
            & $candidate.Source --version *> $null
            if ($LASTEXITCODE -eq 0) {
                $program = $candidate.Source
                if ($name -eq 'py') { $prefix = @('-3') }
                break
            }
        }
    }
    if (-not $program) { throw 'Python 3.9+ is required to run the build driver.' }
    & $program @prefix -I (Join-Path $PSScriptRoot 'tools/firmware.py') @arguments
    exit $LASTEXITCODE
} catch {
    Write-Error $_ -ErrorAction Continue
    exit 1
}
