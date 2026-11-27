[CmdletBinding()]
param(
    [string[]]$DriverArguments = @(),
    [string]$Python
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
try {
    $prefix = @()
    if ($Python) {
        $program = (Get-Command $Python -ErrorAction Stop).Source
    } else {
        $program = $null
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
    }
    if (-not $program) { throw 'Python 3.9+ is required to run the build driver.' }
    & $program @prefix -I (Join-Path $PSScriptRoot 'idf_driver.py') @DriverArguments
    exit $LASTEXITCODE
} catch {
    Write-Error $_ -ErrorAction Continue
    exit 1
}
