param([string]$Uv4 = 'D:\Keil_v5\UV4\UV4.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$projectFile = Join-Path $projectRoot 'MDK-ARM\stm32motor.uvprojx'
$buildLog = Join-Path $PSScriptRoot 'keil_build.log'
if (-not (Test-Path -LiteralPath $Uv4)) { throw "UV4 not found: $Uv4" }
$arguments = '-r "{0}" -t "stm32motor" -o "{1}"' -f $projectFile, $buildLog
$buildProcess = Start-Process -FilePath $Uv4 -ArgumentList $arguments -WorkingDirectory $projectRoot -WindowStyle Hidden -PassThru -Wait
Get-Content -LiteralPath $buildLog
if ($buildProcess.ExitCode -ne 0) { throw "UV4 build exit code: $($buildProcess.ExitCode)" }
if (-not (Select-String -LiteralPath $buildLog -SimpleMatch '0 Error(s), 0 Warning(s).')) {
    throw 'Build did not report zero errors and warnings.'
}
