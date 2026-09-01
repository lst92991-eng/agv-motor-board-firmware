[CmdletBinding()]
param()

$vcvarsCandidates = @(
  'C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat',
  'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
)
$vcvarsPath = $vcvarsCandidates | Where-Object {
  Test-Path -LiteralPath $_
} | Select-Object -First 1
if($null -eq $vcvarsPath) {
  throw 'Visual Studio C compiler environment was not found'
}

$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$sourceIncludePaths = @(
  (Join-Path $repoRoot 'src\app'),
  (Join-Path $repoRoot 'src\services'),
  (Join-Path $repoRoot 'src\drivers'),
  (Join-Path $repoRoot 'src\config')
)
$sourceIncludeArguments = ($sourceIncludePaths | ForEach-Object {
  '/I "{0}"' -f $_
}) -join ' '
$testIncludePath = Join-Path $repoRoot 'tests'
$boardIncludePath = Join-Path $repoRoot 'at32f45x_board'
$contractIncludePath = (Resolve-Path -LiteralPath `
    (Join-Path $repoRoot '..\agv-motion-contract\include')).Path

function Invoke-CHostTest {
  param(
    [Parameter(Mandatory = $true)]
    [string]$Name,
    [Parameter(Mandatory = $true)]
    [string]$TestRelativePath,
    [string[]]$ModuleRelativePath = @()
  )

  $testSource = Join-Path $repoRoot $TestRelativePath
  $moduleSources = $ModuleRelativePath | ForEach-Object {
    '"{0}"' -f (Join-Path $repoRoot $_)
  }
  $moduleArguments = $moduleSources -join ' '
  $testExecutable = Join-Path ([IO.Path]::GetTempPath()) `
    ("agv_{0}.exe" -f $Name)
  $command = 'call "{0}" >nul && cl /nologo /W4 /WX /utf-8 /TC "{1}" {2} {3} /I "{4}" /I "{5}" /I "{6}" /Fe:"{7}" && "{7}"' -f `
    $vcvarsPath, $testSource, $moduleArguments, $sourceIncludeArguments, `
    $contractIncludePath, $testIncludePath, $boardIncludePath, $testExecutable

  & $env:ComSpec /d /c $command
  if($LASTEXITCODE -ne 0) {
    throw "Host test failed: $Name"
  }
}

Invoke-CHostTest `
  -Name 'chassis_heading_fusion_test' `
  -TestRelativePath 'tests\chassis_heading_fusion_test.c' `
  -ModuleRelativePath 'src\services\chassis_heading_fusion.c'
Invoke-CHostTest `
  -Name 'canfd_status_codec_test' `
  -TestRelativePath 'tests\canfd_status_codec_test.c' `
  -ModuleRelativePath 'src\services\canfd_status_codec.c'
Invoke-CHostTest `
  -Name 'motion_reference_test' `
  -TestRelativePath 'tests\motion_reference_test.c' `
  -ModuleRelativePath 'src\services\motion_reference_receiver.c', `
                      'src\services\motion_reference_control.c'
Invoke-CHostTest `
  -Name 'system_watchdog_test' `
  -TestRelativePath 'tests\system_watchdog_test.c' `
  -ModuleRelativePath @()
Invoke-CHostTest `
  -Name 'power_supervisor_test' `
  -TestRelativePath 'tests\power_supervisor_test.c' `
  -ModuleRelativePath 'src\services\power_supervisor.c'
Invoke-CHostTest `
  -Name 'motor_speed_control_test' `
  -TestRelativePath 'tests\motor_speed_control_test.c' `
  -ModuleRelativePath 'src\services\motor_speed_control.c'
