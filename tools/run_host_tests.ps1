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
  $command = 'call "{0}" >nul && cl /nologo /W4 /WX /utf-8 /TC "{1}" {2} {3} /I "{4}" /I "{5}" /Fe:"{6}" && "{6}"' -f `
    $vcvarsPath, $testSource, $moduleArguments, $sourceIncludeArguments, `
    $testIncludePath, $boardIncludePath, $testExecutable

  & $env:ComSpec /d /c $command
  if($LASTEXITCODE -ne 0) {
    throw "Host test failed: $Name"
  }
}

Invoke-CHostTest `
  -Name 'odometry_test' `
  -TestRelativePath 'tests\odometry_test.c' `
  -ModuleRelativePath 'src\services\odometry.c'
Invoke-CHostTest `
  -Name 'drive_control_test' `
  -TestRelativePath 'tests\drive_control_test.c' `
  -ModuleRelativePath 'src\services\drive_control.c'
Invoke-CHostTest `
  -Name 'safety_test' `
  -TestRelativePath 'tests\safety_test.c' `
  -ModuleRelativePath 'src\services\safety.c'
