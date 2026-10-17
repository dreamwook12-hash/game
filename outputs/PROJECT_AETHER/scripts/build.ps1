param(
  [string]$ZigPath = (Join-Path $PSScriptRoot '..\..\..\work\tools\zig-x86_64-windows-0.17.0\zig.exe'),
  [switch]$DebugBuild
)
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
if (!(Test-Path -LiteralPath $ZigPath)) { throw 'Zig not found. Supply -ZigPath; see docs/BUILD.md. No automatic downloads.' }
$compiler = (Resolve-Path -LiteralPath $ZigPath).Path
$buildDir = Join-Path $root '..\..\work\aether-build'
New-Item -ItemType Directory -Force -Path $buildDir,(Join-Path $root 'bin'),(Join-Path $root 'reports') | Out-Null
$priorCache = $env:ZIG_GLOBAL_CACHE_DIR
$env:ZIG_GLOBAL_CACHE_DIR = Join-Path $buildDir 'zig-global-cache'
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$optimization = if ($DebugBuild) { '-O0' } else { '-O2' }
$sources = @(Get-ChildItem -LiteralPath (Join-Path $root 'core\src') -Filter '*.cpp' | Sort-Object Name | ForEach-Object FullName)
$targets = @(
  @{ Name='aether_cli'; Sources=@((Join-Path $root 'application\main.cpp')) },
  @{ Name='aether_tests'; Sources=@(Get-ChildItem -LiteralPath (Join-Path $root 'tests') -Filter '*.cpp' | Sort-Object Name | ForEach-Object FullName) }
)
try {
  foreach ($target in $targets) {
    $log = Join-Path $root "reports\build-$stamp-$($target.Name).log"
    $arguments = @('c++','-std=c++20',$optimization,'-Wall','-Wextra','-Wpedantic','-Werror','-target','x86_64-windows-gnu',"-I$(Join-Path $root 'core\include')") + $sources + $target.Sources + @('-o',(Join-Path $root "bin\$($target.Name).exe"))
    "Compiler: $compiler`nArguments: $($arguments -join ' ')" | Set-Content -LiteralPath $log
    $watch = [Diagnostics.Stopwatch]::StartNew()
    & $compiler @arguments 2>&1 | Tee-Object -FilePath $log -Append
    $code = $LASTEXITCODE
    $watch.Stop()
    "exit=$code elapsed_seconds=$($watch.Elapsed.TotalSeconds)" | Tee-Object -FilePath $log -Append
    if ($code -ne 0) { throw "Build failed; preserved log: $log" }
  }
} finally { $env:ZIG_GLOBAL_CACHE_DIR = $priorCache }
