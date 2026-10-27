# Compile-only check of the C control core with Keil's armclang (Cortex-M7).
# No host C compiler on this PC, so this is the strongest available gate:
# it proves the code is type-correct and free of syntax errors, NOT that tests pass.
# Keep this file ASCII-only: PowerShell 5.1 decodes .ps1 as GBK without a BOM.

param(
    [string]$Root = ''
)
$ErrorActionPreference = 'Stop'
if (-not $Root) { $Root = Split-Path -Parent $PSScriptRoot }   # E:\...\firmware
$root = $Root
$armclang = 'D:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe'
$outDir = Join-Path $env:TEMP 'chassis_build'

if (-not (Test-Path $armclang)) { Write-Error "armclang not found at $armclang"; exit 3 }
New-Item -ItemType Directory -Force -Path $outDir | Out-Null

$sources = @()
$sources += Get-ChildItem (Join-Path $root 'core') -Filter '*.c' | ForEach-Object { $_.FullName }
$sources += Get-ChildItem (Join-Path $root 'port') -Filter '*.c' | ForEach-Object { $_.FullName }
$sources += Get-ChildItem (Join-Path $root 'test') -Filter '*.c' | ForEach-Object { $_.FullName }

# The board layer (port/chassis_uart_h743.c) needs ST's HAL headers. The example pack we
# copied in brings its own HAL source tree, so no extra download is required to check it.
$board = Join-Path $root 'board'
$halFlags = @()
if (Test-Path $board) {
    $inc = @(
        (Join-Path $board 'Libraries\STM32H7xx_HAL_Driver\Inc'),
        (Join-Path $board 'Libraries\CMSIS\Device\ST\STM32H7xx\Include'),
        (Join-Path $board 'Libraries\CMSIS\Include'),
        (Join-Path $board 'User')
    )
    $halFlags = @('-DUSE_HAL_DRIVER', '-DSTM32H743xx')
    foreach ($i in $inc) { $halFlags += "-I$i" }
    Write-Host "board HAL tree: $board"
}

$fail = 0
foreach ($src in $sources) {
    $obj = Join-Path $outDir ((Split-Path -Leaf $src) -replace '\.c$', '.o')
    $args = @('--target=arm-arm-none-eabi', '-mcpu=cortex-m7', '-mthumb', '-c',
              '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror',
              "-I$root\core", "-I$root\port") + $halFlags + @($src, '-o', $obj)
    Write-Host "== $(Split-Path -Leaf $src)"
    & $armclang @args 2>&1 | ForEach-Object { Write-Host $_ }
    if ($LASTEXITCODE -ne 0) { $fail++ }
}

if ($fail -eq 0) {
    Write-Host "`nALL OK ($($sources.Count) files compiled clean for Cortex-M7)"
} else {
    Write-Host "`n$fail file(s) failed"
}
exit $fail
