# Full command-line build of the H743 board image: compile core + port + the wildfire
# template's HAL tree, assemble the Keil-syntax startup file, link with armlink, then
# fromelf a .bin/.hex. This proves the image type-checks, links and fits the memory map.
# It does NOT flash anything and it does NOT prove runtime behaviour.
# Keep this file ASCII-only: PowerShell 5.1 decodes .ps1 as GBK without a BOM.

param(
    [string]$Root = '',
    [switch]$SkipHal
)
$ErrorActionPreference = 'Stop'
if (-not $Root) { $Root = Split-Path -Parent $PSScriptRoot }      # ...\firmware
$root = $Root
$board = Join-Path $root 'board'
$tools = 'D:\Keil_v5\ARM\ARMCLANG\bin'
$armclang = Join-Path $tools 'armclang.exe'
$armasm   = Join-Path $tools 'armasm.exe'
$armlink  = Join-Path $tools 'armlink.exe'
$fromelf  = Join-Path $tools 'fromelf.exe'
$outDir   = Join-Path $env:TEMP 'chassis_link'

foreach ($t in @($armclang, $armasm, $armlink, $fromelf)) {
    if (-not (Test-Path $t)) { Write-Error "missing tool: $t"; exit 3 }
}
if (-not (Test-Path $board)) { Write-Error "board tree not found: $board"; exit 3 }

Remove-Item -Recurse -Force $outDir -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force -Path $outDir | Out-Null

# Past the preflight checks, failures are judged by exit codes only: native compiler
# warnings on stderr must not become PowerShell terminating errors.
$ErrorActionPreference = 'Continue'

$halRoot = Join-Path $board 'Libraries'
$defines = @('-DUSE_HAL_DRIVER', '-DSTM32H743xx')
$includes = @(
    "-I$root\core", "-I$root\port",
    "-I$board\User", "-I$board\User\key", "-I$board\User\led",
    "-I$halRoot\STM32H7xx_HAL_Driver\Inc",
    "-I$halRoot\CMSIS\Device\ST\STM32H7xx\Include",
    "-I$halRoot\CMSIS\Include"
)

# The project's own target string says CPUTYPE("Cortex-M7") FPU3(DFPU), i.e. double
# precision hardware FPU. armclang's arm-none-eabi default is SOFT float, so these
# three flags have to be explicit or the core's double math runs through slow
# compatibility routines and the link ABI disagrees with the vector table.
$arch = @('--target=arm-arm-none-eabi', '-mcpu=cortex-m7', '-mthumb',
          '-mfpu=fpv5-d16', '-mfloat-abi=hard')

# Our own code is held to -Werror. Vendor code (HAL, wildfire demo) is not - it is
# not ours to clean up, and failing on its warnings would hide our own regressions.
$ours   = @('-Wall', '-Wextra', '-Werror')
$theirs = @('-Wall')

$objects = @()
$script:fail = 0

function Invoke-Compile($src, $flags, $hide) {
    $leaf = Split-Path -Leaf $src
    $obj = Join-Path $outDir ($leaf -replace '\.c$', '.o')
    if (Test-Path $obj) { $obj = Join-Path $outDir (([System.IO.Path]::GetFileNameWithoutExtension($leaf) + '_' + [Math]::Abs($src.GetHashCode()) + '.o')) }
    $a = $arch + @('-c', '-std=c11', '-O1', '-g') + $flags + $defines + $includes + @($src, '-o', $obj)
    if (-not $hide) { Write-Host "== $leaf" }
    # Native tools write warnings to stderr; with ErrorActionPreference=Stop PowerShell
    # turns the first one into a terminating error. Vendor warnings are not our failure.
    $eap = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    & $armclang @a 2>&1 | ForEach-Object { Write-Host $_ }
    $rc = $LASTEXITCODE
    $ErrorActionPreference = $eap
    if ($rc -ne 0) { $script:fail++; return $null }
    return $obj
}

# --- core + port (the chassis control code) ---
foreach ($dir in @('core', 'port')) {
    foreach ($f in (Get-ChildItem (Join-Path $root $dir) -Filter '*.c')) {
        $o = Invoke-Compile $f.FullName $ours $false
        if ($o) { $objects += $o }
    }
}

# --- board User (main, interrupts, demo bsp) ---
$userFiles = @(Get-ChildItem (Join-Path $board 'User') -Filter '*.c' | ForEach-Object { $_.FullName })
foreach ($sub in @('key', 'led')) {
    $p = Join-Path $board "User\$sub"
    if (Test-Path $p) { $userFiles += @(Get-ChildItem $p -Filter '*.c' | ForEach-Object { $_.FullName }) }
}
foreach ($f in $userFiles) {
    $o = Invoke-Compile $f $theirs $false
    if ($o) { $objects += $o }
}

# --- CMSIS system file ---
$sys = Join-Path $halRoot 'CMSIS\Device\ST\STM32H7xx\Source\Templates\system_stm32h7xx.c'
$o = Invoke-Compile $sys $theirs $false
if ($o) { $objects += $o }

# --- HAL driver sources: each file self-guards on HAL_x_MODULE_ENABLED, so files for
#     modules the board conf leaves off compile to nothing.
#     The *_template.c files are excluded: they are CubeMX starting points, not driver
#     code, and all three define HAL_InitTick. The weak HAL_InitTick inside
#     stm32h7xx_hal.c configures SysTick for 1 ms, which is what this port relies on.
#     stm32h7xx_ll_utils.c is excluded too: it is only useful when the clock is set up
#     through LL, and it references LL_RCC_PrescTable, which the LL sources compile away
#     to nothing unless USE_FULL_LL_DRIVER is defined (the wildfire project leaves it off). ---
if (-not $SkipHal) {
    $files = @(Get-ChildItem (Join-Path $halRoot 'STM32H7xx_HAL_Driver\Src') -Filter '*.c' |
               Where-Object { $_.Name -notlike '*template*' -and $_.Name -ne 'stm32h7xx_ll_utils.c' })
    Write-Host ("== HAL ({0} files, quiet)" -f $files.Count)
    foreach ($f in $files) {
        $o = Invoke-Compile $f.FullName $theirs $true
        if ($o) { $objects += $o }
        if ($script:fail -gt 0 -and ($script:fail % 10) -eq 0) { Write-Host "   ... $($script:fail) HAL files failed so far" }
    }
}

# --- startup vector table: Keil/armasm syntax ---
$startup = Join-Path $halRoot 'CMSIS\Device\ST\STM32H7xx\Source\Templates\arm\startup_stm32h743xx.s'
$startObj = Join-Path $outDir 'startup.o'
Write-Host "== startup_stm32h743xx.s"
& $armasm --cpu Cortex-M7 -g -o $startObj $startup 2>&1 | ForEach-Object { Write-Host $_ }
if ($LASTEXITCODE -ne 0 -or -not (Test-Path $startObj)) { Write-Host "startup assembly FAILED"; exit 4 }
$objects += $startObj

if ($script:fail -ne 0) { Write-Host "`n$($script:fail) source(s) failed to compile"; exit 1 }
Write-Host ("`ncompiled clean: {0} objects" -f $objects.Count)

# --- link ---
$sct = Join-Path $PSScriptRoot 'board_h743.sct'
$axf = Join-Path $outDir 'chassis.axf'
$map = Join-Path $outDir 'chassis.map'
$list = Join-Path $outDir 'chassis_link_listing.txt'
$linkArgs = @('--cpu=Cortex-M7', '--scatter', $sct, '--strict', '--info=sizes,totals',
              '--map', '--list', $list, '-o', $axf) + $objects
& $armlink @linkArgs 2>&1 | ForEach-Object { Write-Host $_ }
if ($LASTEXITCODE -ne 0) { Write-Host "`narmlink FAILED"; exit 2 }
Write-Host "map written: $map"

# --- hex/bin for the record (nothing is flashed from here) ---
& $fromelf '--bin' '--output' (Join-Path $outDir 'chassis.bin') $axf 2>&1 | ForEach-Object { Write-Host $_ }
& $fromelf '--i32' '--output' (Join-Path $outDir 'chassis.hex') $axf 2>&1 | ForEach-Object { Write-Host $_ }

Write-Host "`nLINK OK"
Write-Host ("  axf : {0}" -f $axf)
Write-Host ("  list: {0}" -f $list)
exit 0
