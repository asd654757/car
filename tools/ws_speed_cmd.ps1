param(
    [string]$PortName = 'COM15',
    [int]$Slave = 4,
    [int]$Baud = 9600,
    [int]$Raw = 100,
    [int]$Seconds = 4,
    [int]$SpeedReg = 0x0303,
    [switch]$Yes
)

# Command an internal preset speed, watch the monitor, then FORCE the register back to 0.
# Ports 2026-10-09: CH9114 quad UART -> COM12=slave1 COM13=slave2 COM14=slave3 COM15=slave4.
# 2026-10-09: the register that actually feeds 0x0707 C.SPd is Pn-303, not Pn-304.
#   Evidence: |C.SPd| == Pn-303 * 0.2 on all four drives (1500 -> 299 rpm, 300 -> 59 rpm),
#             while Pn-304 = 0 on every drive and still produced -59 on drive 4.
# Unit is 0.2 rpm per register, so raw 300 = 60 rpm. Hard cap 300 unless edited.
$HARD_CAP = 300
$CEILING_ADDR = 0x030D
if (@(0x0301, 0x0302, 0x0303, 0x0304) -notcontains $SpeedReg) {
    Write-Host 'REFUSED: -SpeedReg must be one of 0x0301..0x0304 (Pn-301..304)'
    exit 1
}
$RegLabel = 'Pn-30' + [string]($SpeedReg -band 0xF)

function Get-Crc16([byte[]]$data) {
    $crc = 0xFFFF
    foreach ($b in $data) {
        $crc = $crc -bxor [int]$b
        for ($i = 0; $i -lt 8; $i++) {
            if ($crc -band 1) { $crc = ($crc -shr 1) -bxor 0xA001 } else { $crc = $crc -shr 1 }
        }
    }
    return $crc
}
function HexA([byte[]]$d) { if ($null -eq $d -or $d.Count -eq 0) { return '(none)' } else { (($d | ForEach-Object { $_.ToString('X2') }) -join ' ') } }

$p = New-Object System.IO.Ports.SerialPort($PortName, $Baud, 'None', 8, 'Two')
$p.ReadTimeout = 500; $p.WriteTimeout = 500
$p.DtrEnable = $true; $p.RtsEnable = $true
$inbuf = New-Object byte[] 512

function Read-Raw {
    $list = New-Object System.Collections.Generic.List[byte]
    $idle = [System.Diagnostics.Stopwatch]::StartNew()
    $tot = [System.Diagnostics.Stopwatch]::StartNew()
    while ($tot.ElapsedMilliseconds -lt 500) {
        $n = $p.BytesToRead
        if ($n -gt 0) {
            $got = $p.Read($inbuf, 0, $n)
            for ($j = 0; $j -lt $got; $j++) { $list.Add($inbuf[$j]) }
            $idle = [System.Diagnostics.Stopwatch]::StartNew()
        } elseif ($list.Count -gt 0 -and $idle.ElapsedMilliseconds -gt 35) { break }
        Start-Sleep -Milliseconds 1
    }
    return $list.ToArray()
}

function Read-Regs([int]$start, [int]$qty) {
    $req = [byte[]]@($Slave, 3, (($start -shr 8) -band 0xFF), ($start -band 0xFF), (($qty -shr 8) -band 0xFF), ($qty -band 0xFF), 0, 0)
    $c = Get-Crc16 $req[0..5]
    $req[6] = [byte]($c -band 0xFF); $req[7] = [byte](($c -shr 8) -band 0xFF)
    $p.DiscardInBuffer(); $p.Write($req, 0, 8)
    $r = Read-Raw
    if ($r.Count -lt (5 + $qty * 2)) { return $null }
    if ($r[1] -ge 0x80) { return $null }
    $body = [byte[]]$r[0..($r.Count - 3)]
    $cc = Get-Crc16 $body
    if ((($cc -band 0xFF) -ne $r[$r.Count - 2]) -or ((($cc -shr 8) -band 0xFF) -ne $r[$r.Count - 1])) { return $null }
    $out = @{}
    for ($i = 0; $i -lt $qty; $i++) {
        $u = $r[3 + $i * 2] * 256 + $r[4 + $i * 2]
        $out[($start + $i)] = $(if ($u -gt 32767) { $u - 65536 } else { $u })
    }
    return $out
}

function Write-One([int]$addr, [int]$value) {
    $req = [byte[]]@($Slave, 6, (($addr -shr 8) -band 0xFF), ($addr -band 0xFF), (($value -shr 8) -band 0xFF), ($value -band 0xFF), 0, 0)
    $c = Get-Crc16 $req[0..5]
    $req[6] = [byte]($c -band 0xFF); $req[7] = [byte](($c -shr 8) -band 0xFF)
    $p.DiscardInBuffer(); $p.Write($req, 0, 8)
    $r = Read-Raw
    if ($r.Count -ge 8 -and $r[1] -eq 6) {
        $same = $true
        for ($i = 0; $i -lt 6; $i++) { if ($r[$i] -ne $req[$i]) { $same = $false } }
        if ($same) { return $true }
    }
    return $false
}

Write-Host ("slave $Slave  $RegLabel <- {0} raw ({1} rpm)  for {2}s, then forced back to 0" -f $Raw, ($Raw * 0.2), $Seconds)
Write-Host ''

if ([Math]::Abs($Raw) -gt $HARD_CAP) {
    Write-Host ("REFUSED: |raw| $Raw exceeds hard cap $HARD_CAP (60 rpm motor-side). Edit HARD_CAP if you really mean it.")
    exit 1
}
if (-not $Yes) {
    Write-Host 'DRY RUN - no frames sent. Re-run with -Yes.'
    exit 0
}

$p.Open()
try {
    $m0 = Read-Regs 0x0700 24
    if ($null -eq $m0) { Write-Host 'ABORT: cannot read monitor group'; exit 2 }
    $op = $m0[0x070F]
    $pn000h = Read-Regs 0x0000 1
    $mode1 = $null
    if ($null -ne $pn000h) { $mode1 = ($pn000h[0x0000] -shr 4) -band 0xF }
    $ceil = Read-Regs $CEILING_ADDR 1
    $ceilingVal = $null
    if ($null -ne $ceil) { $ceilingVal = $ceil[$CEILING_ADDR] }
    Write-Host ('  OP(不可靠,恒为1) = ' + $op + '   Pn-000 bit1 = ' + $mode1 + '   Pn-30D ceiling = ' + $ceilingVal + '   SPd now = ' + $m0[0x0706] + '   C.SPd now = ' + $m0[0x0707] + '   AvL = ' + $m0[0x0709] + '   vdC = ' + $m0[0x070B] + 'V  td = ' + $m0[0x0717] + 'C')
    if ($mode1 -ne 3) {
        Write-Host '  REFUSED: Pn-000 第1位 is not 3 (内部设定速度模式). Change the mode first.'
        exit 3
    }
    if ($null -eq $ceilingVal -or $ceilingVal -gt $HARD_CAP) {
        Write-Host '  REFUSED: Pn-30D ceiling is missing or above the cap - set the ceiling first.'
        exit 3
    }

    $pos0 = $m0[0x0714]
    if (-not (Write-One $SpeedReg $Raw)) { Write-Host '  WRITE FAILED for speed command'; exit 4 }
    Write-Host ('  speed command sent, sampling ' + $Seconds + 's ...')
    Write-Host '   t(ms)   SPd  C.SPd   AvL  PE.L  vdC   POS      dI  dO   td'

    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $maxSpd = -99999; $minSpd = 99999; $maxAvL = 0; $maxPos = $pos0; $minPos = $pos0; $minVdC = 999; $samples = 0
    $revAcc = 0.0; $prevT = 0.0
    while ($sw.ElapsedMilliseconds -lt ($Seconds * 1000)) {
        $m = Read-Regs 0x0700 24
        if ($null -ne $m) {
            $samples++
            $spd = $m[0x0706]; $cspd = $m[0x0707]; $avl = $m[0x0709]; $pel = $m[0x070A]
            $vdc = $m[0x070B]; $pos = $m[0x0714]; $di = $m[0x0710]; $do = $m[0x0711]; $td = $m[0x0717]
            if ($spd -gt $maxSpd) { $maxSpd = $spd }
            if ($spd -lt $minSpd) { $minSpd = $spd }
            if ($avl -gt $maxAvL) { $maxAvL = $avl }
            if ($pos -gt $maxPos) { $maxPos = $pos }
            if ($pos -lt $minPos) { $minPos = $pos }
            if ($vdc -lt $minVdC) { $minVdC = $vdc }
            $dt = ($sw.ElapsedMilliseconds - $prevT) / 1000.0; $prevT = $sw.ElapsedMilliseconds
            $revAcc += ($spd * $dt) / 60.0
            Write-Host ('  {0,6} {1,6} {2,6} {3,5} {4,5} {5,4} {6,8} {7,4} {8,4} {9,4}' -f $sw.ElapsedMilliseconds, $spd, $cspd, $avl, $pel, $vdc, $pos, $di, $do, $td)
        } else {
            Write-Host ('  {0,6}   read fail' -f $sw.ElapsedMilliseconds)
        }
        Start-Sleep -Milliseconds 150
    }

    Write-One $SpeedReg 0 | Out-Null
    Write-Host "  speed command cleared ($RegLabel = 0)"
    Start-Sleep -Milliseconds 400
    $mv = Read-Regs 0x0700 24
    $afterSpd = 'n/a'; $afterAvL = 'n/a'
    if ($null -ne $mv) { $afterSpd = $mv[0x0706]; $afterAvL = $mv[0x0709] }
    Write-Host ''
    Write-Host ('  samples ok     : ' + $samples)
    Write-Host ('  SPd range      : ' + $minSpd + ' .. ' + $maxSpd + ' rpm')
    Write-Host ('  integrated     : ' + [Math]::Round($revAcc, 3) + ' rev (signed), POS swing = ' + ($maxPos - $minPos) + ' pulses')
    Write-Host ('  AvL peak       : ' + $maxAvL + ' %   vdC min: ' + $minVdC + ' V')
    Write-Host ('  after stopping : SPd = ' + $afterSpd + '  AvL = ' + $afterAvL)
    $verdict = 'no motion detected'
    if ($maxSpd -gt 5 -or $minSpd -lt -5) { $verdict = 'MOTOR TURNED FROM PC' }
    Write-Host ('  VERDICT        : ' + $verdict)
}
finally {
    try { Write-One $SpeedReg 0 | Out-Null } catch { }
    Write-Host "  (safety: $RegLabel written back to 0 on exit)"
    $p.Close(); $p.Dispose()
}
