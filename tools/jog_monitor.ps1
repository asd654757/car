param(
    [string]$PortName = 'COM11',
    [int]$Slave = 4,
    [int]$Baud = 9600,
    [int]$Seconds = 240,
    [string]$Csv = 'E:\底盘\logs\jog_monitor.csv'
)

function Get-Crc16([byte[]]$d) {
    $crc = 0xFFFF
    foreach ($b in $d) {
        $crc = $crc -bxor [int]$b
        for ($i = 0; $i -lt 8; $i++) {
            if ($crc -band 1) { $crc = ($crc -shr 1) -bxor 0xA001 } else { $crc = $crc -shr 1 }
        }
    }
    return $crc
}

$p = New-Object System.IO.Ports.SerialPort($PortName, $Baud, 'None', 8, 'Two')
$p.ReadTimeout = 500; $p.WriteTimeout = 500
$p.DtrEnable = $true; $p.RtsEnable = $true
$p.Open()
$raw = New-Object byte[] 512

function Read-Monitor {
    $req = [byte[]]@($Slave, 3, 0x07, 0x00, 0x00, 0x18, 0, 0)
    $c = Get-Crc16 $req[0..5]
    $req[6] = [byte]($c -band 0xFF); $req[7] = [byte](($c -shr 8) -band 0xFF)
    $p.DiscardInBuffer(); $p.Write($req, 0, 8)
    $l = New-Object System.Collections.Generic.List[byte]
    $idle = [System.Diagnostics.Stopwatch]::StartNew(); $tot = [System.Diagnostics.Stopwatch]::StartNew()
    while ($tot.ElapsedMilliseconds -lt 700) {
        $n = $p.BytesToRead
        if ($n -gt 0) { $g = $p.Read($raw, 0, $n); for ($j = 0; $j -lt $g; $j++) { $l.Add($raw[$j]) }; $idle = [System.Diagnostics.Stopwatch]::StartNew() }
        elseif ($l.Count -gt 0 -and $idle.ElapsedMilliseconds -gt 45) { break }
        Start-Sleep -Milliseconds 1
    }
    $r = $l.ToArray()
    if ($r.Count -lt 53 -or $r[1] -ge 0x80) { return $null }
    $cc = Get-Crc16 $r[0..50]
    if ((($cc -band 0xFF) -ne $r[51]) -or ((($cc -shr 8) -band 0xFF) -ne $r[52])) { return $null }
    $v = @{}
    $keys = @('FbP','Fbr','CPr','Cr','PEr','CPFr','SPd','CSPd','Ctq','AvL','PEL','vdC','JL','PLS','rEv','OP','dI','dO','rtp','SvEr','POS','Uv','AnG','td')
    for ($i = 0; $i -lt 24; $i++) {
        $u = $r[3 + $i * 2] * 256 + $r[4 + $i * 2]
        $v[$keys[$i]] = $(if ($u -gt 32767) { $u - 65536 } else { $u })
        $v[$keys[$i] + '_u'] = $u
    }
    return $v
}

function Read-Fn0A {
    # Fn-0A 速度寸动设定（单位 转/分），面板上设的点动速度就落在这里
    $req = [byte[]]@($Slave, 3, 0x06, 0x0A, 0x00, 0x01, 0, 0)
    $c = Get-Crc16 $req[0..5]
    $req[6] = [byte]($c -band 0xFF); $req[7] = [byte](($c -shr 8) -band 0xFF)
    $p.DiscardInBuffer(); $p.Write($req, 0, 8)
    $l = New-Object System.Collections.Generic.List[byte]
    $idle = [System.Diagnostics.Stopwatch]::StartNew(); $tot = [System.Diagnostics.Stopwatch]::StartNew()
    while ($tot.ElapsedMilliseconds -lt 500) {
        $n2 = $p.BytesToRead
        if ($n2 -gt 0) { $g = $p.Read($raw, 0, $n2); for ($j = 0; $j -lt $g; $j++) { $l.Add($raw[$j]) }; $idle = [System.Diagnostics.Stopwatch]::StartNew() }
        elseif ($l.Count -gt 0 -and $idle.ElapsedMilliseconds -gt 40) { break }
        Start-Sleep -Milliseconds 1
    }
    $r = $l.ToArray()
    if ($r.Count -lt 5 -or $r[1] -ge 0x80) { return -1 }
    return ($r[3] * 256 + $r[4])
}

$base = Read-Monitor
if ($null -eq $base) { Write-Host 'FIRST READ FAILED - check port/bus'; $p.Close(); exit 1 }
Write-Host ('baseline: 控制模式 OP=' + $base.OP + '  DI=0x' + ('{0:X}' -f $base.dI_u) + '  DO=0x' + ('{0:X}' -f $base.dO_u) +
    '  母线=' + $base.vdC + 'V  温度=' + $base.td + 'C  转速=' + $base.SPd + '  Fb.P=' + $base.FbP + '  POS=' + $base.POS)
Write-Host ''
Write-Host '  time   SPd    CSPd   AvL%  PE.L%  Fb.P      Fb.r   PEr     POS      dI    dO   vdC  td   CRC'

$sw = [System.Diagnostics.Stopwatch]::StartNew()
$prev = $null
$n = 0; $bad = 0
$prevJog = Read-Fn0A
Write-Host ('  当前面板点动速度设定 Fn-0A = ' + $prevJog + ' rpm')
$maxSpd = 0; $minSpd = 0; $maxTq = 0; $moved = $false
$posMin = 999999999; $posMax = -999999999
$rows = New-Object System.Collections.Generic.List[string]
$rows.Add('t_ms,SPd,CSPd,Ctq,AvL,PEL,FbP,Fb.r,PEr,POS,dI,dO,vdC,td')
try {
    New-Item -ItemType Directory -Force -Path (Split-Path $Csv) | Out-Null
    Set-Content -Path ($Csv + '.live') -Value 't_ms,SPd,CSPd,Ctq,AvL,PEL,FbP,Fb.r,PEr,POS,dI,dO,vdC,td' -Encoding UTF8
} catch {}
$lastPrint = -1000

while ($sw.ElapsedMilliseconds -lt ($Seconds * 1000)) {
    $m = Read-Monitor
    $n++
    if (($n % 6) -eq 0) {
        $jv = Read-Fn0A
        if ($jv -ge 0 -and $jv -ne $prevJog) {
            Write-Host ('>>> [' + $sw.ElapsedMilliseconds + ' ms] 面板点动速度设定 Fn-0A 变为 ' + $jv + ' rpm')
            $prevJog = $jv
        }
    }
    if ($null -eq $m) { $bad++; Start-Sleep -Milliseconds 60; continue }
    $t = $sw.ElapsedMilliseconds
    $line = ('{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13}' -f $t, $m.SPd, $m.CSPd, $m.Ctq, $m.AvL, $m.PEL, $m.FbP, $m.Fbr, $m.PEr, $m.POS, $m.dI_u, $m.dO_u, $m.vdC, $m.td)
    $rows.Add($line)
    try { [System.IO.File]::AppendAllText(($Csv + '.live'), ($line + [Environment]::NewLine)) } catch {}
    if ([math]::Abs($m.SPd) -gt [math]::Abs($maxSpd)) { $maxSpd = $m.SPd }
    if ($m.SPd -lt $minSpd) { $minSpd = $m.SPd }
    if ([math]::Abs($m.AvL) -gt [math]::Abs($maxTq)) { $maxTq = $m.AvL }
    if ($m.POS -lt $posMin) { $posMin = $m.POS }
    if ($m.POS -gt $posMax) { $posMax = $m.POS }
    if ($m.SPd -ne 0 -or $m.CSPd -ne 0 -or $m.AvL -ne 0) { $moved = $true }
    $changed = ($null -eq $prev) -or ($m.SPd -ne $prev.SPd) -or ($m.CSPd -ne $prev.CSPd) -or ($m.AvL -ne $prev.AvL) -or ($m.POS -ne $prev.POS) -or ($m.dI_u -ne $prev.dI_u) -or ($m.dO_u -ne $prev.dO_u)
    if ($changed -and ($t - $lastPrint) -ge 120) {
        Write-Host ('{0,7} {1,6} {2,6} {3,5} {4,6} {5,9} {6,7} {7,6} {8,8} {9,5} {10,4} {11,4} {12,3}' -f $t, $m.SPd, $m.CSPd, $m.AvL, $m.PEL, $m.FbP, $m.Fbr, $m.PEr, $m.POS, ('0x' + ('{0:X}' -f $m.dI_u)), ('0x' + ('{0:X}' -f $m.dO_u)), $m.vdC, $m.td)
        $lastPrint = $t
    }
    $prev = $m
    Start-Sleep -Milliseconds 80
}
$p.Close(); $p.Dispose()

Write-Host ''
Write-Host '========== SUMMARY =========='
Write-Host ('samples ok=' + ($n - $bad) + '  crc-fail/timeout=' + $bad)
Write-Host ('转速 SPd        : max=' + $maxSpd + ' rpm  min=' + $minSpd + ' rpm')
Write-Host ('速度命令 CSPd   : last=' + $prev.CSPd)
Write-Host ('平均转矩 Av.L   : peak abs=' + $maxTq + ' %')
Write-Host ('编码器位置 POS  : min=' + $posMin + ' max=' + $posMax + '  swing=' + ($posMax - $posMin) + ' 脉冲')
Write-Host ('电机输入脉冲 Fb.P: last=' + $prev.FbP + '   圈数 Fb.r=' + $prev.Fbr)
Write-Host ('DI/DO           : 0x' + ('{0:X}' -f $prev.dI_u) + ' / 0x' + ('{0:X}' -f $prev.dO_u))
Write-Host ('母线电压/温度   : ' + $prev.vdC + ' V / ' + $prev.td + ' C')
if ($moved -or (($posMax - $posMin) -gt 50)) { $verdict = '电机有运动' } else { $verdict = '未检测到运动: 转速/转矩/位置全程无变化' }
Write-Host ('判定            : ' + $verdict)
try { New-Item -ItemType Directory -Force -Path (Split-Path $Csv) | Out-Null; Set-Content -Path $Csv -Value $rows -Encoding UTF8; Write-Host ('csv: ' + $Csv) } catch { Write-Host ('csv write failed: ' + $_.Exception.Message) }
