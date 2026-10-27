param(
    [string]$PortName = 'COM11',
    [int]$Slave = 4,
    [int]$Baud = 9600,
    [ValidateSet('None','Even','Odd')] [string]$Parity = 'None',
    [ValidateSet('One','Two')] [string]$Stop = 'Two',
    [int]$Reps = 15,
    [int]$Start = 0x0708,
    [int]$Qty = 4
)

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
function HexA([byte[]]$d) { if ($d.Count -eq 0) { return '(none)' } else { (($d | ForEach-Object { $_.ToString('X2') }) -join ' ') } }

$p = New-Object System.IO.Ports.SerialPort($PortName, $Baud, $Parity, 8, $Stop)
$p.ReadTimeout = 500; $p.WriteTimeout = 500
$p.DtrEnable = $true; $p.RtsEnable = $true
$p.Open()

$pduHi = ($Start -shr 8) -band 0xFF
$pduLo = $Start -band 0xFF
$qHi = ($Qty -shr 8) -band 0xFF
$qLo = $Qty -band 0xFF

Write-Host ("$PortName ${Baud}bps 8-$Parity-$Stop  slave=$Slave  read 0x{0:X4} x{1}  reps=$Reps" -f $Start, $Qty)
Write-Host ("request frame : " + (HexA [byte[]]@($Slave, 3, $pduHi, $pduLo, $qHi, $qLo)))

$ok = 0; $bad = 0; $none = 0; $cnt3F = 0; $replies = @{}
$expect = 5 + $Qty * 2

for ($k = 1; $k -le $Reps; $k++) {
    $req = [byte[]]@($Slave, 3, $pduHi, $pduLo, $qHi, $qLo, 0, 0)
    $c = Get-Crc16 ($req[0..5])
    $req[6] = [byte]($c -band 0xFF); $req[7] = [byte](($c -shr 8) -band 0xFF)
    $p.DiscardInBuffer()
    $p.Write($req, 0, 8)
    $buf = New-Object System.Collections.Generic.List[byte]
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    while ($sw.ElapsedMilliseconds -lt 500) {
        $n = $p.BytesToRead
        for ($i = 0; $i -lt $n; $i++) { $buf.Add([byte]$p.ReadChar()) }
        if ($buf.Count -ge $expect) { break }
        Start-Sleep -Milliseconds 2
    }
    $r = $buf.ToArray()
    $tag = ''
    if ($r.Count -eq 0) { $none++; $tag = 'NO REPLY' }
    elseif ($r.Count -lt 5) { $bad++; $tag = 'SHORT' }
    else {
        $body = $r[0..($r.Count - 3)]
        $cc = Get-Crc16 $body
        $lo = $cc -band 0xFF; $hi = ($cc -shr 8) -band 0xFF
        $tail = $r[$r.Count - 2]
        $tail2 = $r[$r.Count - 1]
        if ($lo -eq $tail -and $hi -eq $tail2) { $ok++; $tag = 'CRC OK' }
        else { $bad++; $tag = ('CRC FAIL (calc {0:X2} {1:X2})' -f $lo, $hi) }
    }
    $n3f = (@($r | Where-Object { $_ -eq 0x3F })).Count
    $cnt3F += $n3f
    $hex = HexA $r
    if (-not $replies.ContainsKey($hex)) { $replies[$hex] = 0 }
    $replies[$hex] = $replies[$hex] + 1
    Write-Host ("  rep {0,2}: {1,3}B  {2,-26} {3}" -f $k, $r.Count, $tag, $hex)
    Start-Sleep -Milliseconds 30
}

Write-Host ''
Write-Host ('summary: CRC OK=' + $ok + '  CRC FAIL/SHORT=' + $bad + '  NO REPLY=' + $none + '  total 0x3F bytes seen=' + $cnt3F)
Write-Host 'distinct reply patterns:'
foreach ($key in $replies.Keys) { Write-Host ("  x{0,-3} {1}" -f $replies[$key], $key) }
$p.Close(); $p.Dispose()
