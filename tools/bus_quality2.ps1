param(
    [string]$PortName = 'COM11',
    [int]$Slave = 4,
    [int]$Baud = 9600,
    [ValidateSet('None','Even','Odd')] [string]$Parity = 'None',
    [ValidateSet('One','Two')] [string]$Stop = 'Two',
    [int]$Data = 8,
    [int]$Reps = 6,
    [int]$Start = 0x0708,
    [int]$Qty = 4,
    [switch]$Quiet
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
function HexA([byte[]]$d) { if ($null -eq $d -or $d.Count -eq 0) { return '(none)' } else { (($d | ForEach-Object { $_.ToString('X2') }) -join ' ') } }

$p = New-Object System.IO.Ports.SerialPort($PortName, $Baud, $Parity, $Data, $Stop)
$p.ReadTimeout = 500; $p.WriteTimeout = 500
$p.DtrEnable = $true; $p.RtsEnable = $true
$p.Open()

Write-Host ("$PortName ${Baud} 8-$Parity-$Stop datalen=$Data slave=$Slave read 0x{0:X4} x{1} reps=$Reps" -f $Start, $Qty)

for ($k = 1; $k -le $Reps; $k++) {
    $req = [byte[]]@($Slave, 3, (($Start -shr 8) -band 0xFF), ($Start -band 0xFF), (($Qty -shr 8) -band 0xFF), ($Qty -band 0xFF), 0, 0)
    $c = Get-Crc16 $req[0..5]
    $req[6] = [byte]($c -band 0xFF); $req[7] = [byte](($c -shr 8) -band 0xFF)

    $p.DiscardInBuffer()
    $p.Write($req, 0, 8)

    # read until the bus goes quiet instead of until an assumed length
    $buf = New-Object System.Collections.Generic.List[byte]
    $last = [System.Diagnostics.Stopwatch]::StartNew()
    $total = [System.Diagnostics.Stopwatch]::StartNew()
    while ($total.ElapsedMilliseconds -lt 1200) {
        $n = $p.BytesToRead
        if ($n -gt 0) {
            for ($i = 0; $i -lt $n; $i++) { $buf.Add([byte]$p.ReadChar()) }
            $last = [System.Diagnostics.Stopwatch]::StartNew()
        } elseif ($buf.Count -gt 0 -and $last.ElapsedMilliseconds -gt 60) { break }
        Start-Sleep -Milliseconds 1
    }
    $r = $buf.ToArray()
    Write-Host ("  rep {0}: len={1} ({2} ms)  {3}" -f $k, $r.Count, $total.ElapsedMilliseconds, (HexA $r))

    # slide a window to find where a valid CRC actually sits
    for ($i = 2; $i -lt $r.Count - 2; $i++) {
        $body = $r[0..$i]
        $cc = Get-Crc16 ([byte[]]$body)
        $lo = $cc -band 0xFF; $hi = ($cc -shr 8) -band 0xFF
        if ($r[$i + 1] -eq $lo -and $r[$i + 2] -eq $hi) {
            Write-Host ("      -> CRC VALID with body 0..{0} (frame {1}B), CRC={2:X2} {3:X2}" -f $i, ($i + 3), $lo, $hi)
        }
        if ($r[$i + 1] -eq $hi -and $r[$i + 2] -eq $lo) {
            Write-Host ("      -> CRC VALID (swapped order) body 0..{0}, frame {1}B" -f $i, ($i + 3))
        }
    }
    Start-Sleep -Milliseconds 40
}
$p.Close(); $p.Dispose()
