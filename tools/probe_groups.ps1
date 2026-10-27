param(
    [string]$PortName = 'COM11',
    [int]$Slave = 4,
    [int]$Baud = 9600
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

$p = New-Object System.IO.Ports.SerialPort($PortName, $Baud, 'None', 8, 'Two')
$p.ReadTimeout = 500; $p.WriteTimeout = 500
$p.DtrEnable = $true; $p.RtsEnable = $true
$p.Open()
$raw = New-Object byte[] 512

function Read-Raw {
    $list = New-Object System.Collections.Generic.List[byte]
    $idle = [System.Diagnostics.Stopwatch]::StartNew()
    $tot = [System.Diagnostics.Stopwatch]::StartNew()
    while ($tot.ElapsedMilliseconds -lt 700) {
        $n = $p.BytesToRead
        if ($n -gt 0) {
            $got = $p.Read($raw, 0, $n)
            for ($j = 0; $j -lt $got; $j++) { $list.Add($raw[$j]) }
            $idle = [System.Diagnostics.Stopwatch]::StartNew()
        } elseif ($list.Count -gt 0 -and $idle.ElapsedMilliseconds -gt 40) { break }
        Start-Sleep -Milliseconds 1
    }
    return $list.ToArray()
}

function Probe([int]$start, [int]$qty) {
    $req = [byte[]]@($Slave, 3, (($start -shr 8) -band 0xFF), ($start -band 0xFF), (($qty -shr 8) -band 0xFF), ($qty -band 0xFF), 0, 0)
    $c = Get-Crc16 $req[0..5]
    $req[6] = [byte]($c -band 0xFF); $req[7] = [byte](($c -shr 8) -band 0xFF)
    $p.DiscardInBuffer()
    $p.Write($req, 0, 8)
    $r = Read-Raw
    $line = ('  READ 0x{0:X4} x{1,-2} -> ' -f $start, $qty)
    if ($r.Count -lt 5) { Write-Host ($line + 'NO REPLY  raw=' + (HexA $r)); return }
    if ($r[1] -ge 0x80) { Write-Host ($line + ('EXC 0x{0:X2}   raw={1}' -f $r[2], (HexA $r))); return }
    $body = [byte[]]$r[0..($r.Count - 3)]
    $cc = Get-Crc16 $body
    $valid = (($cc -band 0xFF) -eq $r[$r.Count - 2]) -and ((($cc -shr 8) -band 0xFF) -eq $r[$r.Count - 1])
    $vals = @()
    for ($i = 0; $i -lt $qty; $i++) {
        $u = $r[3 + $i * 2] * 256 + $r[4 + $i * 2]
        $s = $(if ($u -gt 32767) { $u - 65536 } else { $u })
        $vals += ('0x{0:X4}={1}(raw {2})' -f ($start + $i), $s, $u)
    }
    Write-Host ($line + ('OK crc={0}  {1}' -f $(if ($valid) { 'V' } else { 'BAD' }), ($vals -join '  ')))
}

Write-Host 'Pn-00x group probe (only Pn-000..Pn-002 exist per manual)'
foreach ($q in 1, 2, 3, 4, 8) { Probe 0x0000 $q; Start-Sleep -Milliseconds 40 }

Write-Host ''
Write-Host 'Other groups, small quantities'
Probe 0x0001 1; Probe 0x0002 1
Probe 0x0100 3
Probe 0x0200 3
Probe 0x0500 8
Probe 0x050C 4
Start-Sleep -Milliseconds 40
Write-Host ''
Write-Host 'Pn-3xx again, tight ranges'
Probe 0x0301 4
Probe 0x030D 1
Write-Host ''
Write-Host 'monitor: OP / dI / dO / SPd'
Probe 0x070F 4
$p.Close(); $p.Dispose()
Write-Host ''
Write-Host '=== done (read only) ==='
