param(
    [string]$PortName = 'COM11',
    [int]$Slave = 4,
    [int]$Baud = 9600,
    [int]$Samples = 12,
    [int]$GapMs = 300
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

$p = New-Object System.IO.Ports.SerialPort($PortName, $Baud, 'None', 8, 'Two')
$p.ReadTimeout = 500; $p.WriteTimeout = 500
$p.DtrEnable = $true; $p.RtsEnable = $true
$p.Open()
$raw = New-Object byte[] 512

function Read-Raw {
    $list = New-Object System.Collections.Generic.List[byte]
    $idle = [System.Diagnostics.Stopwatch]::StartNew()
    $tot = [System.Diagnostics.Stopwatch]::StartNew()
    while ($tot.ElapsedMilliseconds -lt 500) {
        $n = $p.BytesToRead
        if ($n -gt 0) {
            $got = $p.Read($raw, 0, $n)
            for ($j = 0; $j -lt $got; $j++) { $list.Add($raw[$j]) }
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

# READ ONLY - func 03 only, no writes anywhere in this script
Write-Host 't(ms)    Pn000  OP  SPd C.SPd  AvL  PE.L vdC   dI   dO    POS     td'
$t0 = [System.Diagnostics.Stopwatch]::StartNew()
for ($k = 1; $k -le $Samples; $k++) {
    $m = Read-Regs 0x0700 24
    $pn0 = Read-Regs 0x0000 1
    if ($null -ne $m) {
        $pn = 'n/a'
        if ($null -ne $pn0) { $pn = $pn0[0x0000] }
        Write-Host ('{0,6} {1,6} {2,3} {3,5} {4,5} {5,4} {6,5} {7,3} {8,4} {9,3} {10,7} {11,3}' -f `
            $t0.ElapsedMilliseconds, $pn, $m[0x070F], $m[0x0706], $m[0x0707], $m[0x0709], $m[0x070A], `
            $m[0x070B], $m[0x0710], $m[0x0711], $m[0x0714], $m[0x0717])
    } else {
        Write-Host ('{0,6}  read fail' -f $t0.ElapsedMilliseconds)
    }
    Start-Sleep -Milliseconds $GapMs
}
$p.Close(); $p.Dispose()
Write-Host ''
Write-Host '=== done (read only) ==='
