param(
    [string]$PortName = 'COM11',
    [int]$Baud = 9600,
    [int]$MaxAddr = 16
)

# READ ONLY: Modbus func 03 only. No writes anywhere in this script.

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
    while ($tot.ElapsedMilliseconds -lt 400) {
        $n = $p.BytesToRead
        if ($n -gt 0) {
            $got = $p.Read($raw, 0, $n)
            for ($j = 0; $j -lt $got; $j++) { $list.Add($raw[$j]) }
            $idle = [System.Diagnostics.Stopwatch]::StartNew()
        } elseif ($list.Count -gt 0 -and $idle.ElapsedMilliseconds -gt 30) { break }
        Start-Sleep -Milliseconds 1
    }
    return $list.ToArray()
}

function Read-Regs([int]$slave, [int]$start, [int]$qty) {
    $req = [byte[]]@($slave, 3, (($start -shr 8) -band 0xFF), ($start -band 0xFF), (($qty -shr 8) -band 0xFF), ($qty -band 0xFF), 0, 0)
    $c = Get-Crc16 $req[0..5]
    $req[6] = [byte]($c -band 0xFF); $req[7] = [byte](($c -shr 8) -band 0xFF)
    $p.DiscardInBuffer(); $p.Write($req, 0, 8)
    $r = Read-Raw
    if ($r.Count -lt (5 + $qty * 2)) { return @{ ok = $false; why = 'no reply'; raw = $r } }
    if ($r[1] -ge 0x80) { return @{ ok = $false; why = ('exc 0x{0:X2}' -f $r[2]); raw = $r } }
    $body = [byte[]]$r[0..($r.Count - 3)]
    $cc = Get-Crc16 $body
    if ((($cc -band 0xFF) -ne $r[$r.Count - 2]) -or ((($cc -shr 8) -band 0xFF) -ne $r[$r.Count - 1])) { return @{ ok = $false; why = 'crc bad'; raw = $r } }
    if ([int]$r[0] -ne $slave) { return @{ ok = $false; why = ('reply from addr ' + $r[0]); raw = $r } }
    $out = @{}
    for ($i = 0; $i -lt $qty; $i++) {
        $u = $r[3 + $i * 2] * 256 + $r[4 + $i * 2]
        $out[($start + $i)] = $(if ($u -gt 32767) { $u - 65536 } else { $u })
    }
    return @{ ok = $true; v = $out }
}

Write-Host "=== stage 1: discovery, func 03 read 0x0000 on addresses 1..$MaxAddr ==="
$found = @()
for ($a = 1; $a -le $MaxAddr; $a++) {
    $r1 = Read-Regs $a 0x0000 1
    Start-Sleep -Milliseconds 30
    $r2 = Read-Regs $a 0x0000 1
    Start-Sleep -Milliseconds 30
    $hits = 0
    if ($r1.ok) { $hits++ }
    if ($r2.ok) { $hits++ }
    if ($hits -gt 0) {
        $val = 'n/a'
        if ($r1.ok) { $val = $r1.v[0x0000] } elseif ($r2.ok) { $val = $r2.v[0x0000] }
        $why = ''
        if (-not $r1.ok) { $why += (' [' + $r1.why + ']') }
        if (-not $r2.ok) { $why += (' [' + $r2.why + ']') }
        Write-Host ('  addr {0,2}: ANSWERS {1}/2  Pn-000={2}{3}' -f $a, $hits, $val, $why)
        $found += @{ Addr = $a; Hits = $hits }
    } else {
        Write-Host ('  addr {0,2}: silent' -f $a)
    }
}

Write-Host ''
Write-Host ('=== stage 2: detail for ' + $found.Count + ' responder(s) ===')
foreach ($f in $found) {
    $a = $f.Addr
    Write-Host ''
    Write-Host ('--- station ' + $a + ' ---')
    $g1 = Read-Regs $a 0x0000 2
    if ($g1.ok) { Write-Host ('   Pn-000 = ' + ('0x{0:X4}' -f $g1.v[0x0000]) + ' (' + $g1.v[0x0000] + ')   Pn-001 = ' + ('0x{0:X4}' -f $g1.v[0x0001])) }
    else { Write-Host ('   Pn-00x read failed: ' + $g1.why) }

    $g3 = Read-Regs $a 0x0301 4
    if ($g3.ok) {
        Write-Host ('   Pn-301..304 = ' + ($g3.v[0x0301] * 0.2) + ' / ' + ($g3.v[0x0302] * 0.2) + ' / ' + ($g3.v[0x0303] * 0.2) + ' / ' + ($g3.v[0x0304] * 0.2) + ' rpm   (raw ' + $g3.v[0x0301] + ',' + $g3.v[0x0302] + ',' + $g3.v[0x0303] + ',' + $g3.v[0x0304] + ')')
    } else { Write-Host ('   Pn-30x read failed: ' + $g3.why) }

    $cd = Read-Regs $a 0x030D 1
    if ($cd.ok) { Write-Host ('   Pn-30D speed ceiling = ' + $cd.v[0x030D] + ' raw = ' + ($cd.v[0x030D] * 0.2) + ' rpm') }

    $fn = Read-Regs $a 0x0604 2
    if ($fn.ok) { Write-Host ('   Fn-04 encoder lines = ' + $fn.v[0x0604] + '   Fn-05 comm = 0x' + ('{0:X4}' -f $fn.v[0x0605])) }

    $m1 = Read-Regs $a 0x0700 8
    $m2 = Read-Regs $a 0x0708 8
    $m3 = Read-Regs $a 0x0710 8
    if ($m1.ok -and $m2.ok -and $m3.ok) {
        Write-Host ('   SPd=' + $m1.v[0x0706] + '  C.SPd=' + $m2.v[0x0707] + '  AvL=' + $m2.v[0x0709] + '  PE.L=' + $m2.v[0x070A] + '  vdC=' + $m2.v[0x070B] + 'V')
        Write-Host ('   OP=' + $m3.v[0x070F] + '  dI=' + $m3.v[0x0710] + '  dO=' + $m3.v[0x0711] + '  POS=' + $m3.v[0x0714] + '  td=' + $m3.v[0x0717] + 'C  SvEr=' + $m3.v[0x0713])
    } else {
        Write-Host '   monitor read incomplete'
    }

    # stability: 5 identical reads, watch for collisions / dup station numbers
    $good = 0; $vals = @()
    for ($k = 1; $k -le 5; $k++) {
        $t = Read-Regs $a 0x070B 1
        if ($t.ok) { $good++; $vals += $t.v[0x070B] }
        Start-Sleep -Milliseconds 40
    }
    Write-Host ('   link stability: ' + $good + '/5 reads OK on 0x070B')
    Start-Sleep -Milliseconds 60
}

$p.Close(); $p.Dispose()
Write-Host ''
Write-Host '=== done (read only, nothing written) ==='
