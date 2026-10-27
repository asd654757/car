param(
    [int]$Baud = 9600
)

# READ ONLY: Modbus func 03 only, no writes.
$Drives = @(
    @{ Port = 'COM12'; Addr = 1 },
    @{ Port = 'COM13'; Addr = 2 },
    @{ Port = 'COM14'; Addr = 3 },
    @{ Port = 'COM15'; Addr = 4 }
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

function Read-Regs([System.IO.Ports.SerialPort]$port, [byte[]]$buf, [int]$slave, [int]$start, [int]$qty) {
    $req = [byte[]]@($slave, 3, (($start -shr 8) -band 0xFF), ($start -band 0xFF), (($qty -shr 8) -band 0xFF), ($qty -band 0xFF), 0, 0)
    $c = Get-Crc16 $req[0..5]
    $req[6] = [byte]($c -band 0xFF); $req[7] = [byte](($c -shr 8) -band 0xFF)
    for ($try = 1; $try -le 3; $try++) {
        $port.DiscardInBuffer()
        $port.Write($req, 0, 8)
        $list = New-Object System.Collections.Generic.List[byte]
        $idle = [System.Diagnostics.Stopwatch]::StartNew()
        $tot = [System.Diagnostics.Stopwatch]::StartNew()
        while ($tot.ElapsedMilliseconds -lt 500) {
            $n = $port.BytesToRead
            if ($n -gt 0) {
                $got = $port.Read($buf, 0, $n)
                for ($j = 0; $j -lt $got; $j++) { $list.Add($buf[$j]) }
                $idle = [System.Diagnostics.Stopwatch]::StartNew()
            } elseif ($list.Count -gt 0 -and $idle.ElapsedMilliseconds -gt 30) { break }
            Start-Sleep -Milliseconds 1
        }
        $r = $list.ToArray()
        if ($r.Count -ge (5 + $qty * 2) -and [int]$r[0] -eq $slave -and $r[1] -eq 3) {
            $body = [byte[]]$r[0..($r.Count - 3)]
            $cc = Get-Crc16 $body
            if ((($cc -band 0xFF) -eq $r[$r.Count - 2]) -and ((($cc -shr 8) -band 0xFF) -eq $r[$r.Count - 1])) {
                $out = @{}
                for ($i = 0; $i -lt $qty; $i++) {
                    $u = $r[3 + $i * 2] * 256 + $r[4 + $i * 2]
                    $out[($start + $i)] = $(if ($u -gt 32767) { $u - 65536 } else { $u })
                }
                return $out
            }
        }
        Start-Sleep -Milliseconds 40
    }
    return $null
}

function Mode-Decode([int]$v) {
    $d0 = $v -band 0xF
    $d1 = ($v -shr 4) -band 0xF
    $d2 = ($v -shr 8) -band 0xF
    $m = @('analog-speed','pulse-position','analog-torque','internal-preset-speed')
    $ms = 'other(' + $d1 + ')'
    if ($d1 -ge 0 -and $d1 -le 3) { $ms = $m[$d1] }
    $en = $(if ($d2 -eq 1) { 'keyboard-enable' } else { 'hardware-SON-enable' })
    $dir = $(if ($d0 -eq 1) { 'CW=positive' } else { 'CCW=positive' })
    return ($ms + ', ' + $en + ', ' + $dir)
}

foreach ($d in $Drives) {
    Write-Host ''
    Write-Host ('==================== ' + $d.Port + '  slave ' + $d.Addr + ' ====================')
    $p = $null
    try {
        $p = New-Object System.IO.Ports.SerialPort($d.Port, $Baud, 'None', 8, 'Two')
        $p.ReadTimeout = 500; $p.WriteTimeout = 500
        $p.DtrEnable = $true; $p.RtsEnable = $true
        $p.Open()
    } catch {
        Write-Host ('  cannot open port: ' + $_.Exception.Message)
        continue
    }
    $buf = New-Object byte[] 512

    $g0 = Read-Regs $p $buf $d.Addr 0x0000 3
    if ($null -ne $g0) {
        $pn000 = $g0[0x0000]
        Write-Host ('  Pn-000 = ' + ('0x{0:X4}' -f $pn000) + ' (' + $pn000 + ')  -> ' + (Mode-Decode $pn000))
        Write-Host ('  Pn-001 = ' + ('0x{0:X4}' -f $g0[0x0001]) + '   Pn-002 = ' + ('0x{0:X4}' -f $g0[0x0002]))
    } else { Write-Host '  Pn-00x: NO REPLY' }

    $g3 = Read-Regs $p $buf $d.Addr 0x0301 4
    if ($null -ne $g3) {
        Write-Host ('  Pn-301..304 raw ' + $g3[0x0301] + '/' + $g3[0x0302] + '/' + $g3[0x0303] + '/' + $g3[0x0304] + '  = ' + ($g3[0x0301] * 0.2) + '/' + ($g3[0x0302] * 0.2) + '/' + ($g3[0x0303] * 0.2) + '/' + ($g3[0x0304] * 0.2) + ' rpm')
    } else { Write-Host '  Pn-30x: NO REPLY' }

    $cd = Read-Regs $p $buf $d.Addr 0x030D 1
    if ($null -ne $cd) { Write-Host ('  Pn-30D ceiling = ' + $cd[0x030D] + ' raw = ' + ($cd[0x030D] * 0.2) + ' rpm') }

    $fn = Read-Regs $p $buf $d.Addr 0x0604 2
    if ($null -ne $fn) { Write-Host ('  Fn-04 encoder = ' + $fn[0x0604] + '   Fn-05 comm = ' + ('0x{0:X4}' -f $fn[0x0605])) }

    $m1 = Read-Regs $p $buf $d.Addr 0x0700 8
    $m2 = Read-Regs $p $buf $d.Addr 0x0708 8
    $m3 = Read-Regs $p $buf $d.Addr 0x0710 8
    if ($null -ne $m1 -and $null -ne $m2 -and $null -ne $m3) {
        Write-Host ('  SPd=' + $m1[0x0706] + ' rpm   C.SPd=' + $m1[0x0707] + ' rpm   C.tq=' + $m2[0x0708] + '   AvL=' + $m2[0x0709] + '%   PE.L=' + $m2[0x070A] + '%   vdC=' + $m2[0x070B] + 'V')
        Write-Host ('  OP=' + $m2[0x070F] + '   dI=' + $m3[0x0710] + '   dO=' + $m3[0x0711] + '   SvEr=' + $m3[0x0713] + '   POS=' + $m3[0x0714] + '   td=' + $m3[0x0717] + 'C')
        $cmd = $m1[0x0707]
        $flag = ''
        if ([math]::Abs($cmd) -gt 3) { $flag = '   <<< NON-ZERO SPEED COMMAND PRESENT' }
        Write-Host ('  command check: C.SPd = ' + $cmd + $flag)
    } else { Write-Host '  monitor: NO REPLY' }

    $p.Close(); $p.Dispose()
    Start-Sleep -Milliseconds 100
}

Write-Host ''
Write-Host '=== done (read only, nothing written) ==='
