param(
    [string[]]$Ports = @('COM12','COM13','COM14','COM15'),
    [int[]]$Bauds = @(9600, 115200, 38400, 19200),
    [int]$MaxAddr = 8
)

# READ ONLY: Modbus func 03 only, no writes.

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

function Scan-Port([string]$portName) {
    $result = @{ port = $portName; state = 'open-failed'; baud = $null; addrs = @() }
    $p = $null
    foreach ($baud in $Bauds) {
        try {
            $p = New-Object System.IO.Ports.SerialPort($portName, $baud, 'None', 8, 'Two')
            $p.ReadTimeout = 300; $p.WriteTimeout = 300
            $p.DtrEnable = $true; $p.RtsEnable = $true
            $p.Open()
        } catch {
            $result.state = 'open-failed: ' + $_.Exception.Message
            return $result
        }
        $raw = New-Object byte[] 512

        $readRaw = {
            param($port, $buf)
            $list = New-Object System.Collections.Generic.List[byte]
            $idle = [System.Diagnostics.Stopwatch]::StartNew()
            $tot = [System.Diagnostics.Stopwatch]::StartNew()
            while ($tot.ElapsedMilliseconds -lt 250) {
                $n = $port.BytesToRead
                if ($n -gt 0) {
                    $got = $port.Read($buf, 0, $n)
                    for ($j = 0; $j -lt $got; $j++) { $list.Add($buf[$j]) }
                    $idle = [System.Diagnostics.Stopwatch]::StartNew()
                } elseif ($list.Count -gt 0 -and $idle.ElapsedMilliseconds -gt 25) { break }
                Start-Sleep -Milliseconds 1
            }
            return $list.ToArray()
        }

        $hits = @()
        for ($a = 1; $a -le $MaxAddr; $a++) {
            $req = [byte[]]@($a, 3, 0x00, 0x00, 0x00, 1, 0, 0)
            $c = Get-Crc16 $req[0..5]
            $req[6] = [byte]($c -band 0xFF); $req[7] = [byte](($c -shr 8) -band 0xFF)
            $p.DiscardInBuffer(); $p.DiscardOutBuffer()
            $p.Write($req, 0, 8)
            $r = & $readRaw $p $raw
            if ($r.Count -ge 7 -and [int]$r[0] -eq $a -and $r[1] -eq 3) {
                $body = [byte[]]$r[0..($r.Count - 3)]
                $cc = Get-Crc16 $body
                if ((($cc -band 0xFF) -eq $r[$r.Count - 2]) -and ((($cc -shr 8) -band 0xFF) -eq $r[$r.Count - 1])) {
                    $u = $r[3] * 256 + $r[4]
                    $s = $(if ($u -gt 32767) { $u - 65536 } else { $u })
                    $hits += @{ Addr = $a; Pn000 = $s }
                }
            }
            Start-Sleep -Milliseconds 10
        }
        $p.Close(); $p.Dispose()
        if ($hits.Count -gt 0) {
            $result.state = 'ANSWERING'
            $result.baud = $baud
            $result.addrs = $hits
            return $result
        }
    }
    $result.state = 'silent at all bauds'
    return $result
}

Write-Host '=== scanning (func 03 read of 0x0000, addresses 1..' + $MaxAddr + ') ==='
foreach ($pt in $Ports) {
    $res = Scan-Port $pt
    if ($res.state -eq 'ANSWERING') {
        $desc = ($res.addrs | ForEach-Object { 'addr ' + $_.Addr + ' Pn-000=' + ('0x{0:X4}' -f $_.Pn000) + '(' + $_.Pn000 + ')' }) -join ', '
        Write-Host ('  ' + $res.port + ' @ ' + $res.baud + ' 8-N-2 -> ' + $desc)
    } else {
        Write-Host ('  ' + $res.port + ' -> ' + $res.state + '  (tried ' + ($Bauds -join '/') + ')')
    }
}
Write-Host ''
Write-Host '=== done (read only) ==='
