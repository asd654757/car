param(
    [string]$PortName = "COM11",
    [int]$ListenMs = 1200
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

function Hex([byte[]]$d) { if ($d.Count -eq 0) { return '-' } else { (($d | ForEach-Object { $_.ToString('X2') }) -join ' ') } }

function Read-Some($p, [int]$ms, [int]$need) {
    $buf = New-Object System.Collections.Generic.List[byte]
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    while ($sw.ElapsedMilliseconds -lt $ms) {
        $n = $p.BytesToRead
        for ($i = 0; $i -lt $n; $i++) { $buf.Add([byte]$p.ReadChar()) }
        if ($need -gt 0 -and $buf.Count -ge $need) { break }
        Start-Sleep -Milliseconds 3
    }
    return $buf.ToArray()
}

Write-Host ("Available ports: " + (([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object) -join ', '))

$settings = @(
    @{ B = 9600;   D = 8; P = 'None'; S = 'Two'; Tag = 'WS servo factory default 8-N-2' },
    @{ B = 9600;   D = 8; P = 'None'; S = 'One'; Tag = '8-N-1' },
    @{ B = 115200; D = 8; P = 'None'; S = 'One'; Tag = 'MT_API controller default 115200-N-8-1' },
    @{ B = 19200;  D = 8; P = 'None'; S = 'Two'; Tag = 'servo alt baud 19200' },
    @{ B = 38400;  D = 8; P = 'None'; S = 'Two'; Tag = 'servo alt baud 38400' },
    @{ B = 115200; D = 8; P = 'None'; S = 'Two'; Tag = '115200 8-N-2' }
)

foreach ($s in $settings) {
    Write-Host ''
    Write-Host ('=== {0} bps {1}-{2}-{3}  [{4}] ===' -f $s.B, $s.D, $s.P, $s.S, $s.Tag)
    $p = New-Object System.IO.Ports.SerialPort($PortName, $s.B, $s.P, $s.D, $s.S)
    $p.ReadTimeout = 400; $p.WriteTimeout = 400
    $p.DtrEnable = $true; $p.RtsEnable = $true
    try { $p.Open() } catch { Write-Host ('  OPEN FAILED: ' + $_.Exception.Message); continue }
    Write-Host '  open OK'

    # 1) passive listen
    try { $p.DiscardInBuffer() } catch {}
    $rx = Read-Some $p $ListenMs 0
    if ($rx.Count -gt 0) {
        Write-Host ('  LISTEN: ' + $rx.Count + ' unsolicited bytes -> ' + (Hex ($rx[0..([Math]::Min(47, $rx.Count - 1))])))
    } else { Write-Host '  LISTEN: bus silent (nothing streaming)' }

    # 2) read-only Modbus scan, function 03 over monitor block 0x0700..0x0707
    $hits = @()
    $errSeen = ''
    for ($addr = 1; $addr -le 12; $addr++) {
        $pdu = [byte[]]@(([byte]$addr), 3, 7, 0, 0, 8)
        $c = Get-Crc16 $pdu
        $frame = [byte[]]@($pdu[0], $pdu[1], $pdu[2], $pdu[3], $pdu[4], $pdu[5], ($c -band 0xFF), (($c -shr 8) -band 0xFF))
        try {
            $p.DiscardInBuffer()
            $before = -1; $after = -1
            try { $before = $p.BytesWritten } catch {}
            $p.Write($frame, 0, 8)
            try { $after = $p.BytesWritten } catch {}
            if ($before -ge 0 -and $after -le $before) { $errSeen = "slave $addr : BytesWritten did not advance" }
        } catch { $errSeen = "slave $addr : " + $_.Exception.Message; break }
        $resp = Read-Some $p 200 21
        if ($resp.Count -gt 0) {
            Write-Host ('  RESP slave ' + $addr + ' (' + $resp.Count + 'B): ' + (Hex $resp))
            $hits += $addr
        }
        Start-Sleep -Milliseconds 6
    }
    $txTotal = 'n/a'
    try { $txTotal = $p.BytesWritten } catch {}
    Write-Host ('  TX check: BytesWritten total = ' + $txTotal + '   ' + $(if ($errSeen) { '(error: ' + $errSeen + ')' } else { '' }))
    if ($hits.Count -gt 0) { Write-Host ('  >>> RESPONDING SLAVE ADDRESSES: ' + ($hits -join ', ')) }
    else { Write-Host '  >>> no Modbus reply (all 12 addresses silent)' }
    $p.Close(); $p.Dispose()
    Start-Sleep -Milliseconds 200
}
Write-Host ''
Write-Host '=== probe finished ==='
