param(
    [string]$PortName = 'COM11',
    [int]$Slave = 4,
    [int]$Baud = 9600,
    [string]$Writes = '',
    [switch]$Yes
)

# Modbus func 06 writer with pre/post readback. Allow-listed addresses only.
$ALLOW = @(0x0000, 0x0301, 0x0302, 0x0303, 0x0304, 0x030D)

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

function Parse-Reply([byte[]]$r) {
    if ($r.Count -lt 5) { return @{ ok = $false; err = 'NO REPLY' } }
    if ($r[1] -ge 0x80) { return @{ ok = $false; err = ('EXC 0x{0:X2}' -f $r[2]) } }
    $body = [byte[]]$r[0..($r.Count - 3)]
    $cc = Get-Crc16 $body
    $valid = (($cc -band 0xFF) -eq $r[$r.Count - 2]) -and ((($cc -shr 8) -band 0xFF) -eq $r[$r.Count - 1])
    if (-not $valid) { return @{ ok = $false; err = 'CRC BAD' } }
    $u = $r[3] * 256 + $r[4]
    $s = $(if ($u -gt 32767) { $u - 65536 } else { $u })
    return @{ ok = $true; U = $u; S = $s }
}

function Read-One([int]$addr) {
    $req = [byte[]]@($Slave, 3, (($addr -shr 8) -band 0xFF), ($addr -band 0xFF), 0, 1, 0, 0)
    $c = Get-Crc16 $req[0..5]
    $req[6] = [byte]($c -band 0xFF); $req[7] = [byte](($c -shr 8) -band 0xFF)
    $p.DiscardInBuffer(); $p.Write($req, 0, 8)
    return Parse-Reply (Read-Raw)
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
        return @{ ok = $same; err = $(if ($same) { '' } else { 'ECHO MISMATCH' }); raw = $r }
    }
    if ($r.Count -ge 3 -and $r[1] -ge 0x80) { return @{ ok = $false; err = ('EXC 0x{0:X2}' -f $r[2]); raw = $r } }
    return @{ ok = $false; err = 'NO VALID REPLY'; raw = $r }
}

Write-Host "slave $Slave @ $PortName $Baud 8-N-2 - func 06 WRITE tool"
Write-Host 'allow-list: Pn-000(0x0000) Pn-301..304(0x0301-0x0304) Pn-30D(0x030D)'
Write-Host ''

if ([string]::IsNullOrWhiteSpace($Writes)) {
    Write-Host 'ERROR: -Writes "0x030D:300,0x0000:19" required'; $p.Close(); exit 1
}
if (-not $Yes) {
    Write-Host 'DRY RUN - no frames sent. Re-run with -Yes to actually write.'
    Write-Host ''
}

$items = $Writes -split ','
foreach ($it in $items) {
    $parts = $it -split ':'
    if ($parts.Count -ne 2) { Write-Host ("  skip bad item: $it"); continue }
    $addr = [Convert]::ToInt32($parts[0].Trim(), 16)
    $val = [int]$parts[1].Trim()
    if ($ALLOW -notcontains $addr) { Write-Host ('  BLOCKED: 0x{0:X4} not in allow-list' -f $addr); continue }

    $pre = Read-One $addr
    if (-not $pre.ok) { Write-Host ('  ABORT: cannot pre-read 0x{0:X4} -> {1}' -f $addr, $pre.err); $p.Close(); exit 2 }
    $rpmHint = ''
    if ($addr -ge 0x0301 -and $addr -le 0x030F) { $rpmHint = ('  (~{0} rpm if unit 0.2)' -f ($val * 0.2)) }
    Write-Host ('  0x{0:X4} current={1}  ->  write {2}{3}' -f $addr, $pre.S, $val, $rpmHint)

    if ($Yes) {
        $w = Write-One $addr $val
        if (-not $w.ok) {
            $msg = '  WRITE FAILED: ' + $w.err + '  raw=' + (HexA $w.raw) + '   -> stop here'
            Write-Host $msg; $p.Close(); exit 3
        }
        Start-Sleep -Milliseconds 120
        $post = Read-One $addr
        if (-not $post.ok) { Write-Host ('  READBACK FAILED: ' + $post.err); $p.Close(); exit 4 }
        if ($post.S -ne $val) {
            Write-Host ('  READBACK MISMATCH: wrote ' + $val + ' but read ' + $post.S + '   -> stop here, do not continue')
            $p.Close(); exit 5
        }
        Write-Host ('  verified: 0x{0:X4} = {1}' -f $addr, $post.S)
    }
    Start-Sleep -Milliseconds 60
}

Write-Host ''
if ($Yes) { Write-Host 'all writes verified' } else { Write-Host 'nothing written (dry run)' }
$p.Close(); $p.Dispose()
