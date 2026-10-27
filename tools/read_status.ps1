param(
    [string]$PortName = 'COM11',
    [int]$Slave = 4,
    [int]$Baud = 9600,
    [ValidateSet('One','Two')] [string]$Stop = 'Two'
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

$p = New-Object System.IO.Ports.SerialPort($PortName, $Baud, 'None', 8, $Stop)
$p.ReadTimeout = 500; $p.WriteTimeout = 500
$p.DtrEnable = $true; $p.RtsEnable = $true
$p.Open()
$raw = New-Object byte[] 512

function Read-Raw {
    $list = New-Object System.Collections.Generic.List[byte]
    $idle = [System.Diagnostics.Stopwatch]::StartNew()
    $tot = [System.Diagnostics.Stopwatch]::StartNew()
    while ($tot.ElapsedMilliseconds -lt 900) {
        $n = $p.BytesToRead
        if ($n -gt 0) {
            $got = $p.Read($raw, 0, $n)      # raw bytes, no ASCII decoding
            for ($j = 0; $j -lt $got; $j++) { $list.Add($raw[$j]) }
            $idle = [System.Diagnostics.Stopwatch]::StartNew()
        } elseif ($list.Count -gt 0 -and $idle.ElapsedMilliseconds -gt 50) { break }
        Start-Sleep -Milliseconds 1
    }
    return $list.ToArray()
}

function Read-Regs([int]$start, [int]$qty) {
    $req = [byte[]]@($Slave, 3, (($start -shr 8) -band 0xFF), ($start -band 0xFF), (($qty -shr 8) -band 0xFF), ($qty -band 0xFF), 0, 0)
    $c = Get-Crc16 $req[0..5]
    $req[6] = [byte]($c -band 0xFF); $req[7] = [byte](($c -shr 8) -band 0xFF)
    $p.DiscardInBuffer()
    $p.Write($req, 0, 8)
    $r = Read-Raw
    if ($r.Count -lt 5) { return @{ ok = $false; err = 'no reply'; raw = $r } }
    $body = [byte[]]$r[0..($r.Count - 3)]
    $cc = Get-Crc16 $body
    $tailLo = $r[$r.Count - 2]; $tailHi = $r[$r.Count - 1]
    $valid = ($cc -band 0xFF) -eq $tailLo -and (($cc -shr 8) -band 0xFF) -eq $tailHi
    if ($r[1] -ge 0x80) { return @{ ok = $false; err = ('Modbus exception 0x{0:X2}' -f $r[2]); raw = $r } }
    $vals = @()
    for ($i = 0; $i -lt $qty; $i++) {
        $u = $r[3 + $i * 2] * 256 + $r[4 + $i * 2]
        $vals += @{ Addr = ($start + $i); U = $u; S = $(if ($u -gt 32767) { $u - 65536 } else { $u }) }
    }
    return @{ ok = $true; crc = $valid; raw = $r; vals = $vals }
}

Write-Host ("$PortName @ $Baud 8-N-$Stop, slave $Slave, Modbus func 03 (read-only)")
Write-Host ''
Write-Host '########## Link quality: 20 repeats of one register ##########'
$okCrc = 0; $reply = 0
for ($t = 1; $t -le 20; $t++) {
    $res = Read-Regs 0x070B 1
    if ($res.raw.Count -gt 0) { $reply++ }
    if ($res.ok -and $res.crc) { $okCrc++ }
    Start-Sleep -Milliseconds 25
}
Write-Host ("  replies received : $reply / 20")
Write-Host ("  CRC valid        : $okCrc / $reply   (bit errors: " + ($reply - $okCrc) + ')')
Write-Host ''

Write-Host '########## Monitor group 0x0700-0x0717 ##########'
$names = @(
    '00 Fb.P  电机输入脉冲数(绝对)  脉冲', '01 Fb.r  电机输入圈数(绝对)    圈',
    '02 CP.r  脉冲命令输入脉冲数    脉冲', '03 C.r   脉冲命令输入圈数      圈',
    '04 PEr   命令与实际位置误差    脉冲', '05 CP.F  脉冲命令输入频率      KHz',
    '06 SPd   电机转速              转/分', '07 C.SPd 速度输入命令          转/分',
    '08 C.tq  扭矩输入命令          %', '09 Av.L  平均转矩                %',
    '10 PE.L  峰值转矩              %', '11 vdC   直流母线电压            V',
    '12 JL   负载/电机惯量比        -', '13 PLS.  电机输入脉冲数(相对,不支持) 脉冲',
    '14 rEv. 电机输入圈数(相对,不支持)   圈', '15 OP   电机当前控制模式        -',
    '16 dI   数字输入状态          -', '17 dO   数字输出状态            -',
    '18 rtp  电机类型              -', '19 SvEr 软件版本                -',
    '20 POS  电机编码器脉冲位置    脉冲', '21 Uv   UVW 代表的电角度        -',
    '22 AnG  磁极机械角度对应四倍频计数 脉冲', '23 td   散热器温度              ℃'
)
$m = Read-Regs 0x0700 24
if (-not $m.ok) { Write-Host ('  FAILED: ' + $m.err + '  raw=' + (HexA $m.raw)) }
else {
    Write-Host ('  CRC: ' + $(if ($m.crc) { 'VALID' } else { 'INVALID' }) + '   raw head: ' + (HexA ([byte[]]$m.raw[0..5])))
    for ($i = 0; $i -lt 24; $i++) {
        Write-Host ('   {0,-40} = {1,7}  (0x{2:X4})' -f $names[$i], $m.vals[$i].S, $m.vals[$i].U)
    }
}
Write-Host ''

Write-Host '########## Fn group 0x0600-0x060F ##########'
$f = Read-Regs 0x0600 16
if (-not $f.ok) { Write-Host ('  FAILED: ' + $f.err) }
else {
    Write-Host ('  CRC: ' + $(if ($f.crc) { 'VALID' } else { 'INVALID' }))
    $fn = @('Fn-00 用户密码', 'Fn-01 参数重置', 'Fn-02 风扇运行', 'Fn-03 电机类型', 'Fn-04 编码器线数',
            'Fn-05 通讯参数(站号/波特率)', 'Fn-06 故障清除', 'Fn-07 通讯协议', 'Fn-08 保留',
            'Fn-09 位置寸动', 'Fn-0A 速度寸动', 'Fn-0B 扭矩寸动', 'Fn-0C 速度开环寸动',
            'Fn-0D 点动编码器磁极校准', 'Fn-0E 点动自动增益调整', 'Fn-0F 校正功能选择')
    for ($i = 0; $i -lt 16; $i++) {
        Write-Host ('   {0,-34} = {1,7}  (0x{2:X4})' -f $fn[$i], $f.vals[$i].S, $f.vals[$i].U)
    }
}
Write-Host ''

Write-Host '########## Probe which parameter groups are readable ##########'
foreach ($a in 0x0000, 0x0100, 0x0200, 0x0300, 0x0400, 0x0500, 0x0600, 0x0700) {
    $t = Read-Regs $a 1
    if ($t.ok) { Write-Host ('   0x{0:X4}: readable, value={1}' -f $a, $t.vals[0].S) }
    else { Write-Host ('   0x{0:X4}: {1}   raw={2}' -f $a, $t.err, (HexA $t.raw)) }
    Start-Sleep -Milliseconds 30
}
$p.Close(); $p.Dispose()
Write-Host ''
Write-Host '=== done ==='
