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

function Read-Regs([int]$start, [int]$qty) {
    $req = [byte[]]@($Slave, 3, (($start -shr 8) -band 0xFF), ($start -band 0xFF), (($qty -shr 8) -band 0xFF), ($qty -band 0xFF), 0, 0)
    $c = Get-Crc16 $req[0..5]
    $req[6] = [byte]($c -band 0xFF); $req[7] = [byte](($c -shr 8) -band 0xFF)
    $p.DiscardInBuffer()
    $p.Write($req, 0, 8)
    $r = Read-Raw
    if ($r.Count -lt 5) { return @{ ok = $false; err = 'no reply'; raw = $r } }
    if ($r[1] -ge 0x80) { return @{ ok = $false; err = ('Modbus exception 0x{0:X2}' -f $r[2]); raw = $r } }
    $body = [byte[]]$r[0..($r.Count - 3)]
    $cc = Get-Crc16 $body
    $valid = (($cc -band 0xFF) -eq $r[$r.Count - 2]) -and ((($cc -shr 8) -band 0xFF) -eq $r[$r.Count - 1])
    $vals = @()
    for ($i = 0; $i -lt $qty; $i++) {
        $u = $r[3 + $i * 2] * 256 + $r[4 + $i * 2]
        $vals += @{ Addr = ($start + $i); U = $u; S = $(if ($u -gt 32767) { $u - 65536 } else { $u }) }
    }
    return @{ ok = $true; crc = $valid; raw = $r; vals = $vals }
}

function Dump-Group([string]$title, [int]$start, [string[]]$labels) {
    Write-Host ''
    Write-Host "########## $title ##########"
    $g = Read-Regs $start $labels.Count
    if (-not $g.ok) { Write-Host ('  FAILED: ' + $g.err + '  raw=' + (HexA $g.raw)); return }
    Write-Host ('  CRC: ' + $(if ($g.crc) { 'VALID' } else { 'INVALID' }))
    for ($i = 0; $i -lt $labels.Count; $i++) {
        if ([string]::IsNullOrWhiteSpace($labels[$i])) { continue }
        Write-Host ('   {0,-44} = {1,7}  (0x{2:X4})' -f $labels[$i], $g.vals[$i].S, $g.vals[$i].U)
    }
}

Write-Host "$PortName @ $Baud 8-N-2, slave $Slave, Modbus func 03 - READ ONLY, no writes"

Dump-Group 'Pn-00x 系统寄存器 (控制模式/使能来源)' 0x0000 @(
    'Pn-000 功能选择基本开关 [bit0方向 bit1模式 bit2使能源]', '', '', '', '', '', '', '',
    'Pn-008', 'Pn-009', 'Pn-00A', 'Pn-00B', 'Pn-00C', 'Pn-00D', 'Pn-00E', 'Pn-00F'
)

Dump-Group 'Pn-30x 速度相关 (内部设定速度/速度限制)' 0x0300 @(
    'Pn-300 模拟速度指令比例器', 'Pn-301 内部设定速度1 (0.2转/分)', 'Pn-302 内部设定速度2 (0.2转/分)',
    'Pn-303 内部设定速度3 (0.2转/分)', 'Pn-304 内部设定速度4 内部速度模式默认 (0.2转/分)',
    'Pn-305 保留', 'Pn-306 保留', 'Pn-307 模拟速度滤波时间', 'Pn-308 速度反馈滤波时间',
    'Pn-309 保留', 'Pn-30A 保留', 'Pn-30B 模拟速度有效起始电压门槛', 'Pn-30C 模拟速度电压极性',
    'Pn-30D 内部速度限制 (0.2转/分) <<< 上限保护', 'Pn-30E 外部速度限制1', 'Pn-30F 外部速度限制2'
)

Dump-Group 'Pn-40x 位置相关' 0x0400 @(
    'Pn-400 位置指令加减速时间常数', 'Pn-401', 'Pn-402', 'Pn-403', 'Pn-404', 'Pn-405', 'Pn-406', 'Pn-407 扭矩控制时速度限制'
)

Dump-Group 'Fn-0x 辅助功能 (含速度寸动)' 0x0600 @(
    'Fn-00 用户密码', 'Fn-01 参数重置', 'Fn-02 风扇运行', 'Fn-03 电机类型', 'Fn-04 编码器线数',
    'Fn-05 通讯参数(站号/波特率)', 'Fn-06 故障清除', 'Fn-07 通讯协议', 'Fn-08 保留',
    'Fn-09 位置寸动', 'Fn-0A 速度寸动', 'Fn-0B 扭矩寸动', 'Fn-0C 速度开环寸动',
    'Fn-0D 点动编码器磁极校准', 'Fn-0E 点动自动增益调整', 'Fn-0F 校正功能选择'
)

Dump-Group '监控 0x0700-0x0717' 0x0700 @(
    '00 Fb.P  电机输入脉冲数', '01 Fb.r  电机输入圈数', '02 CP.r  命令脉冲数', '03 C.r  命令圈数',
    '04 PEr  位置偏差', '05 CP.F  命令频率', '06 SPd  电机转速', '07 C.SPd 速度输入命令',
    '08 C.tq 扭矩输入命令', '09 Av.L 平均转矩', '10 PE.L 峰值转矩', '11 vdC 直流母线电压',
    '12 JL 惯量比', '13 PLS.', '14 rEv.', '15 OP 电机当前控制模式', '16 dI 数字输入状态 <<< 看SON',
    '17 dO 数字输出状态', '18 rtp 电机类型', '19 SvEr 软件版本', '20 POS 编码器位置', '21 Uv 电角度',
    '22 AnG 机械角度', '23 td 散热器温度'
)

$p.Close(); $p.Dispose()
Write-Host ''
Write-Host '=== done (read only) ==='
