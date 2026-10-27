/**
 * Modbus RTU 帧构造与解析 —— 纯函数，无 IO。
 * 与实机验证过的 PowerShell 工具（tools/ws_speed_cmd.ps1）逐字节等价，可直接翻成 C。
 *
 * 实机约束（都踩过）：
 *  - CRC-16/MODBUS：多项式 0xA001，初值 0xFFFF，**低字节在前**。
 *  - 跨参数组读会返回异常 0x04；0x0000 组一次最多读 8 个。
 *  - 功能码 0x83/0x86 = 异常响应（第 2 字节 ≥ 0x80）。
 */

export const FUNC_READ = 0x03;
export const FUNC_WRITE = 0x06;

/** CRC-16/MODBUS */
export function crc16(buf, len = buf.length) {
  let crc = 0xffff;
  for (let i = 0; i < len; i++) {
    crc ^= buf[i] & 0xff;
    for (let b = 0; b < 8; b++) {
      crc = (crc & 1) ? ((crc >>> 1) ^ 0xa001) : (crc >>> 1);
    }
  }
  return crc & 0xffff;
}

/** 有符号 16 位 → 无符号 16 位（写寄存器用） */
export function toU16(v) {
  if (!Number.isInteger(v)) throw new Error(`写值必须是整数，收到 ${v}`);
  if (v < -32768 || v > 65535) throw new Error(`写值超出 16 位范围: ${v}`);
  return v < 0 ? v + 65536 : v;
}

/** 无符号 16 位 → 有符号（读回来的寄存器） */
export function toI16(u) { return u > 32767 ? u - 65536 : u; }

/**
 * 读回来的寄存器按**无符号 16 位**重新解释（只用于取回高位/位域）。
 * 用在 Pn-000 这类位域参数上：0xF000 读回来是 -4096，直接 >> 会搬进高位 1，必须先归一。
 * 普通数值参数不需要它 —— Pn-30D 手册范围 0~30000，30000 < 32767，有符号读回来还是 30000，
 * toI16 是无损的（2026-10-09 我先写下这条时以为 30000 会变负数，算了一遍：不会，别照那个说法改）。
 * 反过来，**读回来是负数的数值参数就是证据**：说明这帧不是我们要的那个参数，别归一成 65535 当真值用。
 */
export function asU16(v) { return Number.isFinite(v) && v < 0 ? v + 65536 : v; }

/** 读保持寄存器请求帧（功能码 03） */
export function readRequest(slave, startAddr, qty) {
  if (qty < 1 || qty > 125) throw new Error(`读取数量非法: ${qty}`);
  const f = Buffer.alloc(8);
  f[0] = slave; f[1] = FUNC_READ;
  f.writeUInt16BE(startAddr & 0xffff, 2);
  f.writeUInt16BE(qty & 0xffff, 4);
  f.writeUInt16LE(crc16(f, 6), 6);
  return f;
}

/** 写单个寄存器请求帧（功能码 06）。value 允许负数，自动转补码。 */
export function writeRequest(slave, addr, value) {
  const f = Buffer.alloc(8);
  f[0] = slave; f[1] = FUNC_WRITE;
  f.writeUInt16BE(addr & 0xffff, 2);
  f.writeUInt16BE(toU16(value), 4);
  f.writeUInt16LE(crc16(f, 6), 6);
  return f;
}

/**
 * 解析读响应 → { ok, regs:{addr:value}, exception, error }
 * regs 用地址作键，值为有符号 16 位。
 */
export function parseReadResponse(buf, slave, startAddr, qty) {
  if (!buf) return { ok: false, error: 'no-response', raw: buf };
  if (buf[0] !== slave) return { ok: false, error: `slave-mismatch:${buf[0]}`, raw: buf };
  if (isException(buf)) return exceptionResult(buf);
  const byteCount = buf[2];
  if (byteCount !== qty * 2) return { ok: false, error: `byte-count:${byteCount} ${frameNote(buf)}`, raw: buf };
  const need = 3 + byteCount + 2;
  if (buf.length < need) return { ok: false, error: `truncated ${frameNote(buf)}`, raw: buf };
  const crcGot = buf.readUInt16LE(need - 2);
  if (crcGot !== crc16(buf, need - 2)) return { ok: false, error: `crc ${frameNote(buf)}`, raw: buf };
  const regs = {};
  for (let i = 0; i < qty; i++) regs[startAddr + i] = toI16(buf.readUInt16BE(3 + i * 2));
  return { ok: true, regs };
}

/** 异常响应：功能码最高位置 1（0x03→0x83、0x06→0x86），整帧只有 5 字节 */
function isException(buf) { return buf.length >= 3 && buf[1] >= 0x80; }
function exceptionResult(buf) {
  const code = buf[2];
  return { ok: false, exception: code, error: `exception 0x${code.toString(16).padStart(2, '0')} ${exceptionText(code)} ${frameNote(buf)}`, raw: buf };
}

/** 把回来的字节原样打出来 —— 现场只有一行文字时，这串十六进制就是唯一的证据 */
export function frameNote(buf) {
  if (!buf || !buf.length) return '无响应（超时，一个字节都没回来）';
  return `收到 ${buf.length} 字节 [${buf.toString('hex').toUpperCase()}]`;
}

/**
 * 解析写响应（驱动器回显请求帧）→ { ok }
 * ⚠ 顺序很重要：**先认异常帧，再判长度**。异常帧只有 5 字节，如果先判 `length < 8`，
 * 那"这台驱动器为什么拒绝你写"就被压成一句 short-response 了
 * （2026-10-09 写 Pn-30D 四台全报 short-response，就是这么把异常码丢掉的）。
 */
export function parseWriteResponse(buf, request) {
  if (!buf) return { ok: false, error: 'no-response', raw: buf };
  if (buf[0] !== request[0]) return { ok: false, error: `slave-mismatch:${buf[0]} ${frameNote(buf)}`, raw: buf };
  if (isException(buf)) return exceptionResult(buf);
  if (buf.length < 8) return { ok: false, error: `short-response ${frameNote(buf)}`, raw: buf };
  for (let i = 0; i < 6; i++) if (buf[i] !== request[i]) return { ok: false, error: `echo-mismatch ${frameNote(buf)}`, raw: buf };
  const crcGot = buf.readUInt16LE(6);
  if (crcGot !== crc16(buf, 6)) return { ok: false, error: `crc ${frameNote(buf)}`, raw: buf };
  return { ok: true };
}

/** 异常码 → 人话（手册第 6 节 + 参数操作结果说明） */
export function exceptionText(code) {
  return {
    1: '功能码非法',
    2: '地址非法（参数号/组别不对）',
    3: '值非法（Out-r：超出该参数设定范围）',
    4: '跨参数组读取/从站失效',
    6: '伺服器正在运行中，不能修改该数据（Sd-On：先关使能再写）',
  }[code] || `异常 0x${(code || 0).toString(16)}`;
}
