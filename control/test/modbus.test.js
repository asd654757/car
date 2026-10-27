import { test } from 'node:test';
import assert from 'node:assert/strict';
import { crc16, readRequest, writeRequest, parseReadResponse, parseWriteResponse, toU16, toI16, asU16, exceptionText } from '../src/modbus.js';

test('CRC-16/MODBUS 标准校验值：ASCII "123456789" → 0x4B37', () => {
  assert.equal(crc16(Buffer.from('123456789')), 0x4b37);
});

test('读请求帧逐字节正确（从站1 起址0 数量1 → 01 03 00 00 00 01 84 0A）', () => {
  assert.deepEqual([...readRequest(1, 0x0000, 1)], [0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0a]);
});

test('读监控组：从站4 起 0x0700 数量 24', () => {
  const f = readRequest(4, 0x0700, 24);
  assert.equal(f.length, 8);
  assert.deepEqual([...f.subarray(0, 6)], [0x04, 0x03, 0x07, 0x00, 0x00, 0x18]);
  assert.equal(f.readUInt16LE(6), crc16(f, 6));
});

test('写请求：负数按二进制补码下发（-300 → 0xFED4）', () => {
  const f = writeRequest(3, 0x0303, -300);
  assert.deepEqual([...f.subarray(0, 6)], [0x03, 0x06, 0x03, 0x03, 0xfe, 0xd4]);
  assert.equal(f.readUInt16LE(6), crc16(f, 6));
});

test('写请求：正数与边界', () => {
  assert.deepEqual([...writeRequest(1, 0x0303, 300).subarray(4, 6)], [0x01, 0x2c]);
  assert.deepEqual([...writeRequest(1, 0x0303, 0).subarray(4, 6)], [0x00, 0x00]);
  assert.deepEqual([...writeRequest(1, 0x0303, -32768).subarray(4, 6)], [0x80, 0x00]);
  assert.throws(() => writeRequest(1, 0x0303, 32768 + 32768), /16 位范围/);
  assert.throws(() => writeRequest(1, 0x0303, 1.5), /整数/);
});

test('toU16/toI16 往返', () => {
  [-300, -1, 0, 1, 300, 32767].forEach((v) => assert.equal(toI16(toU16(v)), v));
});

test('有符号读回在 0~30000 区间是无损的：Pn-30D 出厂默认 30000 不会变成负数', () => {
  // 记这条是为了拦住"以后有人拿 asU16 去'修'限幅读数"：toI16 的像只落在 -32768~32767，
  // 30000 < 32767 → 原样读回。真读回负数就说明这帧不是 Pn-30D，那是证据，不是符号问题。
  assert.equal(toI16(30000), 30000);
  assert.equal(toI16(0), 0);
  assert.equal(toI16(32767), 32767);
  assert.equal(toI16(32768), -32768);
});

test('asU16 只给位域参数用（Pn-000 的 0x8130 读回是负数，归一后才能取 nibble）', () => {
  assert.equal(asU16(toI16(0x8130)), 0x8130);
  assert.equal(asU16(300), 300, '本来就是正数时不动它');
  assert.equal((asU16(toI16(0x8130)) >> 8) & 0xff, 0x81, '用有符号值右移会搬进符号位');
});

test('读响应解析：正常帧 → 有符号寄存器表', () => {
  const body = Buffer.from([0x04, 0x03, 0x04, 0xff, 0xe4, 0x00, 0x64]); // 2 个寄存器：-28, +100
  const f = Buffer.concat([body, Buffer.alloc(2)]);
  f.writeUInt16LE(crc16(f, 7), 7);
  const r = parseReadResponse(f, 4, 0x0706, 2);
  assert.equal(r.ok, true);
  assert.equal(r.regs[0x0706], -28);
  assert.equal(r.regs[0x0707], 100);
});

test('读响应解析：异常 0x04（跨组读）要认出来', () => {
  const f = Buffer.from([0x04, 0x83, 0x04, 0, 0]);
  f.writeUInt16LE(crc16(f, 3), 3);
  const r = parseReadResponse(f, 4, 0x0000, 1);
  assert.equal(r.ok, false);
  assert.equal(r.exception, 4);
  assert.match(exceptionText(4), /跨参数组/);
});

test('读响应解析：CRC 错 / 从站号错 / 长度不足 全部拒绝', () => {
  // 一帧合法应答：从站 5 回 0x0706 = 100
  const body = Buffer.from([0x05, 0x03, 0x02, 0x00, 0x64]);
  const good = Buffer.concat([body, Buffer.alloc(2)]);
  good.writeUInt16LE(crc16(good, 5), 5);
  assert.equal(parseReadResponse(good, 5, 0x0706, 1).ok, true, '帧本身要能过');

  // 我们问的是 4 号，回来的却是 5 号 → 报错要说"收到的是谁"，这才是"有人抢答"的线索
  assert.equal(parseReadResponse(good, 6, 0x0706, 1).error, 'slave-mismatch:5');

  const bad = Buffer.from(good); bad[4] ^= 0xff; // 只改数据字节，别撞坏从站号
  assert.match(parseReadResponse(bad, 5, 0x0706, 1).error, /^crc/);
  assert.match(parseReadResponse(good.subarray(0, 5), 5, 0x0706, 1).error, /truncated/);
  assert.match(parseReadResponse(null, 5, 0x0706, 1).error, /no-response/);
});

test('"一个字节都没回来"和"回来但太短"必须是两句话（2026-10-09 那次全靠这句分辨）', () => {
  const req = writeRequest(1, 0x030d, 15000);
  assert.match(parseWriteResponse(null, req).error, /no-response/);
  assert.match(parseWriteResponse(Buffer.from([0x01, 0x06, 0x00]), req).error, /short-response/);
  assert.match(parseWriteResponse(Buffer.from([0x01, 0x06, 0x00]), req).error, /\[010600\]/, '要把原始字节打出来');
});

test('写响应：5 字节异常帧要认成异常，不许被"长度不够 8 字节"吞掉', () => {
  const req = writeRequest(1, 0x030d, 15000);
  for (const [code, why] of [[2, /地址非法/], [3, /值非法/], [6, /正在运行中/]]) {
    const f = Buffer.from([0x01, 0x86, code, 0x00, 0x00]);
    f.writeUInt16LE(crc16(f, 3), 3);
    const r = parseWriteResponse(f, req);
    assert.equal(r.ok, false);
    assert.equal(r.exception, code);
    assert.match(r.error, /^exception 0x0./);
    assert.match(r.error, why, `异常码 ${code} 要说清人话：${r.error}`);
  }
});

test('读响应：异常帧同样先于长度判定，且带原始字节', () => {
  const f = Buffer.from([0x04, 0x83, 0x04, 0x00, 0x00]);
  const r = parseReadResponse(f, 4, 0x0619, 8);
  assert.equal(r.exception, 4);
  assert.match(r.error, /跨参数组/);
  assert.match(r.error, /\[0483040000\]/);
});

test('写响应：驱动器回显请求帧', () => {
  const req = writeRequest(4, 0x0303, 150);
  assert.equal(parseWriteResponse(req, req).ok, true);
  const tampered = Buffer.from(req); tampered[5] ^= 0xff;
  assert.match(parseWriteResponse(tampered, req).error, /^echo-mismatch/);
  const exc = Buffer.from([0x04, 0x86, 0x02, 0, 0, 0, 0, 0]);
  assert.equal(parseWriteResponse(exc, req).exception, 2);
  const wrongSlave = Buffer.from(req); wrongSlave[0] = 5;
  assert.match(parseWriteResponse(wrongSlave, req).error, /slave-mismatch:5/);
});
