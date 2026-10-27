import { test } from 'node:test';
import assert from 'node:assert/strict';
import { EventEmitter } from 'node:events';
import { RtuBus } from '../src/bus.js';
import { CONFIG } from '../src/config.js';
import { crc16, readRequest } from '../src/modbus.js';

/* ---------------- 假串口：按 Modbus RTU 规则应答，可注入延迟/丢帧/坏帧 ---------------- */
const state = {};
function resetState(o = {}) {
  Object.assign(state, {
    delayMs: 0, chunked: false, silent: false, corrupt: false, overlap: 0,
    regs: new Map(), written: [], active: new Set(), maxConcurrent: 0, frames: [], ...o,
  });
}
const defaultReg = (slave, addr) => (addr === CONFIG.regs.mode ? 0x0130 : addr === CONFIG.regs.ceiling ? 300 : addr === CONFIG.regs.mon.spd ? 59 : 0);
const regValue = (slave, addr) => {
  const k = `${slave}:${addr}`;
  return state.regs.has(k) ? state.regs.get(k) : defaultReg(slave, addr);
};
const slaveOf = (path) => CONFIG.wheels.find((w) => w.port === path).slave;

function buildResponse(req) {
  const slave = req[0];
  if (req.readUInt16LE(6) !== crc16(req, 6)) return null; // 坏帧驱动器不应答
  if (req[1] === 0x03) {
    const start = req.readUInt16BE(2), qty = req.readUInt16BE(4);
    if (qty > 8 && start < 0x0100) { // 0x0000 组实机只能一次读 ≤8
      const e = Buffer.from([slave, 0x83, 0x04]);
      return Buffer.concat([e, crcBuf(e)]);
    }
    const body = Buffer.alloc(3 + qty * 2);
    body[0] = slave; body[1] = 0x03; body[2] = qty * 2;
    for (let i = 0; i < qty; i++) body.writeUInt16BE(regValue(slave, start + i) & 0xffff, 3 + i * 2);
    return Buffer.concat([body, crcBuf(body)]);
  }
  if (req[1] === 0x06) {
    state.regs.set(`${slave}:${req.readUInt16BE(2)}`, req.readUInt16BE(4));
    state.written.push({ slave, addr: req.readUInt16BE(2), value: req.readInt16BE(4) });
    return Buffer.from(req); // 回显
  }
  return null;
}
const crcBuf = (b) => { const c = Buffer.alloc(2); c.writeUInt16LE(crc16(b, b.length), 0); return c; };

class FakeSerialPort extends EventEmitter {
  constructor(opts) {
    super();
    this.path = opts.path;
    this.opts = opts;
    this.isOpen = false;
    FakeSerialPort.instances.push(this);
  }
  open(cb) { this.isOpen = true; setImmediate(() => cb(null)); }
  close(cb) { this.isOpen = false; setImmediate(() => cb(null)); }
  drain(cb) { setImmediate(() => cb()); }
  write(buf, cb) {
    cb(null);
    if (state.silent) return;
    const resp = buildResponse(buf);
    if (!resp) return;
    if (state.active.has(this.path)) state.overlap++; // 同口叠帧 = 半双工被打断
    state.frames.push({ path: this.path, req: Buffer.from(buf) });
    state.active.add(this.path);
    state.maxConcurrent = Math.max(state.maxConcurrent, state.active.size);
    const out = Buffer.from(resp);
    if (state.corrupt) out[4] ^= 0xff; // 改数据字节 → CRC 对不上，但帧长不变
    setTimeout(() => {
      state.active.delete(this.path);
      if (state.chunked) {
        this.emit('data', out.subarray(0, 3));
        setTimeout(() => this.emit('data', out.subarray(3)), 3);
      } else this.emit('data', out);
    }, state.delayMs);
  }
}
FakeSerialPort.instances = [];

async function openedBus(opts = {}) {
  resetState(opts.state);
  FakeSerialPort.instances.length = 0;
  const bus = new RtuBus(CONFIG.wheels, { baud: 9600, timeoutMs: opts.timeoutMs ?? 300, SerialPortCtor: FakeSerialPort });
  await bus.open();
  return bus;
}

/* ---------------- 用例 ---------------- */
test('串口参数按实机：9600 / 8-N-2，一口一从站（物理通道 n == 站号 n）', async () => {
  const bus = await openedBus();
  assert.equal(FakeSerialPort.instances.length, 4, '四个口各开一次');
  FakeSerialPort.instances.forEach((p, i) => {
    assert.equal(p.opts.path, CONFIG.wheels[i].port);
    assert.equal(p.opts.baudRate, 9600);
    assert.equal(p.opts.dataBits, 8);
    assert.equal(p.opts.stopBits, 2, '这台驱动器是 8-N-2，stopBits 写成 1 就整条总线读不通');
    assert.equal(p.opts.parity, 'none');
    assert.equal(p.opts.autoOpen, false);
    assert.equal(slaveOf(p.path), CONFIG.wheels[i].slave);
  });
  await bus.close();
});

test('第三个口打不开 → 报错点名是哪个轮哪个口，并把已经开着的口全部关回去', async () => {
  resetState();
  FakeSerialPort.instances.length = 0;
  const bad = CONFIG.wheels[2]; // 下标 2 = BL 左后
  class FlakyFake extends FakeSerialPort {
    open(cb) {
      if (this.path === bad.port) { setImmediate(() => cb(new Error('Access denied'))); return; }
      super.open(cb);
    }
  }
  const bus = new RtuBus(CONFIG.wheels, { baud: 9600, timeoutMs: 300, SerialPortCtor: FlakyFake });
  await assert.rejects(() => bus.open(), (e) => {
    assert.ok(e.message.includes(bad.port), `报错里要带上打不开的那个口名：${e.message}`);
    assert.match(e.message, /BL 左后 站2/, '要能一眼看出是哪一个轮');
    assert.match(e.message, /--list-ports/, '要给下一步动作，不许只丢一句 File not found');
    return true;
  });
  const stillOpen = FakeSerialPort.instances.filter((p) => p.isOpen);
  assert.deepEqual(stillOpen.map((p) => p.path), [], '半途失败也要关干净：一个口只能被一个进程占，留着就是下次启动 Access denied');
});

test('读 Pn-000：请求帧逐字节对，应答解成有符号寄存器', async () => {
  const bus = await openedBus();
  const r = await bus.readRegs(0, CONFIG.regs.mode, 1);
  assert.deepEqual([...state.frames[0].req], [0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0a]);
  assert.equal(r.ok, true);
  assert.equal(r.regs[CONFIG.regs.mode], 0x0130);
  await bus.close();
});

test('写 Pn-303 负数：驱动器回显，且它自己读回来的也是这个值', async () => {
  const bus = await openedBus();
  const w = await bus.writeReg(2, CONFIG.regs.speed, -300); // 下标 2 = BL 左后 = COM13/站2
  assert.equal(w.ok, true);
  assert.deepEqual([...state.frames[0].req.slice(4, 6)], [0xfe, 0xd4]);
  assert.deepEqual(state.written, [{ slave: 2, addr: 0x0303, value: -300 }]);
  const back = await bus.readRegs(2, CONFIG.regs.speed, 1);
  assert.equal(back.regs[CONFIG.regs.speed], -300, '写后回读：这是实机每次都要做的那一步');
  await bus.close();
});

test('半包应答要能拼回来（9600 波特下一帧常被拆成多段送上来）', async () => {
  const bus = await openedBus({ state: { chunked: true } });
  const r = await bus.readRegs(3, CONFIG.regs.monitor, CONFIG.regs.monitorQty);
  assert.equal(r.ok, true);
  assert.equal(r.regs[CONFIG.regs.mon.spd], 59);
  await bus.close();
});

test('整块读监控组：0x0700 起 24 个，一次拿全（跨组读会被拒）', async () => {
  const bus = await openedBus({ timeoutMs: 80 });
  const r = await bus.readRegs(1, CONFIG.regs.monitor, CONFIG.regs.monitorQty);
  assert.equal(r.ok, true);
  assert.equal(Object.keys(r.regs).length, CONFIG.regs.monitorQty);
  const bad = await bus.readRegs(1, 0x0000, 24); // 越界读 0x0000 组
  assert.equal(bad.ok, false);
  assert.equal(bad.exception, 4, '实机就是回异常 0x04，得认出来');
  await bus.close();
});

test('无应答 ≠ 抛异常：超时后返回失败，控制环才不会崩', async () => {
  const bus = await openedBus({ state: { silent: true }, timeoutMs: 60 });
  const r = await bus.readRegs(0, CONFIG.regs.mode, 1);
  assert.equal(r.ok, false);
  assert.match(r.error, /no-response/, '一个字节都没回来要说成"无响应"，不能和"回来但太短"混成一句');
  await bus.close();
});

test('CRC 坏帧被拒绝，不会被当成"读到了 0"', async () => {
  const bus = await openedBus({ state: { corrupt: true } });
  const r = await bus.readRegs(0, CONFIG.regs.mode, 1);
  assert.equal(r.ok, false);
  assert.match(r.error, /^crc/, '坏帧要报 crc，并且带上原始字节');
  await bus.close();
});

test('EEPROM 参数写用长超时，控制环那一类写不许被拖长', async () => {
  // 假驱动器 120 ms 后才回写应答：短超时（40 ms）拿不到，长超时（4×40 = 160 ms）拿得到。
  const bus = await openedBus({ state: { delayMs: 120 }, timeoutMs: 40 });
  const slow = await bus.writeReg(0, CONFIG.regs.speed, 100);
  assert.equal(slow.ok, false, '默认超时没变长 → 控制环掉线时仍然 40 ms 就判失败，看门狗不会被参数写拖钝');
  const ee = await bus.writeReg(0, CONFIG.regs.ceiling, 15000, { eeprom: true });
  assert.equal(ee.ok, true, '参数写等得起：EEPROM 是"先落盘后应答"');
  assert.equal(bus.channels[0].writeTimeoutMs, 160);
  await bus.close();
});

test('同一通道必须串行（RS485 半双工，两帧叠在一起就全废）', async () => {
  const bus = await openedBus({ state: { delayMs: 20 } });
  const ch = bus.channels[0];
  await Promise.all([
    ch._transact(readRequest(1, CONFIG.regs.mode, 1), 7),
    ch._transact(readRequest(1, CONFIG.regs.mon.spd, 1), 7),
  ]);
  assert.equal(state.frames.length, 2, '两帧都发了');
  assert.equal(state.overlap, 0, '但没有叠帧：第二帧等第一帧收完才发');
  assert.equal(ch.stats.tx, 2);
  await bus.close();
});

test('四个通道并行下发（整机一拍 35~45ms 的依据）', async () => {
  const bus = await openedBus({ state: { delayMs: 40 } });
  const t0 = Date.now();
  const cmd = [300, 300, -300, -300]; // 前进那一行
  const res = await bus.writeMany(CONFIG.regs.speed, CONFIG.wheels.map((w, i) => ({ idx: i, value: cmd[i] })));
  const dt = Date.now() - t0;
  assert.deepEqual(res.map((r) => r.ok), [true, true, true, true]);
  assert.equal(state.maxConcurrent, 4, '四路要真的同时在发');
  assert.ok(dt < 4 * 40, `并行而不是串行：${dt}ms`);
  const bySlave = Object.fromEntries(state.written.map((x) => [x.slave, x.value]));
  assert.deepEqual(bySlave, { 1: 300, 4: 300, 2: -300, 3: -300 }, '站 1/4（前轮）正、站 2/3（后轮）负 = 14:58 自检那组');
  assert.ok(state.written.every((x) => x.addr === CONFIG.regs.speed), '全部只写 Pn-303');
  await bus.close();
});

test('某一路没应答，不牵连其他三路（writeMany 逐个报结果）', async () => {
  resetState({ delayMs: 5, silent: false });
  FakeSerialPort.instances.length = 0;
  const bus = new RtuBus(CONFIG.wheels, { timeoutMs: 40, SerialPortCtor: FakeSerialPort });
  await bus.open();
  const dead = bus.channels[1];
  dead.port.write = (buf, cb) => cb(null); // 13 号口静默
  const res = await bus.writeMany(CONFIG.regs.speed, [{ idx: 0, value: 100 }, { idx: 1, value: -100 }, { idx: 2, value: 100 }]);
  assert.deepEqual(res.map((r) => `${r.idx}:${r.ok}`), ['0:true', '1:false', '2:true']);
  assert.match(res[1].error, /no-response/);
  await bus.close();
});
