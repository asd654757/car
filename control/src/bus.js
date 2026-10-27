/**
 * RS485 / Modbus RTU 传输层。一口一从站（WCH CH9114：COM12/13/14/15 = 1/2/3/4 号）。
 *
 * 实机约束：
 *  - 一个 COM 口只能被一个进程占用 → 本模块独占四个口。
 *  - 单帧 round-trip 实测 25~46 ms → 四轮用四个通道**并行**发，整机一拍 35~45 ms。
 *  - 每通道同一时刻只允许一帧在途，所以内部做了串行队列。
 *
 * serialport 只在 open() 时动态 import，这样 --dry-run / 单元测试不装依赖也能跑。
 */

export class Channel {
  constructor(portPath, slave, { baud = 9600, timeoutMs = 300, writeTimeoutMs = 0 } = {}) {
    this.portPath = portPath;
    this.slave = slave;
    this.baud = baud;
    this.timeoutMs = timeoutMs;
    // EEPROM 保持参数（Pn-30D 这类）要等写完闪存的应答，实测会比普通读慢一截；
    // 只把"写"的等待拉长，读的超时保持原值 —— 控制环靠它快速判定掉线，不能变钝。
    this.writeTimeoutMs = writeTimeoutMs || timeoutMs * 4;
    this.port = null;
    this.rx = Buffer.alloc(0);
    this.queue = Promise.resolve();
    this.stats = { tx: 0, rxOk: 0, rxErr: 0, lastError: null, totalMs: 0, stale: 0 };
  }

  async open(SerialPortCtor) {
    this.port = new SerialPortCtor({
      path: this.portPath, baudRate: this.baud, dataBits: 8, stopBits: 2, parity: 'none',
      autoOpen: false,
    });
    this.port.on('data', (d) => { this.rx = Buffer.concat([this.rx, d]); });
    this.port.on('error', (e) => { this.stats.lastError = String(e.message || e); });
    await new Promise((res, rej) => this.port.open((err) => (err ? rej(err) : res())));
  }

  async close() {
    if (this.port && this.port.isOpen) await new Promise((res) => this.port.close(res));
  }

  /** 发一帧，等 expectedLen 字节回来（或超时）。ms 可覆盖默认超时：EEPROM 参数写要等更久 */
  _transact(req, expectedLen, ms = this.timeoutMs, isReply = null) {
    this.queue = this.queue.then(async () => {
      if (!this.port || !this.port.isOpen) throw new Error(`${this.portPath} 未打开`);
      this.rx = Buffer.alloc(0);
      this.stats.tx++;
      const t0 = Date.now();
      await new Promise((res, rej) => this.port.write(req, (e) => (e ? rej(e) : res())));
      await this._drain();
      await new Promise((res) => this.port.drain ? this.port.drain(res) : res());
      const got = await this._wait(expectedLen, ms, req, isReply);
      this.stats.totalMs += Date.now() - t0;
      return got;
    });
    return this.queue;
  }

  /**
   * 等"属于本次请求"的那一帧。
   * 为什么要认帧身份：上一次超时之后，它的回显可能**迟到**落进缓冲区。不认身份的话，
   * 下一次请求就会把上一轮的应答当成这一次的确认 —— 对写命令来说，这就是"以为写进去了、
   * 其实没写进去"。所以：不是这一帧的就丢掉继续等，并计入 stats.stale。
   * 异常帧只有 5 字节，这里也认（否则要一直等到超时才报得出异常码）。
   */
  async _wait(expectedLen, ms, req, isReply) {
    const t0 = Date.now();
    while (Date.now() - t0 < ms) {
      const exc = this.rx.length >= 2 && this.rx[1] >= 0x80;
      const need = exc ? 5 : expectedLen;
      if (this.rx.length >= need) {
        const cand = this.rx.subarray(0, need);
        this.rx = this.rx.subarray(need);
        if (exc || (isReply ? isReply(cand) : cand[0] === req[0])) return cand;
        this.stats.stale++;
        continue;
      }
      await new Promise((r) => setTimeout(r, 2));
    }
    const out = this.rx; this.rx = Buffer.alloc(0);
    return out.length ? out : null;
  }

  _drain() { return new Promise((r) => setTimeout(r, 0)); }
}

export class RtuBus {
  constructor(wheels, opts = {}) {
    this.wheels = wheels;
    this.opts = opts;
    this.channels = wheels.map((w) => new Channel(w.port, w.slave, opts));
  }

  async open() {
    // opts.SerialPortCtor 是给测试留的注入点：单元测试用假串口就能跑完整传输层，
    // 真机才需要 npm install serialport。
    let Ctor = this.opts.SerialPortCtor;
    if (!Ctor) {
      try {
        const mod = await import('serialport');
        Ctor = mod.SerialPort;
      } catch (e) {
        throw new Error('缺少 serialport 依赖：在 control 目录里执行 npm install（--dry-run / 单元测试不需要它）');
      }
    }
    const opened = [];
    for (let i = 0; i < this.channels.length; i++) {
      const ch = this.channels[i];
      try {
        await ch.open(Ctor);
        opened.push(ch);
      } catch (e) {
        // 中途失败必须把已经开着的口关回去：一个口只能被一个进程占用，
        // 留着就是"下一次启动 Access denied"，而且车上一半的轮子还挂在总线上。
        for (const o of opened) { try { await o.close(); } catch { /* 关不掉也不能把真正的错盖掉 */ } }
        const w = this.wheels[i];
        const who = w ? `${w.id} ${w.name} 站${w.slave}` : `第 ${i + 1} 个轮`;
        throw new Error(
          `打不开 ${ch.portPath}（${who}）：${e.message || e}\n`
          + `  先跑 node src/cli.js --list-ports 看这个口在不在系统里；不在就查 USB 线 / CH9114 供电 / 驱动`
          + `（Windows 装 CH343 系列，Linux 装 WCH 的 ch343ser 内核模块）。\n`
          + `  口在但编号变了，用环境变量覆盖，别改源码：CHASSIS_PORT_${w ? w.id : 'FL'}=<新口> 或整串偏移 CHASSIS_PORT_FIRST=<1 号口>。`
        );
      }
    }
  }

  async close() { for (const ch of this.channels) await ch.close(); }

  async readRegs(idx, addr, qty) {
    const { readRequest, parseReadResponse } = await import('./modbus.js');
    const ch = this.channels[idx];
    const req = readRequest(ch.slave, addr, qty);
    // 只按"是不是这个从站答的"筛：抢答/串线要能被报成 slave-mismatch，而不是被静悄悄丢掉。
    // 帧内容对不对，交给 parseReadResponse 判（字节数、CRC、异常码都在那边）。
    const isReply = (b) => b[0] === ch.slave;
    const buf = await ch._transact(req, 5 + qty * 2, ch.timeoutMs, isReply);
    const r = parseReadResponse(buf, ch.slave, addr, qty);
    if (r.ok) ch.stats.rxOk++; else { ch.stats.rxErr++; ch.stats.lastError = r.error; }
    return r;
  }

  /**
   * 写单个寄存器。
   * @param {boolean} eeprom 目标是不是 EEPROM 保持参数（如 Pn-30D）。是 → 用长超时等它落盘；
   *   否（控制环每拍写 Pn-303）→ 保持短超时。**这里绝不能变长**：一次写卡 1.2 秒就等于
   *   看门狗和 20 Hz 周期同时失效，那比"参数没写进去"危险得多。
   */
  async writeReg(idx, addr, value, { eeprom = false } = {}) {
    const { writeRequest, parseWriteResponse } = await import('./modbus.js');
    const ch = this.channels[idx];
    const req = writeRequest(ch.slave, addr, value);
    // 写的应答就是请求帧的回显，所以能逐字节比对 —— 这一点必须用足：
    // 上一笔超时后迟到的回显长得几乎一样，只有比对地址+值才能分辨"这次真的写进去了"。
    const isReply = (b) => b[0] === ch.slave && (b[1] >= 0x80 || b.subarray(0, 6).equals(req.subarray(0, 6)));
    const buf = await ch._transact(req, 8, eeprom ? ch.writeTimeoutMs : ch.timeoutMs, isReply);
    const r = parseWriteResponse(buf, req);
    if (r.ok) ch.stats.rxOk++; else { ch.stats.rxErr++; ch.stats.lastError = r.error; }
    return r;
  }

  /** 多路并行写：{idx, value}[] → 每路结果（互不阻塞） */
  async writeMany(addr, entries, opts = {}) {
    return Promise.all(entries.map(async (e) => {
      try { return { idx: e.idx, ...(await this.writeReg(e.idx, addr, e.value, opts)) }; }
      catch (err) { return { idx: e.idx, ok: false, error: String(err.message || err) }; }
    }));
  }

  async readMany(addr, qty, idxs = this.channels.map((_, i) => i)) {
    return Promise.all(idxs.map(async (i) => {
      try { return { idx: i, ...(await this.readRegs(i, addr, qty)) }; }
      catch (err) { return { idx: i, ok: false, error: String(err.message || err) }; }
    }));
  }

  latencyMs() {
    return this.channels.map((c) => ({ port: c.portPath, avg: c.stats.tx ? Math.round(c.stats.totalMs / (c.stats.rxOk + c.stats.rxErr || 1)) : 0 }));
  }
}
