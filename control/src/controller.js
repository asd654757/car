/**
 * 控制环 —— 输入(与设备无关) → 安全裁决 → 麦轮运动学 → 写入节流 → RS485。
 *
 * 输入层只负责调 setTwist()，不知道串口、不知道键盘也不知道手柄；
 * 手柄/键盘/网页都只是往这里灌 {vx, vy, wz} ∈ [-1,1]。
 */

import { CONFIG } from './config.js';
import { twistToRaw, describeTwist, fullScaleSpeedMps } from './kinematics.js';
import { makeThrottleState, planWrites, planStopAll, rollbackWrites } from './limits.js';
import { makeSafetyState, decide, noteInput, noteComm, latchEstop, clearLatch, STATE } from './safety.js';

export class ChassisController {
  constructor({ cfg = CONFIG, bus = null, dryRun = false, log = () => {} } = {}) {
    this.cfg = cfg;
    this.bus = bus;
    this.dryRun = dryRun || !bus;
    this.log = log;
    this.throttle = makeThrottleState(cfg.wheels.length);
    this.safety = makeSafetyState();
    this.lastTarget = [0, 0, 0, 0];
    this.lastSent = [0, 0, 0, 0];
    this.twist = { vx: 0, vy: 0, wz: 0 };
    this.forceZero = true; // 默认停：没跑过一拍之前不许有任何"我还在动"的印象
    this.deferStreak = 0;
    this.lastDeferLogAt = -Infinity;
    this.lastProbeAt = -Infinity; // 下一拍就发心跳
    this.monitor = new Array(cfg.wheels.length).fill(null); // 只读回来的 0x0700 组（驱动器自报）
    this.tickCount = 0;
    this.running = false;
    this.timer = null;
    this.events = [];
  }

  setTwist(t, nowMs = Date.now()) {
    this.twist = { vx: +t.vx || 0, vy: +t.vy || 0, wz: +t.wz || 0 };
    noteInput(this.safety, nowMs);
  }

  zero(nowMs = Date.now()) { this.setTwist({ vx: 0, vy: 0, wz: 0 }, nowMs); }
  estop(reason = '手动急停') { latchEstop(this.safety, reason); this._pushEvent(`急停：${reason}`); }
  release() { clearLatch(this.safety); }
  get locked() { return this.safety.state === STATE.ESTOP || this.safety.state === STATE.FAULT; }

  _pushEvent(msg) {
    this.events.unshift({ t: new Date().toISOString(), msg });
    if (this.events.length > 50) this.events.pop();
    this.log(msg);
  }

  /** 一拍：算目标 → 节流 → 发出去。dryRun 下只算不发。 */
  async step(nowMs = Date.now()) {
    this.tickCount++;
    const cfg = this.cfg;
    const d = decide(this.safety, nowMs, cfg.control);
    let target;
    let scaled = false;
    if (d.forceZero) {
      target = [0, 0, 0, 0];
    } else {
      const k = twistToRaw(this.twist, cfg);
      target = k.raw; scaled = k.scaled;
    }
    this.lastTarget = target;
    this.forceZero = d.forceZero; // 界面用：松手后操纵杆值还留着，但车已经停了
    if (!target.every((v) => Number.isFinite(v))) {
      this._pushEvent(`运动学输出非法：[${target}] → 本拍强制归零`);
      target = new Array(cfg.wheels.length).fill(0);
      this.lastTarget = target;
    }

    const { writes, deferred } = planWrites(this.throttle, target, nowMs, cfg.control);
    if (writes.length) this.deferStreak = 0;
    if (deferred.length) {
      this.deferStreak++;
      // 斜坡期间"这一拍没写"是设计内的常态（限频 250ms / 周期 50ms），所以只有连续 2 秒一个字节都发不出去
      // 才算异常（要么配置把 minWriteIntervalMs 提太高，要么某个口卡住了），并且 5 秒最多提示一次。
      if (this.deferStreak >= cfg.control.loopHz * 2 && nowMs - this.lastDeferLogAt > 5000) {
        this.lastDeferLogAt = nowMs;
        this._pushEvent(`连续 ${Math.round(this.deferStreak / cfg.control.loopHz)} 秒发不出新指令（累计推迟 ${this.throttle.counts.deferred} 次）：检查 minWriteIntervalMs 与串口是否卡住`);
      }
    } else if (!writes.length) this.deferStreak = 0;

    let results = writes.map((w) => ({ idx: w.idx, ok: true, dry: true }));
    if (!this.dryRun && writes.length) {
      results = await this.bus.writeMany(cfg.regs.speed, writes);
    }
    const failed = results.filter((r) => !r.ok);
    if (failed.length) {
      // 退回"线上值"，下一拍重发；节流器如果以为命令已经生效，就会永远闭嘴
      rollbackWrites(this.throttle, failed.map((f) => {
        const w = writes.find((x) => x.idx === f.idx);
        return { idx: f.idx, prev: w ? w.prev : this.throttle.written[f.idx] };
      }));
      this._pushEvent(`写入失败：${failed.map((f) => `${cfg.wheels[f.idx].id}(${f.error})`).join(' ')}`);
    }
    results.filter((r) => r.ok).forEach((r) => {
      const w = writes.find((x) => x.idx === r.idx);
      if (w) this.lastSent[r.idx] = w.value;
    });

    let probed = false;
    if (writes.length) {
      noteComm(this.safety, nowMs, failed.length === 0);
    } else if (!this.dryRun && nowMs - this.lastProbeAt >= cfg.control.commProbeMs) {
      // 只读心跳：稳态行驶一帧都不发，所以"发帧失败"这条路永远走不到，
      // 不主动读一次就永远不知道链路已经断了 —— 而这台驱动器断了线不会自己停。
      this.lastProbeAt = nowMs;
      const res = await this.bus.readMany(cfg.regs.monitor, cfg.regs.monitorQty);
      res.forEach((r) => { if (r.ok) this.monitor[r.idx] = r.regs; });
      noteComm(this.safety, nowMs, res.every((r) => r.ok));
      probed = true;
      if (!res.every((r) => r.ok)) this._pushEvent(`心跳读取失败：${res.filter((r) => !r.ok).map((f) => `${cfg.wheels[f.idx].id}(${f.error})`).join(' ')}`);
    }

    return { target, writes, results, safety: d, scaled, probed };
  }

  /** 强制四轮归零，绕过节流（退出/急停用） */
  async stopAll(reason = '停止') {
    const writes = planStopAll(this.throttle);
    if (!writes.length) return { writes: [] };
    if (this.dryRun) return { writes, dry: true };
    const results = await this.bus.writeMany(this.cfg.regs.speed, writes);
    const failed = results.filter((r) => !r.ok);
    if (failed.length) {
      rollbackWrites(this.throttle, failed.map((f) => {
        const w = writes.find((x) => x.idx === f.idx);
        return { idx: f.idx, prev: w ? w.prev : 0 };
      }));
      this._pushEvent(`⚠ ${reason}时有 ${failed.length} 路写 0 失败：${failed.map((f) => `${this.cfg.wheels[f.idx].id}(${f.error})`).join(' ')} —— 用物理总开关`);
    } else {
      this._pushEvent(`${reason}：已对 ${writes.length} 路写 0`);
    }
    return { writes, results, failed };
  }

  async start() {
    if (this.running) return;
    this.running = true;
    const period = Math.round(1000 / this.cfg.control.loopHz);
    this._pushEvent(`控制环启动 ${this.cfg.control.loopHz}Hz，满杆 ${fullScaleSpeedMps(this.cfg).toFixed(4)} m/s，${this.dryRun ? '干跑（不发帧）' : '在线'}`);
    const loop = async () => {
      if (!this.running) return;
      try { await this.step(); } catch (e) { this._pushEvent(`控制环异常：${e.message}`); }
      this.timer = setTimeout(loop, period);
    };
    this.timer = setTimeout(loop, 0);
  }

  async shutdown() {
    this.running = false;
    if (this.timer) clearTimeout(this.timer);
    try { await this.stopAll('退出清理'); } catch (e) { this._pushEvent(`退出清理失败：${e.message}`); }
  }

  snapshot() {
    const m = this.cfg.regs.mon;
    const pick = (i, a) => (this.monitor[i] ? this.monitor[i][a] ?? null : null);
    return {
      state: this.safety.state, reason: this.safety.reason,
      twist: this.twist, action: this.forceZero ? '停（无输入/闭锁）' : describeTwist(this.twist), forceZero: !!this.forceZero,
      target: this.lastTarget, onWire: this.throttle.written.slice(),
      wheels: this.cfg.wheels.map((w, i) => ({
        id: w.id, name: w.name, port: w.port, slave: w.slave, sign: w.sign,
        target: this.lastTarget[i], onWire: this.throttle.written[i],
        // 驱动器自报（电机侧 rpm）。"我们以为发了"和"它真的在转"是两件事，靠这个对撞。
        driveSpd: pick(i, m.spd), driveCmd: pick(i, m.cmd), moving: pick(i, m.pos),
      })),
      eeprom: { ...this.throttle.counts }, ticks: this.tickCount, dryRun: this.dryRun,
      events: this.events.slice(0, 12),
    };
  }
}
