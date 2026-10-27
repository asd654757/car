import { test } from 'node:test';
import assert from 'node:assert/strict';
import { CONFIG } from '../src/config.js';
import { speedPreset } from '../src/preset.js';
import { ChassisController } from '../src/controller.js';
import { STATE } from '../src/safety.js';

const N = CONFIG.wheels.length;
const PERIOD = 1000 / CONFIG.control.loopHz; // 50ms

/** 每份测试自己的配置，避免跨用例污染 */
function cfg() {
  return { ...CONFIG, control: { ...CONFIG.control }, mech: { ...CONFIG.mech }, wheels: CONFIG.wheels.map((w) => ({ ...w })), regs: { ...CONFIG.regs } };
}

/** 假总线：记帧 + 可注入失败。只实现控制核用到的两个方法。 */
function fakeBus({ failWrite = null, failRead = false } = {}) {
  const sent = [];
  const reads = [];
  return {
    sent, reads,
    async writeMany(addr, entries) {
      return entries.map((e) => {
        const bad = failWrite ? failWrite(e) : false;
        if (!bad) sent.push({ addr, idx: e.idx, value: e.value });
        return bad ? { idx: e.idx, ok: false, error: 'fake-timeout' } : { idx: e.idx, ok: true };
      });
    },
    async readMany(addr, qty) {
      reads.push(addr);
      if (failRead) return CONFIG.wheels.map((_, i) => ({ idx: i, ok: false, error: 'fake-no-response' }));
      // 回一帧"驱动器自报"：SPd = 线上值 × 0.2
      return CONFIG.wheels.map((_, i) => ({ idx: i, ok: true, regs: { [CONFIG.regs.mon.spd]: 0, [CONFIG.regs.mon.cmd]: 0, [CONFIG.regs.mon.pos]: 0 } }));
    },
  };
}

/** 喂 n 拍，输入保持新鲜 */
async function drive(ctl, bus, n, twist = { vx: 1, vy: 0, wz: 0 }, t0 = 1_000_000) {
  for (let i = 0; i < n; i++) {
    const t = t0 + i * PERIOD;
    if (twist) ctl.setTwist(twist, t);
    const r = await ctl.step(t);
    if (r.safety.state === STATE.FAULT || r.safety.state === STATE.ESTOP) return i;
  }
  return n;
}

test('前进满杆 → 下发的就是四轮自检那组 [+300,+300,-300,-300]，且只写 Pn-303', async () => {
  const bus = fakeBus();
  const ctl = new ChassisController({ cfg: cfg(), bus });
  await drive(ctl, bus, 60); // 3 秒，够爬完斜坡

  assert.deepEqual(ctl.lastTarget, [300, 300, -300, -300], '前轮正、后轮负 = 2026-10-09 14:58 实物确认的符号矩阵');
  assert.deepEqual(ctl.throttle.written, [300, 300, -300, -300], '60 rpm 电机侧 = 顶到 Pn-30D');
  assert.deepEqual([...new Set(bus.sent.map((s) => s.addr))], [CONFIG.regs.speed], '运行期只碰速度寄存器');
  assert.equal(ctl.safety.state, STATE.RUN);
});

test('满杆 + maxCmdRaw=75（1/4 档实机跑法）→ 线上值不越过限幅，绝不因量化写成 80', async () => {
  const bus = fakeBus();
  const c = cfg();
  c.control.maxCmdRaw = 75;
  const ctl = new ChassisController({ cfg: c, bus });
  await drive(ctl, bus, 40); // 够爬完 0→40→70
  assert.deepEqual(ctl.throttle.written, [70, 70, -70, -70], '退到限幅以下最大整档（14 rpm 电机侧）');
  assert.ok(bus.sent.every((f) => Math.abs(f.value) <= 75), `有帧越过限幅：${JSON.stringify(bus.sent)}`);
  const before = ctl.throttle.counts.writes;
  await drive(ctl, bus, 20);
  assert.equal(ctl.throttle.counts.writes, before, '到位以后稳态零写入');
});

test('斜坡过程中不换向、不回退，幅值单调上升', async () => {
  const bus = fakeBus();
  const ctl = new ChassisController({ cfg: cfg(), bus });
  await drive(ctl, bus, 60);
  const seq = [[], [], [], []];
  bus.sent.forEach((s) => seq[s.idx].push(s.value));
  seq.forEach((values, i) => {
    const sign = CONFIG.wheels[i].sign;
    assert.ok(values.every((v) => v === 0 || Math.sign(v) === sign), `轮 ${CONFIG.wheels[i].id} 中途换向：${values}`);
    assert.ok(values.every((v) => Math.abs(v) <= CONFIG.control.maxCmdRaw), '任何一拍都不越硬顶');
    const amps = values.map(Math.abs);
    assert.ok(amps.every((a, k) => k === 0 || a >= amps[k - 1]), `轮 ${CONFIG.wheels[i].id} 幅值回退：${values}`);
    assert.ok(values.length <= 8, `爬坡最多 ${CONFIG.control.maxCmdRaw / CONFIG.control.rampRawPerTick} 步，实际 ${values.length}`);
  });
});

test('稳态 60 秒：写入次数停在常数上，EEPROM 不会被刷爆', async () => {
  const bus = fakeBus();
  const ctl = new ChassisController({ cfg: cfg(), bus });
  await drive(ctl, bus, 1200); // 60 s @20Hz
  const maxWrites = N * Math.ceil(CONFIG.control.maxCmdRaw / CONFIG.control.rampRawPerTick);
  assert.equal(ctl.throttle.counts.writes, maxWrites, `只该是爬坡那 ${maxWrites} 次`);
  assert.equal(bus.sent.length, maxWrites, '一帧不多发');
  assert.ok(ctl.tickCount >= 1200);
  assert.ok(bus.reads.length > 0, '稳态期间必须有只读心跳，否则链路断了没人知道');
});

test('松手（手柄丢信号/网页关掉）→ 超过 inputTimeoutMs 四轮自动写 0', async () => {
  const bus = fakeBus();
  const ctl = new ChassisController({ cfg: cfg(), bus });
  await drive(ctl, bus, 60);
  assert.deepEqual(ctl.throttle.written, [300, 300, -300, -300]);
  const tEnd = 1_000_000 + 59 * PERIOD; // 最后一次喂输入的时刻
  bus.sent.length = 0;

  let tick = 0;
  for (let i = 1; i <= 30; i++) {
    await ctl.step(tEnd + i * PERIOD); // 故意不再喂任何输入
    if (bus.sent.some((s) => s.value === 0)) { tick = i; break; }
  }
  assert.ok(tick, '看门狗必须自己触发，不需要谁去点停止');
  assert.ok((tick - 1) * PERIOD <= CONFIG.control.inputTimeoutMs && tick * PERIOD > CONFIG.control.inputTimeoutMs,
    `第 ${tick} 拍（${tick * PERIOD}ms）归零，阈值 ${CONFIG.control.inputTimeoutMs}ms`);
  assert.equal(bus.sent.filter((s) => s.value === 0).length, N, '四轮都写了 0');
  assert.deepEqual(ctl.throttle.written, [0, 0, 0, 0]);
  assert.equal(ctl.safety.state, STATE.IDLE);
});

test('急停闭锁：喂再多的"前进"都不许动，只有 release 能恢复', async () => {
  const bus = fakeBus();
  const ctl = new ChassisController({ cfg: cfg(), bus });
  await drive(ctl, bus, 60);
  bus.sent.length = 0;
  ctl.estop('用户按急停');
  await drive(ctl, bus, 10);
  assert.deepEqual(ctl.throttle.written, [0, 0, 0, 0], '急停这一拍就把线上值清零');
  assert.deepEqual(bus.sent.map((s) => s.value).filter((v) => v !== 0), [], '闭锁后没有第二帧非零指令');
  bus.sent.length = 0;
  await drive(ctl, bus, 10);
  assert.deepEqual(bus.sent, [], '已经全零 → 一字节都不发');
  assert.equal(ctl.locked, true);

  ctl.release();
  await drive(ctl, bus, 60);
  assert.deepEqual(ctl.throttle.written, [300, 300, -300, -300], '解除后可以重新走');
});

test('写失败会重发：不把"以为发了"当成"发到了"', async () => {
  let drop = true;
  const bus = fakeBus({ failWrite: (e) => { if (e.idx === 2 && drop) { drop = false; return true; } return false; } });
  const ctl = new ChassisController({ cfg: cfg(), bus });
  const t0 = 1_000_000;
  ctl.setTwist({ vx: 1, vy: 0, wz: 0 }, t0);
  await ctl.step(t0); // 第一拍：四轮都发，3 号这一路丢帧
  assert.equal(ctl.throttle.written[2], 0, '失败的路把"线上值"退回原值，否则节流器以为已生效就永远闭嘴');
  assert.notEqual(ctl.throttle.written[0], 0, '其余三路照常，互不牵连');

  await drive(ctl, bus, 60, { vx: 1, vy: 0, wz: 0 }, t0 + PERIOD);
  assert.deepEqual(ctl.throttle.written, [300, 300, -300, -300], '重发之后追上其他三轮');
  assert.equal(ctl.safety.state, STATE.RUN, '一次丢帧不该误判成故障');
});

test('连续 3 次通讯失败 → 故障闭锁并清零，且不会自己解锁', async () => {
  const bus = fakeBus({ failWrite: () => true, failRead: true });
  const ctl = new ChassisController({ cfg: cfg(), bus });
  await drive(ctl, bus, 200);
  assert.equal(ctl.safety.state, STATE.FAULT, '链路断了必须闭锁 —— 这台驱动器没有断线停车');
  assert.equal(ctl.locked, true);
  await drive(ctl, bus, 20);
  assert.equal(ctl.safety.state, STATE.FAULT, '闭锁不自动解除');
});

test('纯前进 = 四轮物理滚向一致；纯平移 = 对角同号（标准麦轮 X 型）', async () => {
  const roll = (w) => w.map((v, i) => v * CONFIG.wheels[i].sign); // 命令值 → "向前滚为正"

  const b1 = fakeBus(); const c1 = new ChassisController({ cfg: cfg(), bus: b1 });
  await drive(c1, b1, 60, { vx: 1, vy: 0, wz: 0 });
  assert.deepEqual(roll(c1.throttle.written), [300, 300, 300, 300], '四轮都向前滚 —— 这就是 14:58 实物确认的那件事');

  const b2 = fakeBus(); const c2 = new ChassisController({ cfg: cfg(), bus: b2 });
  await drive(c2, b2, 60, { vx: 0, vy: 1, wz: 0 });
  const r = roll(c2.throttle.written);
  assert.equal(Math.sign(r[0]), Math.sign(r[3]), '左前 / 右后 是一条对角');
  assert.equal(Math.sign(r[1]), Math.sign(r[2]), '右前 / 左后 是另一条对角');
  assert.equal(Math.sign(r[0]), -Math.sign(r[1]), '两条对角反号');

  const b3 = fakeBus(); const c3 = new ChassisController({ cfg: cfg(), bus: b3 });
  await drive(c3, b3, 60, { vx: 0, vy: 0, wz: 1 });
  const r3 = roll(c3.throttle.written);
  assert.equal(Math.sign(r3[0]), -Math.sign(r3[1]), '原地转：左右两侧反号');
  assert.equal(Math.sign(r3[0]), Math.sign(r3[2]), '同侧前后同向，左右两侧反向 —— 这才是绕中心转');
});

test('stopAll 只清零非零的那几路，全零时一帧不发', async () => {
  const bus = fakeBus();
  const ctl = new ChassisController({ cfg: cfg(), bus });
  await drive(ctl, bus, 60, { vx: 0.3, vy: 0, wz: 0 });
  bus.sent.length = 0;
  const r = await ctl.stopAll('测试');
  assert.equal(r.writes.length, N);
  assert.deepEqual(ctl.throttle.written, [0, 0, 0, 0]);
  const r2 = await ctl.stopAll('测试');
  assert.deepEqual(r2.writes, [], '已经全零 → 不重复写');
});

test('干跑（不接串口）也能出完整指令表，供落地前核对', async () => {
  const ctl = new ChassisController({ cfg: cfg(), dryRun: true });
  await drive(ctl, null, 60);
  assert.deepEqual(ctl.throttle.written, [300, 300, -300, -300]);
  const snap = ctl.snapshot();
  assert.equal(snap.dryRun, true);
  assert.equal(snap.wheels.length, N);
  assert.equal(snap.wheels[0].target, 300);
  assert.deepEqual(snap.wheels[2].target, -300);
});

test('限频推迟不刷屏：猛推满杆和慢捻推杆都一条不报，真发不出去才报', async () => {
  const a = new ChassisController({ cfg: cfg(), bus: fakeBus() });
  await drive(a, null, 40, { vx: 1, vy: 0, wz: 0 });
  assert.equal(a.events.filter((e) => /推迟/.test(e.msg)).length, 0, '正常爬坡不该刷屏');
  assert.ok(a.throttle.counts.deferred > 20, `但推迟次数照常进统计：${a.throttle.counts.deferred}`);

  const c = cfg(); c.control.minWriteIntervalMs = 60_000; // 故意把限频调到一分钟
  const b = new ChassisController({ cfg: c, bus: fakeBus() });
  const t0 = 5_000_000;
  for (let i = 0; i < 80; i++) { const t = t0 + i * PERIOD; b.setTwist({ vx: i / 80, vy: 0, wz: 0 }, t); await b.step(t); }
  const stuck = b.events.filter((e) => /发不出新指令/.test(e.msg));
  assert.equal(stuck.length, 1, `连续 2 秒一个字节都发不出去 → 报一条，实际 ${stuck.length} 条`);
  assert.match(stuck[0].msg, /minWriteIntervalMs/);
});

test('松手后界面不能还写着"前进"：action 要跟着 forceZero 变', async () => {
  const bus = fakeBus();
  const ctl = new ChassisController({ cfg: cfg(), bus });
  await drive(ctl, bus, 20, { vx: 1, vy: 0, wz: 0 });
  assert.equal(ctl.snapshot().action, '前进');
  const tEnd = 1_000_000 + 19 * PERIOD;
  await ctl.step(tEnd + 10 * PERIOD); // 连着 10 拍没喂输入
  const s = ctl.snapshot();
  assert.equal(s.forceZero, true);
  assert.match(s.action, /停/);
  assert.equal(s.twist.vx, 1, '操纵杆值本身还留着（再来输入就接着走）');
});

test('快照里的驱动器自报读数来自心跳，不是我们的假设', async () => {
  const bus = fakeBus();
  bus.readMany = async (addr, qty) => CONFIG.wheels.map((_, i) => ({ idx: i, ok: true, regs: { [CONFIG.regs.mon.spd]: 59 - i, [CONFIG.regs.mon.cmd]: 0, [CONFIG.regs.mon.pos]: 123 } }));
  const ctl = new ChassisController({ cfg: cfg(), bus });
  await drive(ctl, bus, 60);
  const snap = ctl.snapshot();
  assert.equal(snap.wheels[0].driveSpd, 59, 'SPd 读回来了（60 rpm 电机侧 ≈ 面板 59.x）');
  assert.equal(snap.wheels[3].driveSpd, 56);
  assert.equal(snap.wheels[0].moving, 123);
});

test('1.0 m/s 档在控制核里也是"爬上去"而不是一跳到位，高速稳态同样零写入', async () => {
  const fast = structuredClone(speedPreset(CONFIG, 1.0));
  const bus = fakeBus();
  const ctl = new ChassisController({ cfg: fast, bus });
  const t0 = 1_000_000;
  let prev = 0;
  let reachedAt = null;
  for (let i = 0; i < 120; i++) {
    const t = t0 + i * PERIOD;
    ctl.setTwist({ vx: 1, vy: 0, wz: 0 }, t);
    await ctl.step(t);
    const w = ctl.throttle.written[0];
    assert.ok(w >= prev, `斜坡不许回退：第 ${i} 拍从 ${prev} 变成 ${w}`);
    prev = w;
    if (w === 14210 && reachedAt === null) reachedAt = i;
  }
  assert.equal(prev, 14210, `满杆落在量化后的 14210（14324 越界向下退一档），实际 ${prev}`);
  assert.ok(reachedAt !== null && reachedAt <= 30, `第 ${reachedAt} 拍到位 ≈ ${reachedAt * 50} ms，设计目标 1.25 s`);
  assert.deepEqual(ctl.throttle.written, [14210, 14210, -14210, -14210], '符号矩阵在高档位不变');
  assert.deepEqual([...new Set(bus.sent.map((s) => s.addr))], [CONFIG.regs.speed], '高速档运行期也只碰速度寄存器');
  assert.ok(Math.abs(ctl.lastTarget[0]) <= fast.control.maxCmdRaw, `目标不许越过软顶 ${fast.control.maxCmdRaw}`);

  const before = ctl.throttle.counts.writes;
  for (let i = 0; i < 60; i++) {
    const t = t0 + (120 + i) * PERIOD;
    ctl.setTwist({ vx: 1, vy: 0, wz: 0 }, t);
    await ctl.step(t);
  }
  assert.equal(ctl.throttle.counts.writes, before, '高速稳态也必须一字节都不发（EEPROM 约束不分档位）');
  assert.equal(ctl.safety.state, STATE.RUN);
});
