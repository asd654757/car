import { test } from 'node:test';
import assert from 'node:assert/strict';
import { CONFIG } from '../src/config.js';
import { toI16 } from '../src/modbus.js';
import { preflight, formatPreflight, readNameplate, formatNameplate } from '../src/preflight.js';
import { rawForSpeedMps } from '../src/kinematics.js';

/** 造一台"只会应答读数"的假驱动器：spec 里给 null 表示这条寄存器读不到 */
function fakeDrive(spec = {}) {
  const regs = {
    [CONFIG.regs.mode]: 0x0130,
    [CONFIG.regs.ceiling]: 300,
    [CONFIG.regs.speed]: 0,
  };
  // 监控组 0x0700~0x0717 整块都要能应答（实机就是一次读 24 个），否则测不出跨组语义
  for (let a = CONFIG.regs.monitor; a < CONFIG.regs.monitor + CONFIG.regs.monitorQty; a++) regs[a] = 0;
  regs[CONFIG.regs.mon.torque] = 5;
  regs[0x070b] = 48;
  // 铭牌组 Fn-18~Fn-1E：块读 8 个必须整段有值，否则测不到"块读失败→逐寄存器兜底"
  for (let a = CONFIG.regs.nameplate; a < CONFIG.regs.nameplate + CONFIG.regs.nameplateQty; a++) regs[a] = 0;
  regs[CONFIG.regs.mon.ratedSpd] = 3000;
  regs[CONFIG.regs.mon.ratedVolt] = 48;
  regs[CONFIG.regs.mon.ratedTq] = 16;
  regs[CONFIG.regs.mon.ratedCur] = 5;
  regs[CONFIG.regs.mon.poles] = 10;
  const key = {
    mode: CONFIG.regs.mode, ceiling: CONFIG.regs.ceiling, preset: CONFIG.regs.speed, spd: CONFIG.regs.mon.spd,
    rated: CONFIG.regs.mon.ratedSpd,
  };
  for (const [k, v] of Object.entries(spec)) {
    if (!(k in key)) continue;
    if (v === null) delete regs[key[k]]; else regs[key[k]] = v;
  }
  const dead = spec.dead === true;
  return {
    async readRegs(_i, addr, qty) {
      if (dead) return { ok: false, error: 'no-response' };
      if (spec.nameplateBlockFails && addr === CONFIG.regs.nameplate && (qty || 1) > 1) return { ok: false, error: 'exception:4' };
      const n = qty || 1;
      const out = {};
      for (let a = addr; a < addr + n; a++) if (regs[a] !== undefined) out[a] = regs[a];
      if (Object.keys(out).length < n) return { ok: false, error: 'missing-register' };
      return { ok: true, regs: out };
    },
  };
}
const busOf = (drives) => ({ readRegs: (i, a, q) => drives[i].readRegs(i, a, q) });
const four = (specs) => busOf([0, 1, 2, 3].map((i) => fakeDrive(specs?.[i])));

test('四台都合格 → 通过，且体检报告把每台都列出来', async () => {
  const p = await preflight(four(), CONFIG);
  assert.equal(p.ok, true);
  assert.equal(p.rows.length, 4);
  const txt = formatPreflight(p);
  assert.match(txt, /通过/);
  ['FL', 'FR', 'BL', 'BR'].forEach((id) => assert.match(txt, new RegExp(id)));
  assert.match(txt, /0x0130/);
  assert.match(txt, /执行中命令=0/, 'SPd 不为 0 时要能对照"驱动器执行中的速度命令"，靠它区分测速抖动和真有人在开');
});

test('Pn-303 非零 → 拒绝启动，并且报告里看得见实际值（"上使能即自转"那道闸）', async () => {
  const p = await preflight(four([{ preset: 15000 }]), CONFIG);
  assert.equal(p.ok, false);
  assert.match(p.rows[0].problems.join(' '), /上使能即自转/);
  assert.equal(p.rows[0].info.preset, 15000, 'info.preset 必须回填，否则报告永远显示 0');
  assert.match(formatPreflight(p), /Pn-303=15000/);
  assert.equal(p.rows[1].ok, true, '只有一台有问题，其他台照常');
});

test('模式位不是"内部设定速度" → 拒绝（写了 Pn-303 也不会动，或动在别的命令源上）', async () => {
  const p = await preflight(four([{ mode: 0x0011 }]), CONFIG);
  assert.equal(p.ok, false);
  assert.match(p.rows[0].problems.join(' '), /第1位=1/);
});

test('Pn-30D 被人抬高 → 拒绝（脚本层的 60 rpm 硬顶就成纸了）', async () => {
  const p = await preflight(four([{ ceiling: 30000 }]), CONFIG);
  assert.equal(p.ok, false);
  assert.match(p.rows[0].problems.join(' '), /高于允许上限/);
});

test('Pn-30D 低于控制限幅 → 拒绝（驱动器会先把命令截掉，四轮就不等了）', async () => {
  const p = await preflight(four([{ ceiling: 100 }]), CONFIG);
  assert.equal(p.ok, false);
  assert.match(p.rows[0].problems.join(' '), /限幅失效/);
});

test('已经在转 → 拒绝，先确认现场安全', async () => {
  const p = await preflight(four([{ spd: 59 }]), CONFIG);
  assert.equal(p.ok, false);
  assert.match(p.rows[0].problems.join(' '), /转速 59 rpm 超出停转容差/);
});

test('静态 SPd=-1（Pn-303 已经是 0）→ 放行，但读数要留在报告里看得见', async () => {
  // 实机 2026-10-09：四台传令都为 0，FL/BR 稳定报 -1 rpm。1 rpm 电机侧 ≈ 0.00035 m/s，
  // 是测速抖动；卡"必须正好为 0"会让闸门永远过不去，反而逼人去做危险的事（--skip-preflight）。
  const p = await preflight(four([{ spd: -1 }, { spd: 0 }, { spd: 0 }, { spd: -1 }]), CONFIG);
  assert.equal(p.ok, true, '容差之内不该拦');
  assert.match(p.rows[0].problems.join(' '), /判为静态抖动/);
  assert.match(p.rows[0].problems.join(' '), /SPd=-1/);
  assert.equal(p.rows[1].problems.length, 0, '读数正好是 0 的台子不给任何提示');

  const tol = CONFIG.expect.spdIdleToleranceRpm;
  const over = await preflight(four([{ spd: tol + 1 }]), CONFIG);
  assert.equal(over.ok, false, `容差外（${tol + 1} rpm）必须仍然拦`);
});

test('读不到不等于合格：Pn-303 / Pn-000 拿不到就拒绝放行', async () => {
  const a = await preflight(four([{ preset: null }]), CONFIG);
  assert.equal(a.ok, false, '确认不了它是 0，就不能进控制环');
  assert.match(a.rows[0].problems.join(' '), /无法确认/);

  const b = await preflight(four([{ mode: null }]), CONFIG);
  assert.equal(b.ok, false);
  assert.match(b.rows[0].problems.join(' '), /读不到 Pn-000/);

  const c = await preflight(four([{ dead: true }]), CONFIG);
  assert.equal(c.ok, false, '整台没应答更要拒绝');
});

test('Pn-000 以有符号负数读回来时，nibble 仍然取对（0x8130 → 第1位=3）', async () => {
  const p = await preflight(four([{ mode: toI16(0x8130) }]), CONFIG);
  assert.equal(p.rows[0].info.mode1, 3, '用有符号值直接 >>4 会算出 2，误判成模式不对');
  assert.equal(p.ok, true);
});

test('Pn-30D 读数落在这台设备不可能给出的区间 → 按"读数不可信"拒绝，而不是当成限幅低/高', async () => {
  // 有符号读回来的值只会落在 -32768~32767；手册范围 0~30000 之外的读数（负数、30001~32767）
  // 一定不是我们要的那个参数，报"超出手册范围"比报"限幅失效"更接近真相。
  for (const bad of [-5536, 32000]) {
    const p = await preflight(four([{ ceiling: bad }]), CONFIG);
    assert.equal(p.ok, false, `读数 ${bad} 不许放行`);
    assert.match(p.rows[0].problems.join(' '), /超出手册范围/);
    assert.equal(p.rows[1].ok, true, '只有那一台的读数有问题');
  }
});

/* ---------------- 铭牌只读探测：提转速前的功课 ---------------- */

test('铭牌读回来的额定转速是"能不能提速"的唯一依据，不是我的假设', async () => {
  const p = await readNameplate(four(), CONFIG);
  assert.equal(p.ok, true);
  assert.equal(p.rows[0].info.ratedSpd, 3000, 'Fn-1C');
  assert.equal(p.rows.length, 4);
  // 3000 rpm(电机侧) ÷ 30 = 100 rpm(轮子) × 周长 0.6283 / 60 ≈ 1.047 m/s
  assert.ok(Math.abs(p.rows[0].info.ratedMps - 1.047) < 0.005, `额定折到车速应≈1.05，实际 ${p.rows[0].info.ratedMps}`);
  const txt = formatNameplate(p);
  assert.match(txt, /Fn-1C 额定转速=3000/);
  assert.match(txt, /当前 Pn-30D=300/);
});

test('给目标车速 → 换算出需要的 raw，并拿当前限幅比出来（1.0 m/s 需要改 Pn-30D）', async () => {
  const p = await readNameplate(four(), CONFIG, 1.0);
  const want = Math.round(rawForSpeedMps(1.0, CONFIG));
  assert.equal(p.need.raw, want, '报告里的需求值必须和 kinematics 的换算同源');
  assert.ok(want > 10000 && want < 20000, `1.0 m/s @30:1/Ø0.20 应当在 1.4 万 raw 量级，实际 ${want}`);
  assert.equal(p.rows[0].info.overCeiling, true, '当前 300 的限幅远低于需求 → 明着说出来');
  assert.equal(p.rows[0].info.overRated, false, '2865 rpm < 3000 rpm 额定，没超');
  assert.equal(p.rows[0].ok, true, '限幅不够只是"还差一次参数写入"，不该和安全问题混成同一种红');
  assert.match(p.rows[0].problems.join(' '), /低于需求/);
  assert.match(formatNameplate(p), /需要改 Pn-30D/);
  assert.match(formatNameplate(p), /写参数要单独确认/);
});

test('当前限幅读数不可信（负数 / 超过 30000）→ 不许拿它算"还差多少"，也不许放行写参数', async () => {
  const p = await readNameplate(four([{ ceiling: -5536 }]), CONFIG, 1.0);
  assert.equal(p.rows[0].info.ceilingPlausible, false);
  assert.equal(p.rows[0].info.overCeiling, false, '来路不明的数不能参与"需求 > 现值"的比较，否则结论也是来路不明');
  assert.match(p.rows[0].problems.join(' '), /读数不可信/);
});

test('目标超额定 → 直接拒绝，并说明是物理上限不是软件限幅', async () => {
  const p = await readNameplate(four(), CONFIG, 1.3); // 3724 rpm > 3000 rpm
  assert.equal(p.rows[0].info.overRated, true);
  assert.equal(p.rows[0].ok, false, '超额定才是 FAIL，限幅不够不是');
  assert.equal(p.ok, false);
  assert.match(p.rows[0].problems.join(' '), /超额定/);
  assert.match(formatNameplate(p), /超额定/);
});

test('读不到额定转速 = 不许提速；块读失败时逐寄存器兜底', async () => {
  const a = await readNameplate(four([{ rated: null }]), CONFIG);
  assert.equal(a.ok, false);
  assert.match(a.rows[0].problems.join(' '), /读不到 Fn-1C/);

  const b = await readNameplate(four([{ nameplateBlockFails: true }]), CONFIG);
  assert.equal(b.rows[0].info.fallback, true, '整块读失败要退化成单个读，第一次上机不能卡在块读语义上');
  assert.equal(b.rows[0].info.ratedSpd, 3000);
  assert.equal(b.rows[1].info.fallback, undefined, '只有那台块读失败');
});

test('减速比 ±10% 的不确定区间要印在报告里（1.0 m/s 可能是 0.95~1.11）', async () => {
  const p = await readNameplate(four(), CONFIG, 1.0);
  const lo = Math.min(p.need.mpsIfGearLow, p.need.mpsIfGearHigh);
  const hi = Math.max(p.need.mpsIfGearLow, p.need.mpsIfGearHigh);
  assert.ok(lo < 0.96 && hi > 1.10, `区间应覆盖 ±10%，实际 ${lo}~${hi}`);
  assert.match(formatNameplate(p), /不确定区间/);
});
