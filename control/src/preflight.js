/**
 * 预检（只读）—— 不满足条件就拒绝进入控制环。
 *
 * 每一项都对应一个真踩过的坑：
 *  - 模式不是"内部设定速度" → 写 Pn-303 不会有反应，或者更糟：反应在别的命令源上。
 *  - Pn-30D 上限被抬高 → 脚本层的 maxCmdRaw 保护就形同虚设。
 *  - Pn-303 不为 0 → **一上使能电机就自己转**（2026-10-01 那个 -59 rpm 的"幽灵命令"就是这个）。
 *  - SPd 不为 0 → 有人在手动使能，或者上一次没停干净。
 */

import { CONFIG } from './config.js';
import { asU16 } from './modbus.js';
import { rawForSpeedMps, speedMpsFromRaw } from './kinematics.js';

export async function preflight(bus, cfg = CONFIG) {
  const MON = cfg.regs.mon; // 与运行期用同一张地址表，别在这里另抄一份
  const rows = [];
  for (let i = 0; i < cfg.wheels.length; i++) {
    const w = cfg.wheels[i];
    const row = { wheel: w.id, name: w.name, port: w.port, slave: w.slave, ok: true, problems: [], info: {} };
    const rd = async (addr) => {
      const r = await bus.readRegs(i, addr, 1);
      return r.ok ? r.regs[addr] : null;
    };
    const pn000 = await rd(cfg.regs.mode);
    const ceiling = await rd(cfg.regs.ceiling);
    const preset = await rd(cfg.regs.speed);
    const mon = await bus.readRegs(i, cfg.regs.monitor, cfg.regs.monitorQty);

    if (pn000 === null) { row.ok = false; row.problems.push('读不到 Pn-000（口/站号/接线？）'); }
    else {
      const u = asU16(pn000); // 位域参数：读回 -4096 这种负值就是高位 1，直接 >> 会把符号位搬进来
      const mode1 = (u >> 4) & 0xf;
      row.info.pn000 = `0x${u.toString(16).padStart(4, '0')}`;
      row.info.mode1 = mode1;
      if (mode1 !== cfg.expect.modeNibble1) { row.ok = false; row.problems.push(`Pn-000 第1位=${mode1}，不是内部设定速度模式(3)`); }
    }
    if (ceiling === null) { row.ok = false; row.problems.push('读不到 Pn-30D 限速'); }
    else if (ceiling < 0 || ceiling > (cfg.pn30dManualMaxRaw ?? 30000)) {
      // 手册给的范围是 0~30000，落在这个区间之外说明这帧根本不是 Pn-30D 的值（地址/组别/串扰）
      // —— 宁可当成"不知道现状"，也不能拿它去做限幅判断。
      row.info.ceiling = ceiling;
      row.ok = false;
      row.problems.push(`Pn-30D 读数 ${ceiling} 超出手册范围 0~${cfg.pn30dManualMaxRaw ?? 30000} → 这读数不可信，按"不知道现状"处理`);
    }
    else {
      row.info.ceiling = ceiling;
      if (ceiling > cfg.expect.maxCeilingRaw) { row.ok = false; row.problems.push(`Pn-30D=${ceiling} 高于允许上限 ${cfg.expect.maxCeilingRaw}`); }
      if (ceiling < cfg.control.maxCmdRaw) { row.ok = false; row.problems.push(`Pn-30D=${ceiling} 低于控制限幅 ${cfg.control.maxCmdRaw}，限幅失效`); }
    }
    row.info.preset = preset ?? 0; // 显示用的实际值（曾经漏赋值，导致体检永远显示 0）
    if (preset === null) { row.ok = false; row.problems.push('读不到 Pn-303，无法确认它是不是 0 → 不放行'); }
    else if (preset !== 0) { row.ok = false; row.problems.push(`Pn-303=${preset} 非零 → 上使能即自转，先归零`); }

    if (mon.ok) {
      row.info.spd = mon.regs[MON.spd];
      row.info.cmd = mon.regs[MON.cmd];
      row.info.vdC = mon.regs[0x070b];
      row.info.tq = mon.regs[MON.torque];
      const idleTol = Number.isFinite(cfg.expect?.spdIdleToleranceRpm) ? cfg.expect.spdIdleToleranceRpm : 0;
      const spdNow = mon.regs[MON.spd];
      // 与写参数闸门同一把尺子：Pn-303=0 时驱动器静态报 ±1 rpm 是测速抖动（1 rpm 电机侧 ≈ 0.00035 m/s），
      // 卡"必须正好为 0"会让正常启动永远过不去；容差之外的读数一律当"它在动"。
      if (spdNow !== 0 && Math.abs(spdNow) > idleTol) { row.ok = false; row.problems.push(`当前转速 ${spdNow} rpm 超出停转容差 ${idleTol} rpm，先确认安全`); }
      else if (spdNow !== 0) row.problems.push(`SPd=${spdNow} rpm 在停转容差 ${idleTol} rpm 内（Pn-303=0）→ 判为静态抖动，放行但记在这里`);
    } else {
      row.problems.push('读不到监控组（不影响启动，但遥测缺失）');
    }
    rows.push(row);
  }
  return { ok: rows.every((r) => r.ok), rows };
}

export function formatPreflight(p) {
  const lines = p.rows.map((r) =>
    `  ${r.ok ? 'OK  ' : 'FAIL'} ${r.wheel} ${r.name} ${r.port}/站${r.slave}  ` +
    `Pn-000=${r.info.pn000 ?? '?'} 模式位=${r.info.mode1 ?? '?'} 上限=${r.info.ceiling ?? '?'} ` +
    `Pn-303=${r.info.preset ?? '?'} SPd=${r.info.spd ?? '?'} 执行中命令=${r.info.cmd ?? '?'} vdC=${r.info.vdC ?? '?'}V` +
    (r.problems.length ? `\n       └ ${r.problems.join('；')}` : ''));
  return [`预检（只读）：${p.ok ? '通过' : '不通过'}`, ...lines].join('\n');
}

/**
 * 铭牌只读探测 —— 提转速之前必须先跑，一个字节都不写。
 *
 * 和 preflight() 的分工：那个判"现在能不能安全启动"，这个判"这台电机物理上允许跑多快"。
 * Pn-30D 只是限幅（手册 0~30000 raw = 0~6000 rpm），Fn-1C 额定转速才是真上限，
 * 手册原话"在没有负载的情况下，转速会一直升高到额定转速"——空载打到额定之上就是超额定跑。
 *
 * @param {number|null} targetMps 想要的车速；给了就把 raw 需求、额定、当前限幅三者摆一起对比
 */
export async function readNameplate(bus, cfg = CONFIG, targetMps = null) {
  const MON = cfg.regs.mon;
  const one = async (i, addr) => {
    try { const r = await bus.readRegs(i, addr, 1); return r.ok ? r.regs[addr] : null; }
    catch { return null; }
  };

  // 目标车速要多少 raw：只看机械参数，与四轮无关，算一次
  const need = targetMps && targetMps > 0 ? (() => {
    const raw = Math.round(rawForSpeedMps(targetMps, cfg));
    const mpsAt = (f) => speedMpsFromRaw(raw, { ...cfg, mech: { ...cfg.mech, gearRatio: cfg.mech.gearRatio * f } });
    return { mps: targetMps, raw, motorRpm: raw / cfg.rawPerRpm, wheelRpm: raw / cfg.rawPerRpm / cfg.mech.gearRatio,
      mpsIfGearLow: mpsAt(0.9), mpsIfGearHigh: mpsAt(1.1) }; // 减速比 ±10% 对应的真实车速区间
  })() : null;

  const rows = [];
  for (let i = 0; i < cfg.wheels.length; i++) {
    const w = cfg.wheels[i];
    const row = { wheel: w.id, name: w.name, port: w.port, slave: w.slave, ok: true, problems: [], info: {} };

    // 块读覆盖 Fn-19~Fn-20；Fn-21 反电势常数在块外，单独读一次
    let regs = {};
    try {
      const blk = await bus.readRegs(i, cfg.regs.nameplate, cfg.regs.nameplateQty);
      if (blk.ok) regs = blk.regs;
    } catch { /* 整块读不到就走下面的逐寄存器兜底 */ }
    const pick = (addr) => (regs[addr] !== undefined ? regs[addr] : null);
    let rated = pick(MON.ratedSpd);
    if (rated === null) { // 整块读失败时逐个读，第一次上机不能让探测卡在块读语义上
      row.info.fallback = true;
      for (const key of ['ratedPow', 'ratedVolt', 'ratedTq', 'ratedSpd', 'ratedCur', 'poles', 'rotorInertia', 'tqConst']) {
        regs[MON[key]] = await one(i, MON[key]);
      }
      rated = pick(MON.ratedSpd);
    }
    row.info.backEmf = await one(i, MON.backEmf);
    row.info.ratedSpd = rated;
    row.info.ratedVolt = pick(MON.ratedVolt);
    row.info.ratedTq = pick(MON.ratedTq);
    row.info.ratedCur = pick(MON.ratedCur);
    row.info.poles = pick(MON.poles);

    // 读到 ≠ 读对：0x07xx 越界时驱动器照样回过 10851 这种数，所以先过合理区间再拿去做决策
    const gate = cfg.expect.ratedSpdPlausible || { min: 50, max: 6000 };
    const plausible = rated !== null && rated >= gate.min && rated <= gate.max;
    row.info.ratedPlausible = plausible;
    if (rated === null) { row.ok = false; row.problems.push('读不到 Fn-1C 额定转速 → 不许提转速'); }
    else if (!plausible) { row.ok = false; row.problems.push(`Fn-1C=${rated} rpm 不在合理区间 ${gate.min}~${gate.max} → 地址或组别映射存疑，不许据此提转速`); }

    row.info.ceiling = await one(i, cfg.regs.ceiling);
    if (row.info.ceiling === null) row.problems.push('读不到 Pn-30D 当前限幅');
    // 和额定转速同一个道理：读到 ≠ 读对。超出手册 0~30000 的数（或负数）只能是这帧不是它，
    // 标成不可信，让写参数的闸门据此拦住 —— 不能拿一个来路不明的"当前限幅"去做差值判断。
    else if (row.info.ceiling < 0 || row.info.ceiling > (cfg.pn30dManualMaxRaw ?? 30000)) {
      row.info.ceilingPlausible = false;
      row.problems.push(`当前 Pn-30D=${row.info.ceiling} 超出手册范围 0~${cfg.pn30dManualMaxRaw ?? 30000} → 读数不可信，不许据此写参数`);
    }
    row.info.preset = await one(i, cfg.regs.speed);

    if (plausible) {
      row.info.ratedMps = speedMpsFromRaw(rated * cfg.rawPerRpm, cfg); // 额定折到车速（还受减速比精度影响）
      if (row.info.ceiling !== null) row.info.ceilingMps = speedMpsFromRaw(row.info.ceiling, cfg);
      if (need) {
        row.info.needRaw = need.raw;
        row.info.overRated = need.motorRpm > rated * 1.02; // 留 2% 余量口径
        row.info.overCeiling = row.info.ceiling !== null && row.info.ceilingPlausible !== false && need.raw > row.info.ceiling;
        // 超额定 = 物理上不该做，判 FAIL；限幅不够 = 只是"还差一次参数写入"，不判 FAIL，
        // 否则会把它和安全问题混成同一种红。
        if (row.info.overRated) { row.ok = false; row.problems.push(`目标 ${need.motorRpm.toFixed(0)} rpm(电机侧) 超额定 ${rated} rpm`); }
        if (row.info.overCeiling) row.problems.push(`当前 Pn-30D=${row.info.ceiling} 低于需求 ${need.raw} → 不改限幅就跑不到（写参数要单独确认）`);
      }
    }
    rows.push(row);
  }
  return { ok: rows.every((r) => r.ok), rows, need, cfg };
}

export function formatNameplate(p) {
  const out = ['铭牌（只读）：'];
  for (const r of p.rows) {
    const sub = [];
    if (r.info.ratedMps !== undefined) {
      sub.push(`额定折到车速 ≈ ${r.info.ratedMps.toFixed(2)} m/s（减速比按 ${p.cfg.mech.gearRatio}:1、直径 ${p.cfg.mech.wheelDiameterM} m，两者都还是粗测）`);
    }
    if (r.info.preset !== null && r.info.preset !== 0) sub.push(`Pn-303 现值=${r.info.preset}（非零，说明上一次没归零）`);
    out.push(`  ${r.ok ? 'OK  ' : 'FAIL'} ${r.wheel} ${r.name} ${r.port}/站${r.slave}  ` +
      `Fn-1C 额定转速=${r.info.ratedSpd ?? '?'} rpm  Fn-1A 电压=${r.info.ratedVolt ?? '?'}  ` +
      `Fn-1B 转矩=${r.info.ratedTq ?? '?'}  Fn-1D 电流=${r.info.ratedCur ?? '?'}  Fn-1E 极数=${r.info.poles ?? '?'}  Fn-21 反电势常数=${r.info.backEmf ?? '?'}` +
      (r.info.fallback ? '  [逐寄存器读]' : '') +
      (r.info.ceiling !== null ? `  | 当前 Pn-30D=${r.info.ceiling} raw${r.info.ceilingMps !== undefined ? ` ≈ ${r.info.ceilingMps.toFixed(3)} m/s` : ''}` : '') +
      sub.map((s) => `\n       └ ${s}`).join('') +
      (r.problems.length ? `\n       ⚠ ${r.problems.join('；')}` : ''));
  }
  if (p.need) {
    const n = p.need;
    out.push(`  目标 ${n.mps} m/s → ${n.raw} raw = 电机侧 ${n.motorRpm.toFixed(0)} rpm = 轮子 ${n.wheelRpm.toFixed(1)} rpm`);
    out.push(`  减速比 ±10% 的不确定区间：同样这个 raw，实际车速约 ${Math.min(n.mpsIfGearLow, n.mpsIfGearHigh).toFixed(2)}~${Math.max(n.mpsIfGearLow, n.mpsIfGearHigh).toFixed(2)} m/s`);
    const over = p.rows.filter((r) => r.info.overRated).map((r) => r.wheel);
    const low = p.rows.filter((r) => r.info.overCeiling).map((r) => r.wheel);
    if (over.length) out.push(`  ⛔ ${over.join(' ')} 目标超额定 → 不改机械就不做`);
    if (low.length) out.push(`  需要改 Pn-30D 的台子：${low.join(' ')}（写参数要单独确认）`);
    if (!over.length && !low.length) out.push(`  ${p.rows.every((r) => r.info.ratedPlausible) ? '结论：目标在额定与当前限幅之内，不需要写任何参数' : '结论：目标算术上可行，但额定转速还没读回来 → 先解决读数，再谈提速'}`);
  }
  out.push('  注：Fn-xx 铭牌组的通讯地址是 0x06xx（手册 §9.2），监控组 0x07xx 只到 0x0717；除 Fn-1C 标 rpm 外其余项倍率未核实，只显示原始值。');
  return out.join('\n');
}
