/**
 * Pn-30D 写入的"决定"部分 —— 纯函数，不碰串口，所以能被单测覆盖。
 *
 * 为什么要从 setceiling.js 里拆出来：写参数是不可逆动作，判据必须在**没有硬件**的情况下
 * 也能被反复验证（同样也是给后面 STM32 侧复用：这套闸门要跟到嵌入式里去）。
 *
 * 输入 rows 来自只读探测（readNameplate + 监控组），每行需要：
 *   { wheel, name, port, slave, info: { ratedSpd, ceiling, preset, spd } }
 * 缺字段一律按"读不到 = 不许写"处理，不做乐观推断。
 */

import { ceilingForRated } from './preset.js';

/** 各道闸门的口径，写在这里便于测试和以后调整 */
export const CEILING_RULES = {
  requireRatedPlausible: true, // Fn-1C 读不到或不在合理区间 → 不许写
  requirePn303Zero: true, // 线上还留着速度指令 → 不许写
  requireSpdZero: true, // 轮子真在转（哪怕是外力拖着）→ 不许写
};

/**
 * @param {object[]} rows 四轮只读探测结果
 * @param {object} cfg CONFIG
 * @param {number} requested 想写进 Pn-30D 的 raw
 * @returns {{ok:boolean, target:number|null, hardCap:number|null, problems:string[], rows:object[]}}
 */
export function planCeilingWrite(rows, cfg, requested) {
  const problems = [];
  const notes = []; // 不拦但要看见的判断（比如"不为零但在停转容差内"）
  if (!Array.isArray(rows) || rows.length !== cfg.wheels.length) {
    return { ok: false, target: null, hardCap: null, problems: [`探测结果不是 ${cfg.wheels.length} 行（收到 ${rows?.length}）→ 不许写`], notes, rows: rows || [] };
  }
  if (!Number.isInteger(requested) || requested < 1) {
    problems.push(`目标值必须是正整数 raw（收到 ${requested}）`);
  }

  const missing = rows.filter((r) => !r.info || !Number.isFinite(r.info.ratedSpd));
  if (missing.length) problems.push(`${missing.map((r) => r.wheel).join(' ')} 读不到 Fn-1C 额定转速 → 不知道上限就不抬上限`);
  const implausible = rows.filter((r) => r.info && Number.isFinite(r.info.ratedSpd) && r.info.ratedPlausible === false);
  if (implausible.length) problems.push(`${implausible.map((r) => `${r.wheel}(${r.info.ratedSpd} rpm)`).join(' ')} 额定转速不在合理区间 → 地址/组别存疑，不许写`);

  const noCeiling = rows.filter((r) => !r.info || !Number.isFinite(r.info.ceiling));
  if (noCeiling.length) problems.push(`${noCeiling.map((r) => r.wheel).join(' ')} 读不到当前 Pn-30D → 不知道现状就不改现状`);
  // 现状读到了，但落在这台设备不可能给出的区间里（0~30000 之外或负数）→ 同样算"不知道现状"。
  // 写参数是不可逆动作，宁可停在"先搞清读数"，也不要基于一个来路不明的数抬限幅。
  const manualMax = cfg.pn30dManualMaxRaw ?? 30000;
  const oddCeiling = rows.filter((r) => r.info && Number.isFinite(r.info.ceiling)
    && (r.info.ceilingPlausible === false || r.info.ceiling < 0 || r.info.ceiling > manualMax));
  if (oddCeiling.length) problems.push(`${oddCeiling.map((r) => `${r.wheel}(Pn-30D=${r.info.ceiling})`).join(' ')} 当前限幅读数超出手册范围 0~${manualMax} → 读数不可信，先查地址映射再写`);
  const noPreset = rows.filter((r) => !r.info || !Number.isFinite(r.info.preset));
  if (noPreset.length) problems.push(`${noPreset.map((r) => r.wheel).join(' ')} 读不到 Pn-303 → 确认不了它是 0，不许写`);
  const nonzeroPreset = rows.filter((r) => r.info && Number.isFinite(r.info.preset) && r.info.preset !== 0);
  if (nonzeroPreset.length) problems.push(`${nonzeroPreset.map((r) => `${r.wheel}(Pn-303=${r.info.preset})`).join(' ')} 线上还留着速度指令 → 先归零再改限幅`);
  const idleTol = Number.isFinite(cfg.expect?.spdIdleToleranceRpm) ? cfg.expect.spdIdleToleranceRpm : 0;
  const spinning = rows.filter((r) => r.info && Number.isFinite(r.info.spd) && Math.abs(r.info.spd) > idleTol);
  if (spinning.length) problems.push(`${spinning.map((r) => `${r.wheel}(SPd=${r.info.spd} rpm)`).join(' ')} 正在转（超出停转容差 ${idleTol} rpm）→ 现场确认安全后再改限幅`);
  // 不为零但在停转容差内：不拦，但要把这个"我看到了却没当成问题"的判断摆在报告里，
  // 免得以后出事故时回头找不到当时到底读到了什么。
  const jitter = rows.filter((r) => r.info && Number.isFinite(r.info.spd) && r.info.spd !== 0 && Math.abs(r.info.spd) <= idleTol);
  if (jitter.length) notes.push(`注：${jitter.map((r) => `${r.wheel}(SPd=${r.info.spd})`).join(' ')} 在停转容差 ${idleTol} rpm 内（Pn-303=0，线上传令为零）→ 判为静态测速抖动，不判为"在转"`);
  const noSpd = rows.filter((r) => !r.info || !Number.isFinite(r.info.spd));
  if (noSpd.length) problems.push(`${noSpd.map((r) => r.wheel).join(' ')} 读不到实际转速 → 确认不了"它停着"，不许写`);

  // 天花板：四台里**最小**的额定 raw，再和手册上限取小。软顶可以更低，但不能高于这个。
  const ratedRaws = rows.filter((r) => r.info && Number.isFinite(r.info.ratedSpd)).map((r) => Math.floor(r.info.ratedSpd * cfg.rawPerRpm));
  const hardCap = ratedRaws.length ? Math.min(Math.min(...ratedRaws), cfg.pn30dManualMaxRaw ?? Infinity, ceilingForRated(cfg)) : null;
  if (hardCap === null) problems.push('没有任何一台给出可信额定转速 → 没有天花板，不许写');
  else if (Number.isInteger(requested) && requested > hardCap) {
    problems.push(`目标 ${requested} 超过允许值 ${hardCap}（四台最小额定 ${hardCap / cfg.rawPerRpm} rpm，手册上限 ${cfg.pn30dManualMaxRaw}）`);
  }
  if (Number.isInteger(requested) && requested > (cfg.pn30dManualMaxRaw ?? Infinity)) {
    problems.push(`目标 ${requested} 超出手册设定范围上限 ${cfg.pn30dManualMaxRaw}`);
  }
  return { ok: problems.length === 0, target: requested, hardCap, problems, notes, rows };
}

/** 把计划打成人话（setceiling.js 和测试都用它，保证"看到的"和"判的"是同一份） */
export function formatCeilingPlan(plan, cfg) {
  const mps = (raw) => (Number.isFinite(raw) ? (raw / cfg.rawPerRpm / cfg.mech.gearRatio) * (Math.PI * cfg.mech.wheelDiameterM) / 60 : NaN);
  const out = [`Pn-30D 写入计划（${plan.ok ? '闸门通过' : `闸门拦截 ${plan.problems.length} 条`}）：`];
  for (const r of plan.rows) {
    const i = r.info || {};
    out.push(`  ${r.wheel} ${r.name} ${r.port}/站${r.slave}：额定 ${i.ratedSpd ?? '?'} rpm，当前 Pn-30D=${i.ceiling ?? '?'} ≈ ${Number.isFinite(mps(i.ceiling)) ? mps(i.ceiling).toFixed(4) : '?'} m/s，`
      + `Pn-303=${i.preset ?? '?'}，SPd=${i.spd ?? '?'} → 目标 ${plan.target}`);
  }
  if (plan.hardCap !== null) out.push(`  允许值天花板 ${plan.hardCap} raw = ${plan.hardCap / cfg.rawPerRpm} rpm ≈ ${mps(plan.hardCap).toFixed(3)} m/s（四台最小额定）`);
  (plan.notes || []).forEach((n) => out.push(`  ${n}`));
  plan.problems.forEach((p) => out.push(`  ✋ ${p}`));
  if (plan.ok) {
    // 使能状态读不到（0x07xx 监控组里没有这一位），所以只能把它写成现场动作提示。
    // 方向是更安全的一边：断使能 = 轮子不会动，而手册又要求"非运行中"才改参数（Sd-On）。
    out.push('  写之前先把四台使能断开（面板停止 / 断 SON）—— 手册：伺服器正在运行中不能改参数（Sd-On）；断使能只会让轮子更不可能动。');
  }
  return out.join('\n');
}
