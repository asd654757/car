/**
 * 写入节流 —— 保护驱动器 EEPROM，同时保证"停"永远最快。
 *
 * 为什么必须有：`Pn-303` 是掉电保持参数（实测断电后值还在），手册故障表里有
 * "08 EEPROM 超出写入次数"。按 20 Hz 每帧刷 = 一小时 7 万次，会把驱动器写坏。
 *
 * 三条规则（按优先级）：
 *  1. **归零不受任何限制**：急停、看门狗、退出清理都必须立刻落到线上。
 *  2. 量化：目标值先归到 quantizeRaw 一档（默认 10 raw = 2 rpm 电机侧），稳态行驶零写入；
 *     量化结果越过 maxCmdRaw 时**向下退一整档**（不贴 cap，避免半档抖动来回写），硬顶才是硬顶。
 *  3. 限频 + 斜坡：非零写入之间至少隔 minWriteIntervalMs，且每拍最多变 rampRawPerTick。
 *
 * 纯函数 + 显式状态，可整体翻成 C。
 */

export function makeThrottleState(nWheels) {
  return {
    written: new Array(nWheels).fill(0), // 当前在线上的值（初始假定 Pn-303 = 0，预检会核实）
    // -Infinity：开机第一帧不受限频约束。用 0 会让 t=0 那次写入被自己"限"掉。
    lastWriteAt: new Array(nWheels).fill(-Infinity),
    counts: { writes: 0, deferred: 0, stops: 0, failed: 0 },
  };
}

export function quantize(v, step, cap) {
  if (!Number.isFinite(v)) throw new Error(`quantize: 目标值非有限数（${v}）`);
  if (!Number.isFinite(step) || step < 1) throw new Error(`quantize: step 非法（${step}）`);
  if (cap !== undefined && cap !== null && !(Number.isFinite(cap) && cap > 0)) {
    throw new Error(`quantize: cap 非法（${cap}）——要么不传，要么给正的 maxCmdRaw`);
  }
  const q = step <= 1 ? Math.round(v) : Math.round(v / step) * step;
  if (cap === undefined || cap === null) return q;
  if (Math.abs(q) <= cap) return q;
  // 四舍五入只许往下不许往上：--max 75 时 Math.round(75/10)*10 = 80，硬顶被量化悄悄抬高 → maxCmdRaw 就不成硬顶了。
  // 越过 cap 时向下退一整档，**不返回 cap 本身**：74 会量化成 70，若这里返回 75，
  // 输入在 74/75 附近抖一下就会产生 70↔75 的反复写入 —— 正是要防的 EEPROM 磨损。
  const byStep = Math.sign(v) * (Math.floor(cap / step) * step);
  return Number.isFinite(byStep) && byStep !== 0 ? byStep : Math.sign(v) * cap;
}

/** 斜坡：把 next 相对 current 的变化量限制在 maxDelta 内 */
export function rampToward(current, next, maxDelta) {
  const d = next - current;
  if (Math.abs(d) <= maxDelta) return next;
  return current + Math.sign(d) * maxDelta;
}

/**
 * 决定这一拍要写哪几路。
 * @param {object} state      makeThrottleState()
 * @param {number[]} target   四轮目标 raw（带符号，已限幅）
 * @param {number} nowMs      Date.now() 或单调时钟
 * @param {object} cfg        CONFIG.control
 * @returns {{writes:{idx:number,value:number,prev:number}[], state:object, deferred:number[]}}
 *   注意 writes 里的值已经被**乐观地**记进 state.written —— 发帧失败必须调 rollbackWrites 退回，
 *   否则节流器会以为命令已生效而永远不再重试（这台驱动器没有断线停车，后果是保持原速一直转）。
 */
export function planWrites(state, target, nowMs, cfg) {
  // 传错对象（把整个 CONFIG 当 CONFIG.control 传进来）会让所有值静默变成 NaN —— 必须先炸出来
  for (const k of ['quantizeRaw', 'minWriteIntervalMs', 'rampRawPerTick', 'maxCmdRaw']) {
    if (!Number.isFinite(cfg?.[k])) throw new Error(`planWrites: cfg.${k} 非法（${cfg?.[k]}）——这里要传 CONFIG.control`);
  }
  if (!Number.isFinite(nowMs)) throw new Error(`planWrites: nowMs 非法（${nowMs}）`);
  const writes = [];
  const deferred = [];
  target.forEach((t, i) => {
    const q = quantize(t, cfg.quantizeRaw, cfg.maxCmdRaw);

    // 规则 1 优先于斜坡：只要"目标就是停"，立刻写 0，不允许被 rampRawPerTick 拖成 260、220……
    if (q === 0) {
      if (state.written[i] === 0) return;
      writes.push({ idx: i, value: 0, prev: state.written[i] });
      state.written[i] = 0;
      state.lastWriteAt[i] = nowMs;
      state.counts.writes++;
      state.counts.stops++;
      return;
    }

    const limited = cfg.rampRawPerTick > 0
      ? rampToward(state.written[i], q, cfg.rampRawPerTick)
      : q;
    if (limited === state.written[i]) return; // 与线上值一致 → 一个字节都不发

    // 规则 3：非零写入才限频
    if (nowMs - state.lastWriteAt[i] < cfg.minWriteIntervalMs) {
      deferred.push(i); state.counts.deferred++; return;
    }

    writes.push({ idx: i, value: limited, prev: state.written[i] });
    state.written[i] = limited;
    state.lastWriteAt[i] = nowMs;
    state.counts.writes++;
  });
  return { writes, deferred, state };
}

/** 发帧失败 → 把"线上值"退回原值，下一拍自动重试。归零失败尤其要退，否则悄悄停在半速。 */
export function rollbackWrites(state, failed) {
  for (const f of failed || []) {
    if (typeof f.idx !== 'number' || typeof f.prev !== 'number') continue;
    state.written[f.idx] = f.prev;
    state.counts.writes = Math.max(0, state.counts.writes - 1);
    state.counts.failed++;
  }
  return state;
}

/** 强制四轮归零，绕过量化/斜坡/限频（急停与退出清理走这条） */
export function planStopAll(state) {
  const writes = [];
  state.written.forEach((v, i) => { if (v !== 0) writes.push({ idx: i, value: 0, prev: v }); });
  writes.forEach((w) => { state.written[w.idx] = 0; });
  state.counts.stops += writes.length;
  state.counts.writes += writes.length;
  return writes;
}
