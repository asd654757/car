/**
 * 键盘前端 —— 纯逻辑，不碰 tty、不碰串口，可单测，也是将来 C 侧"输入映射"层的对应物。
 *
 * 这里有一条必须显式处理的坑：`setRawMode(true)` 之后，Ctrl+C **不再**产生 SIGINT，
 * 终端驱动会把 0x03 当普通字节喂进 stdin。不认它，交互终端里就没有干净退出 ——
 * 而"没干净退出"在这台机器上意味着最后的 `Pn-303` 留在驱动器里，四轮保持那个速度继续转
 * （驱动器自己没有断线停车功能）。
 */

/** 键 → 轴与方向。坐标口径与 kinematics.js 一致：vx 前进、vy 左移、wz 逆时针。 */
export const KEYMAP = { w: 'vx+', s: 'vx-', a: 'vy+', d: 'vy-', q: 'wz+', e: 'wz-' };
export const CTRL_C = '\x03';
export const GEAR_STEPS = 4;

/**
 * 处理一个输入字符。api 是一组回调（exit/estop/release/zero/clearKeys/help/snapshot/gear/press）。
 * @returns {string} 命中的动作名，方便日志与测试
 */
export function dispatchKey(ch, api) {
  if (ch === CTRL_C) { api.exit('Ctrl+C'); return 'exit'; }
  if (ch === ' ') { api.estop('空格急停'); return 'estop'; }
  const k = typeof ch === 'string' && ch.length === 1 ? ch.toLowerCase() : '';
  if (k === 'r') { api.release(); return 'release'; }
  if (k === 'x') { api.clearKeys(); api.zero(); return 'zero'; }
  if (k === 'h') { api.help(); return 'help'; }
  if (k === 'c') { api.snapshot(); return 'snapshot'; }
  if (k >= '1' && k <= '4') { api.gear(Number(k)); return 'gear'; }
  if (KEYMAP[k]) { api.press(k); return 'press'; }
  return 'ignore';
}

/** 同一根轴上两个相反键互相抵消，不会叠成 ±2 */
export function twistFromKeys(held, gear = 2, steps = GEAR_STEPS) {
  const t = { vx: 0, vy: 0, wz: 0 };
  for (const k of held) {
    const m = KEYMAP[k];
    if (!m) continue;
    const axis = m.slice(0, 2), sign = m[2] === '+' ? 1 : -1;
    t[axis] = Math.max(-1, Math.min(1, t[axis] + sign));
  }
  // 档位一旦被配坏（NaN、0、超界），宁可当 0 档处理：
  // NaN 会顺着运动学一路传到四轮目标，控制核那边只能"整拍归零"，等于莫名其妙不会动了。
  const g = (Number.isFinite(gear) ? Math.max(0, Math.min(steps, gear)) : 0) / steps;
  return { vx: t.vx * g, vy: t.vy * g, wz: t.wz * g };
}

/**
 * 终端 raw 模式没有 keyup，只能靠"系统自动重复还在不在"判断松手：
 * 某个键超过 grace 毫秒没再出现重复事件，就认为已经放开。
 * @returns {string[]} 被判定为已松开的键
 */
export function sweepHeld(held, keySeen, nowMs, graceMs) {
  const dropped = [];
  for (const k of Array.from(held)) {
    if (!(nowMs - (keySeen[k] || 0) > graceMs)) continue;
    held.delete(k);
    dropped.push(k);
  }
  return dropped;
}
