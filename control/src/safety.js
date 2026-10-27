/**
 * 安全层 —— 看门狗、急停闭锁、通讯失效闭锁。纯状态机，可翻成 C。
 *
 * 为什么必须有：手册里**没有任何"通讯超时自动停车"功能**（查遍 Pn/Fn 组），
 * 也就是说 PC 掉线 / 程序崩了，驱动器会保持最后一次速度一直转。
 * 所以停车只能靠上位机自己做，而且要做成"默认停"：拿不到明确许可就不许动。
 */

export const STATE = { IDLE: 'idle', RUN: 'run', ESTOP: 'estop', FAULT: 'fault' };

export function makeSafetyState() {
  // -1 = "从没发生过"。不能用 0 当哨兵：0 是一个合法时间戳，测试里（以及翻成 C 后用毫秒计时器）会撞车。
  return { state: STATE.IDLE, lastInputAt: -1, lastOkAt: -1, failStreak: 0, reason: '未启动' };
}

/** 输入到达（键盘/手柄任一） */
export function noteInput(st, nowMs) { st.lastInputAt = nowMs; }

/** 一次总线交互的结果 */
export function noteComm(st, nowMs, ok) {
  if (ok) { st.failStreak = 0; st.lastOkAt = nowMs; }
  else st.failStreak++;
  return st;
}

export function latchEstop(st, reason = '急停') { st.state = STATE.ESTOP; st.reason = reason; }
export function latchFault(st, reason) { st.state = STATE.FAULT; st.reason = reason; }
/** 只有显式解除才允许恢复；急停闭锁不自动解锁 */
export function clearLatch(st) {
  if (st.state === STATE.ESTOP || st.state === STATE.FAULT) { st.state = STATE.IDLE; st.reason = '已解除，等待输入'; }
  return st;
}

/**
 * 每拍决定：这一拍的指令要不要强制归零。
 * @returns {{forceZero:boolean, state:string, reason:string}}
 */
export function decide(st, nowMs, cfg) {
  if (st.state === STATE.ESTOP) return { forceZero: true, state: st.state, reason: `急停闭锁：${st.reason}` };
  if (st.state === STATE.FAULT) return { forceZero: true, state: st.state, reason: `故障闭锁：${st.reason}` };
  if (cfg.commFailLimit && st.failStreak >= cfg.commFailLimit) {
    latchFault(st, `连续 ${st.failStreak} 次通讯失败`);
    return { forceZero: true, state: st.state, reason: st.reason };
  }
  const age = nowMs - st.lastInputAt;
  if (st.lastInputAt < 0) {
    st.state = STATE.IDLE; st.reason = '等待输入';
    return { forceZero: true, state: st.state, reason: st.reason };
  }
  if (age > cfg.inputTimeoutMs) {
    if (st.state !== STATE.IDLE) { st.state = STATE.IDLE; st.reason = `输入超时 ${age}ms → 停`; }
    return { forceZero: true, state: st.state, reason: st.reason };
  }
  st.state = STATE.RUN;
  st.reason = '正常';
  return { forceZero: false, state: st.state, reason: st.reason };
}
