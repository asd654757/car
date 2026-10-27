/**
 * 车速档 → 控制参数：把"我要 X 米每秒"翻译成 raw / 量化档 / 斜坡三件套。
 *
 * 为什么要单独一层：限幅原来是人手填 raw 的。1.0 m/s = 14324 raw，这种数少写一位、多写一位
 * 就是 10 倍速的差别，而 10 倍速在一台 30 kg 的轮式底盘上是事故。现在只允许按 m/s 提要求，
 * raw 一律从机械参数派生，并被铭牌实测的额定转速（Fn-1C = 3000 rpm，2026-10-09 四台一致）硬夹住。
 *
 * 纯函数，不碰 IO，可翻译成 C。
 */

import { rawForSpeedMps, speedMpsFromRaw, fullScaleSpeedMps } from './kinematics.js';

/** 默认期望加速度：从 0 到满杆速度的时间尺度，决定斜坡每拍允许变化多少 */
export const DEFAULT_ACCEL_MPS2 = 0.8;
/**
 * 爬满速的时间窗口（秒）。太快 = 一次写入跳几千 raw，机械冲击；太慢 = 手感跟不上。
 * 为什么按"时间"而不是按"每拍变化量"定斜坡：非零写入受 minWriteIntervalMs 限频，
 * 被推迟的那些拍**不会累积**（planWrites 每次都从"线上值"起步），所以
 * 真正决定爬升速度的是"写入次数 × 限频间隔"，不是拍的个数。
 */
export const RAMP_TIME_WINDOW_S = { min: 1.0, max: 4.0 };

/** 额定转速对应的 raw —— 一切限幅的天花板（超过它就是超额定跑） */
export function ratedRaw(cfg) {
  const r = cfg.nameplate?.ratedSpdRpm * cfg.rawPerRpm;
  if (!Number.isFinite(r) || r <= 0) throw new Error('ratedRaw: 铭牌额定转速没标定（cfg.nameplate.ratedSpdRpm）→ 不许据此提速');
  return Math.floor(r);
}

/** 额定折到车速：这台车"物理上限"的权威值，不再靠口算 */
export function ratedSpeedMps(cfg) {
  return speedMpsFromRaw(ratedRaw(cfg), cfg);
}

/**
 * 建议写进 Pn-30D 的值：正好等于额定转速。
 * 一次写到位，之后所有车速档都在软件侧切，不再动参数（EEPROM 写次数是有限资源）。
 * 手册允许填到 30000 raw（6000 rpm），但那只是"能填"，不是"能跑"。
 */
export function ceilingForRated(cfg) {
  return Math.min(ratedRaw(cfg), cfg.pn30dManualMaxRaw ?? ratedRaw(cfg));
}

/**
 * 上了高速档之后的"起始幅度档"（界面用它决定进来是 1/4 还是 2/4）。
 * 原来固定从 2/4 起步，那是 60 rpm 架台档（满杆 300 raw ≈ 0.02 m/s）时代定的，半档无所谓。
 * --mps 1.0 时满杆 14324 raw，2/4 = 0.5 m/s —— 手柄一进来就是半速，而这一档正是给
 * "轮子离地 → 清场 → 逐档试"用的。所以满杆超过 SLOW_START_MPS 就从最低档起步。
 * 用户自己按过档位之后界面就不再覆盖（这是显示层的默认值，不参与限幅）。
 * @param {number} gears 总档数，与前端 GEARS 一致
 * @returns {number} 1..gears
 */
export const SLOW_START_MPS = 0.3;
export function suggestedStartGear(cfg, gears = 4) {
  if (!(Number.isFinite(gears) && gears >= 1)) throw new Error(`suggestedStartGear: gears 必须是正整数（收到 ${gears}）`);
  return fullScaleSpeedMps(cfg) > SLOW_START_MPS ? 1 : Math.min(2, Math.max(1, Math.floor(gears)));
}

/**
 * @param {number} mps 期望的满杆车速（m/s，空载）
 * @param {number} accelMps2 期望加速度，决定斜坡斜率
 * @returns 新的 cfg（不修改传入对象）
 * @throws 目标超过额定折算车速时直接拒绝 —— 这是"改机械才能做的事"，不是软件能豁免的
 */
export function speedPreset(cfg, mps, accelMps2 = DEFAULT_ACCEL_MPS2) {
  if (!(Number.isFinite(mps) && mps > 0)) throw new Error(`speedPreset: 车速必须是正有限数（收到 ${mps}）`);
  if (!(Number.isFinite(accelMps2) && accelMps2 > 0)) throw new Error(`speedPreset: 加速度必须是正有限数（收到 ${accelMps2}）`);
  const limit = ratedSpeedMps(cfg);
  if (mps > limit) {
    throw new Error(`目标 ${mps} m/s 超过额定折算上限 ${limit.toFixed(3)} m/s`
      + `（Fn-1C ${cfg.nameplate.ratedSpdRpm} rpm、减速比 ${cfg.mech.gearRatio}:1、轮径 ${cfg.mech.wheelDiameterM} m）`
      + ` → 想更快只能改机械，软件不给绕`);
  }
  const maxCmdRaw = Math.max(cfg.control.quantizeRaw, Math.round(rawForSpeedMps(mps, cfg)));
  // 稳态量化档 = 满量程的 1%，且不小于原档：低速小档（300 raw）行为完全不变，高速档不会出现"每拍都在写"
  const quantizeRaw = Math.max(cfg.control.quantizeRaw, Math.round((maxCmdRaw * 0.01) / 5) * 5);
  // 斜坡：先定"多少秒爬到满杆"，再除以限频间隔得到允许的写入次数，最后把满量程摊到这几次写入上。
  // 下限保留原 40 raw/拍：小速度档不许被"一次跳到位"。
  const gateS = cfg.control.minWriteIntervalMs / 1000;
  const timeToFullS = Math.min(RAMP_TIME_WINDOW_S.max, Math.max(RAMP_TIME_WINDOW_S.min, mps / accelMps2));
  const writesToFull = Math.max(1, Math.round(timeToFullS / gateS));
  const rampRawPerTick = Math.max(cfg.control.rampRawPerTick, Math.ceil(maxCmdRaw / writesToFull));
  return {
    ...cfg,
    control: { ...cfg.control, maxCmdRaw, quantizeRaw, rampRawPerTick },
    expect: { ...cfg.expect, maxCeilingRaw: ceilingForRated(cfg) },
    bench: { mps, accelMps2, ratedMps: limit, ratedRaw: ratedRaw(cfg), timeToFullS, writesToFull, gateS },
  };
}
