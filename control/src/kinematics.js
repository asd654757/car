/**
 * 麦轮运动学 —— 纯函数，无 IO、无 Node 依赖，可逐行翻译成 C。
 *
 * 坐标系（右手，俯视）：x 朝车头，y 朝车身左，ω 逆时针为正。
 * 轮速正方向 = "该轮向前滚"（轮顶朝车头）。
 *
 * 公式与实机滚轮斜向图一致（1 左前与 3 右后同为 `\`、2 左后与 4 右前同为 `/`，
 * 即对角同斜向的标准 X 型装法），已由 2026-10-09 四轮直行自检验证。
 */

/** 四个轮在结果数组里的固定下标，与 CONFIG.wheels 顺序一致 */
export const W = { FL: 0, FR: 1, BL: 2, BR: 3 };
export const WHEEL_IDS = ['FL', 'FR', 'BL', 'BR'];

/**
 * 底盘 twist → 四轮向前滚的线速度（m/s）。
 * @param {{vx:number, vy:number, wz:number}} twist m/s, m/s, rad/s
 * @param {{lxPlusLyM:number}} mech
 * @returns {number[]} [FL, FR, BL, BR]
 */
export function twistToWheelMps(twist, mech) {
  const k = mech.lxPlusLyM;
  const { vx, vy, wz } = twist;
  return [
    vx - vy - k * wz, // FL 左前
    vx + vy + k * wz, // FR 右前
    vx + vy - k * wz, // BL 左后
    vx - vy + k * wz, // BR 右后
  ];
}

/** 四轮线速度（m/s，向前滚为正）→ 轮子转速 rpm */
export function mpsToWheelRpm(mpsArr, mech) {
  const circ = Math.PI * mech.wheelDiameterM;
  return mpsArr.map((v) => (v / circ) * 60);
}

/** 轮子转速 rpm → 电机侧 raw（未套符号）。Pn-3xx 单位 0.2 转/分，且是**电机侧**量 */
export function wheelRpmToMotorRaw(rpmArr, mech, rawPerRpm) {
  return rpmArr.map((v) => v * mech.gearRatio * rawPerRpm);
}

/** 电机侧 raw → 轮子转速 rpm（遥测反算用） */
export function motorRawToWheelRpm(rawArr, mech, rawPerRpm) {
  return rawArr.map((v) => v / mech.gearRatio / rawPerRpm);
}

/** 套上实机标定的符号矩阵：sign=+1 的轮写正数即向前滚（实测：前 +1、后 -1） */
export function applySigns(rawArr, signs) {
  return rawArr.map((v, i) => v * signs[i]);
}

/** 等比缩放到 |每路| ≤ cap：保持运动方向不变，比逐路削顶安全 */
export function scaleToCap(values, cap) {
  const peak = values.reduce((m, v) => Math.max(m, Math.abs(v)), 0);
  if (peak === 0 || peak <= cap) return { values: values.slice(), scaled: false, factor: 1 };
  const factor = cap / peak;
  return { values: values.map((v) => v * factor), scaled: true, factor };
}

/**
 * 目标车速（m/s）→ 电机侧 raw。
 * 限幅应当"按车速定"，不是让人手算 0.2 转/分 —— 手算一次就有一次算错的机会。
 * ⚠ 结果完全依赖 mech.gearRatio 与 mech.wheelDiameterM 这两个实测值，它们错 10%，车速就错 10%。
 */
export function rawForSpeedMps(mps, cfg) {
  const circ = Math.PI * cfg.mech.wheelDiameterM;
  return (mps * 60 / circ) * cfg.mech.gearRatio * cfg.rawPerRpm;
}

/** 电机侧 raw → 车速 m/s（日志、界面、"这个限幅到底多快"都走这里） */
export function speedMpsFromRaw(raw, cfg) {
  const circ = Math.PI * cfg.mech.wheelDiameterM;
  return (raw / cfg.rawPerRpm / cfg.mech.gearRatio) * circ / 60;
}

/**
 * 满杆速度：纯前进时，让电机侧刚好打到 maxCmdRaw 的那个 vx（m/s）。
 * 输入层用它把归一化摇量换算成物理 twist。
 */
export function fullScaleSpeedMps(cfg) {
  return speedMpsFromRaw(cfg.control.maxCmdRaw, cfg);
}

/**
 * 控制环每拍调用的唯一入口：归一化输入 → 四轮 Pn-303 目标值（带符号、已限幅、已取整）。
 * @param {{vx:number, vy:number, wz:number}} norm 各分量 ∈ [-1,1]
 */
export function twistToRaw(norm, cfg) {
  const clamp = (v) => Math.max(-1, Math.min(1, Number.isFinite(v) ? v : 0));
  const n = { vx: clamp(norm.vx), vy: clamp(norm.vy), wz: clamp(norm.wz) };
  const vFull = fullScaleSpeedMps(cfg);
  const mps = twistToWheelMps({ vx: n.vx * vFull, vy: n.vy * vFull, wz: n.wz * vFull / cfg.mech.lxPlusLyM }, cfg.mech);
  const rawUnsigned = wheelRpmToMotorRaw(mpsToWheelRpm(mps, cfg.mech), cfg.mech, cfg.rawPerRpm);
  const { values, scaled } = scaleToCap(rawUnsigned, cfg.control.maxCmdRaw);
  const signed = applySigns(values, cfg.wheels.map((w) => w.sign));
  return { raw: signed.map((v) => Math.round(v) || 0), scaled, wheelMps: mps, vFull }; // `|| 0` 顺手把 -0 归正

}

/** 四轮向前滚速度（同单位）→ 底盘 twist。里程计用。 */
export function wheelToTwist(w, mech) {
  const k = mech.lxPlusLyM;
  const [fl, fr, bl, br] = w;
  return {
    vx: (fl + fr + bl + br) / 4,
    vy: (-fl + fr + bl - br) / 4,
    wz: (-fl + fr - bl + br) / (4 * k),
  };
}

/** 把 twist 说成人话，日志和界面都用它 */
export function describeTwist(t) {
  if (Math.abs(t.vx) < 1e-6 && Math.abs(t.vy) < 1e-6 && Math.abs(t.wz) < 1e-6) return '停止';
  const parts = [];
  if (Math.abs(t.vx) > 0.05) parts.push(t.vx > 0 ? '前进' : '后退');
  if (Math.abs(t.vy) > 0.05) parts.push(t.vy > 0 ? '左移' : '右移');
  if (Math.abs(t.wz) > 0.05) parts.push(t.wz > 0 ? '逆时针自转' : '顺时针自转');
  return parts.join(' + ') || '微动';
}
