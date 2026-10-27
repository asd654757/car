/**
 * 标定常量 —— 全部来自 2026-10-09 实机标定，唯一来源。
 * 依据：E:\底盘\麦轮底盘四轮标定记录.md（四轮直行自检通过）
 *
 * 本文件是纯数据，不 import 任何 Node 内置模块，方便整体翻译成 C 烧进 STM32H743。
 * 唯一的外部依赖是 ./ports.js —— 它也只读全局 process，且只负责"串口叫什么名"这种
 * 部署配置；翻译成 C 时那部分由板级引脚表（board_pins_h743.h）替代，不冲突。
 */

import { resolvePorts } from './ports.js';

/** Modbus / 驱动器层的量纲（实测） */
export const RAW_PER_RPM = 5; // Pn-3xx 单位 = 0.2 转/分 → raw = rpm × 5。注意：这是**电机侧**转速
export const SPEED_REG = 0x0303; // 内部预设速度 3 号 = Pn-303。实测这台机器的命令源是它，不是 Pn-304
export const MODE_REG = 0x0000; // Pn-000
export const CEILING_REG = 0x030d; // Pn-30D 速度上限
export const MONITOR_REG = 0x0700; // 监控组首地址
export const MONITOR_QTY = 24; // 0x0700~0x0717，跨组读会返回异常 0x04

/** 监控组里我们真正要看的几个（全是**电机侧**读数，轮子还要除以 gearRatio） */
export const MON = {
  spd: 0x0706, // 实际转速 rpm
  cmd: 0x0707, // 驱动器执行中的速度命令
  torque: 0x0709, // 平均负载/转矩
  pos: 0x0714, // 编码器位置（16384 ppr，单圈内回绕，只适合看"有没有在动"）
  temp: 0x0717, // 标 td，疑似温度；1/2 号读出 0，语义未核实 —— 别拿它做判据
  /**
   * 铭牌组（Fn-18~Fn-1E）：**地址在 0x06xx，不在 0x07xx**，见下面 MON 里那段实机教训。
   * 提转速前必须先读 ratedSpd：它才是电机能跑多快的物理上限，
   * Pn-30D 只是"允许写到的限幅"（手册范围 0~30000 raw = 0~6000 rpm），两者不是一回事。
   * 单位：手册只给了名称，除 ratedSpd 标 rpm 外其余倍率未核实 → 打印时注明原始值。
   */
  // 铭牌组 = **Fn-xx → 通讯地址 0x06xx**（手册 §9.2 分组表：P0~P5→0x00~0x05xx，Fn→0x06xx，监控→0x07xx）。
  // ⚠ 曾经按 0x071a~0x071e 找过：2026-10-09 实机只读探测四台全部读不到 —— 0x07xx 监控组按手册到
  //   0x0717 "td" 就结束了（那次 0x071a 还回过 10851 这种越界野值，不能信）。地址结论固化在这里，别再猜。
  //   除 ratedSpd 标 rpm 外，其余项倍率手册未给 → 只显示原始值，不拿来算物理量。
  ratedPow: 0x0619, // Fn-19 额定功率（Kw）
  ratedVolt: 0x061a, // Fn-1A 额定电压（V）
  ratedTq: 0x061b, // Fn-1B 额定转矩（Nm）
  ratedSpd: 0x061c, // Fn-1C 额定转速（rpm）← 提速前必量，这才是物理上限
  ratedCur: 0x061d, // Fn-1D 额定电流（A）
  poles: 0x061e, // Fn-1E 电机极数
  rotorInertia: 0x061f, // Fn-1F 转子惯量（Kgm2）
  tqConst: 0x0620, // Fn-20 转矩常数（Nm/Arms）
  backEmf: 0x0621, // Fn-21 相反电势常数（V/Krpm）——可用母线电压交叉核验能不能到额定
};
export const NAMEPLATE_REG = 0x0619; // Fn-19 起，一次读 8 个覆盖到 Fn-20；块读失败时逐寄存器兜底
export const NAMEPLATE_QTY = 8;
/** 额定转速的合理区间：越界说明地址/组别还是错的，不许拿它做限速决策 */
export const RATED_SPD_PLAUSIBLE = { min: 50, max: 6000 }; // 6000 rpm = Pn-30D 手册上限（30000 raw）对应的电机侧转速

/**
 * 铭牌实测读数（2026-10-09 只读探测，四台完全一致 → `node src/cli.js --motors`）。
 * 这一小块是"物理上限"的来源：提速的所有决策都从 ratedSpdRpm 往下推，不再靠假设。
 *  - 单位按手册：Fn-1C 标 rpm、Fn-1A 标 V、Fn-1D 标 A；极数无量纲。
 *  - Fn-21 反电势常数读出 32，但手册没给倍率口径：按 V/Krpm 解释会得到"3000 rpm 需要 96 V"，
 *    与 34 V 额定电压矛盾 → 语义未核实，禁止拿它推算可达转速，只作为"实机到不了额定时的线索"。
 */
export const NAMEPLATE = {
  ratedSpdRpm: 3000, // Fn-1C 电机侧额定转速 = 物理上限（空载会一直升到它）
  ratedVoltV: 34, // Fn-1A
  ratedTqNm: 1, // Fn-1B（读数 1，倍率存疑：1 N·m 还是 0.1 N·m？别拿去做力矩决策）
  ratedCurA: 11, // Fn-1D
  poles: 8, // Fn-1E
};
/** Pn-30D 的手册设定范围上限，任何情况下都不许越过（比额定高出一大截，不是许可） */
export const PN30D_MANUAL_MAX_RAW = 30000;

/**
 * 不加 `--mps` 时的默认车速档（满杆、空载）。唯一来源在这里，命令行与界面都从它派生。
 *
 * 为什么是 0.5：2026-10-09 离地四轮实测通过的就是这一档（1/4→4/4 逐档，实际转速达到命令的
 * 约 99%，四轮彼此 <2 rpm，通讯零失败）。在此之前"裸启动"落在 CONTROL.maxCmdRaw=300
 * （60 rpm 电机侧 ≈ 0.02 m/s），那是台架点动时代的遗留，手柄一推只有蠕行，不算能用的车。
 *
 * 这个值是**上限**而不是常态：满杆 0.5 m/s，起始幅度档仍被 suggestedStartGear 压到 1/4（≈0.13 m/s），
 * 因为 fullScale > SLOW_START_MPS(0.3)。想临时回到最低风险台架档就 `--mps 0`（回到 raw 300 硬顶），
 * 想提速就显式写 `--mps 0.8` 之类 —— 提速必须重新做安全确认，所以那一档故意不给默认。
 */
export const DEFAULT_TARGET_MPS = 0.5;

/** 期望的驱动器状态（预检用，不满足就拒绝启动） */
export const EXPECT = {
  modeNibble1: 3, // Pn-000 第1位=3 → 内部设定速度模式（nibble 编码，第0位在最右）
  /**
   * 预检允许看到的 Pn-30D 上限。2026-10-09 经逐条确认后把四台写成 15000（=额定 3000 rpm 对应值）并回读确认，
   * 所以这里的口径从"台架 300"改成"额定对应值"：仍然拦出厂默认 30000（6000 rpm，超额定）和任何越界读数。
   * ⚠ 这条一改，驱动器侧那道"自动截断大命令"的物理保险就没了：脚本层唯一的闸是 control.maxCmdRaw
   *   （本字段裸值 300 只是"不跑控制环时的兜底"，实际入场值由 DEFAULT_TARGET_MPS 的 --mps 档派生），
   *   并且预检会检查 maxCmdRaw ≤ 这个上限。
   */
  maxCeilingRaw: Math.min(Math.floor(NAMEPLATE.ratedSpdRpm * RAW_PER_RPM), PN30D_MANUAL_MAX_RAW),
  mustIdleReg: SPEED_REG, // 启动前该寄存器必须为 0，否则一上使能就自转
  ratedSpdPlausible: RATED_SPD_PLAUSIBLE, // 铭牌 Fn-1C 的合理区间：越界就当"没读到"，不拿野值做限速决策
  /**
   * "停着"的判定容差（电机侧 rpm，驱动器自己报的 C.SPd）。
   * 2026-10-09 实机：四台 Pn-303 都是 0（线上没有任何速度指令），但 FL/BR 稳定报 SPd=-1。
   * 1 rpm 电机侧 = 轮子 0.033 rpm = 0.00035 m/s —— 这是测速的最小分辨率/静态抖动，不是车在动。
   * 卡成"必须正好等于 0"会让闸门永远过不去；但容差只给到 2，因为真正的幽灵命令是 -59 那个量级，
   * 放宽到能盖住它的程度就失去意义了。读不到转速仍然一律按不安全处理。
   */
  spdIdleToleranceRpm: 2,
};

/**
 * 四轮定义。
 *  - sign: +1 表示"写正数 = 该轮向前滚"。实测结论是**前轮 +1、后轮 -1**，
 *    与 Pn-000 方向位分组（1/3 vs 2/4）、滚轮斜向分组（1/3 vs 2/4）都不一样，
 *    不要试图用任何规律外推，改轮子/改接线后必须重测。
 *  - slave: 一口一从站（WCH CH9114），n 号口挂站号 n，一个 COM 只能被一个进程占用。
 *  - port: **不在这里写死**，由 src/ports.js 按平台/环境变量解析（Windows 走 COM12-15 默认，
 *    Ubuntu 走 /dev/ttyCH343USB0-3，临时改线用 CHASSIS_PORT_FL=… 或整串偏移 CHASSIS_PORT_FIRST=…）。
 *    站号、符号、轮序是实机标定量，端口名只是部署配置，两者故意分开。
 */
const resolvedPorts = resolvePorts();
/** 轮号 → 串口路径（{FL,FR,BL,BR}），以及它的来源与警告，供启动时打印 */
export const PORTS = resolvedPorts.ports;
export const PORT_SOURCE = resolvedPorts.source;
export const PORT_WARNINGS = resolvedPorts.warnings;
export const WHEELS = [
  { id: 'FL', name: '左前', port: PORTS.FL, slave: 1, sign: +1 },
  { id: 'FR', name: '右前', port: PORTS.FR, slave: 4, sign: +1 },
  { id: 'BL', name: '左后', port: PORTS.BL, slave: 2, sign: -1 },
  { id: 'BR', name: '右后', port: PORTS.BR, slave: 3, sign: -1 },
];

/** 顺序约定：运动学输出、总线并发写、显示都按这个数组下标走 */
export const WHEEL_INDEX = { FL: 0, FR: 1, BL: 2, BR: 3 };

/**
 * 机械参数。gearRatio 是粗测值（电机 39 圈 ≈ 轮子 1.3 圈 → ≈30:1，误差 ±10%），
 * wheelDiameterM 是 2026-10-09 用户口述"大概 20cm"，未上卡尺。
 *
 * ⚠ 这两个数以前只影响 m/s 的显示；现在 `--mps` 用它反推限幅 raw，
 * 也就是说**它们错了，车速就真错**（±10% 的减速比 = 1.0 m/s 变成 0.91~1.11 m/s）。
 * 提转速前先按教程里"数轮子圈数"的方法把比值量准，落地后再用"量一米"复核。
 */
export const MECH = {
  gearRatio: 30, // 电机转 / 轮子转（粗测 28.6，±10%，待量准）
  wheelDiameterM: 0.20, // 轮子直径（用户口述，待卡尺复核）
  lxPlusLyM: 0.25, // 半宽+半轴距，(lx+ly)，待实测；只影响原地转与直行的比例
};

/** 控制环与安全参数 */
export const CONTROL = {
  loopHz: 20, // 实测单帧 round-trip 25~46 ms，四口并行 → 整机 35~45 ms
  maxCmdRaw: 300, // 60 rpm 电机侧 ≈ 0.02 m/s。裸值，只在 `--mps 0`（回到台架档）时生效；正常入场值见 DEFAULT_TARGET_MPS
  quantizeRaw: 10, // 2 rpm 一档：稳态行驶零写入，保护 EEPROM（手册故障 08 = EEPROM 超出写入次数）
  minWriteIntervalMs: 250, // 非零指令的限频；**归零指令不受此限制**
  inputTimeoutMs: 400, // 超过这么久没收到输入 → 立即四轮归零
  rampRawPerTick: 40, // 每次循环最多变化 8 rpm，避免突跳
  slewStopBypass: true, // 急停/归零绕过斜坡与限频
  commFailLimit: 3, // 连续 3 次总线交互失败 → 故障闭锁（驱动器没有断线停车，只能自己兜底）
  commProbeMs: 1000, // 没有写入的拍做一次只读心跳，否则链路断了也发现不了（详见 controller.step）
};

/** 纯校验：把配置里的自相矛盾在启动前就暴露出来 */
export function validateConfig(cfg) {
  const errs = [];
  if (cfg.wheels.length !== 4) errs.push('必须四轮');
  const seen = new Set();
  cfg.wheels.forEach((w, i) => {
    if (w.sign !== 1 && w.sign !== -1) errs.push(`轮 ${w.id} 的 sign 必须是 ±1`);
    if (seen.has(w.port)) errs.push(`端口重复: ${w.port}`);
    if (seen.has(w.slave)) errs.push(`站号重复: ${w.slave}`);
    seen.add(w.port); seen.add(w.slave);
    if (!Number.isInteger(w.slave) || w.slave < 1 || w.slave > 247) errs.push(`轮 ${w.id} 站号非法`);
    if (i !== cfg.index[w.id]) errs.push(`轮序与 index 表不一致: ${w.id}`);
  });
  if (cfg.control.maxCmdRaw > cfg.expect.maxCeilingRaw) {
    errs.push(`maxCmdRaw(${cfg.control.maxCmdRaw}) 超过驱动器上限 Pn-30D(${cfg.expect.maxCeilingRaw})`);
  }
  // 物理闸：额定转速（Fn-1C 实机读回 3000 rpm）折成 raw。软件限幅、允许的驱动器限幅都不许越过它 ——
  // Pn-30D 手册能填到 30000（=6000 rpm），那是"能填"，不是"能跑"。
  const ratedRaw = cfg.nameplate?.ratedSpdRpm * cfg.rawPerRpm;
  if (Number.isFinite(ratedRaw) && ratedRaw > 0) {
    if (cfg.control.maxCmdRaw > ratedRaw) errs.push(`maxCmdRaw(${cfg.control.maxCmdRaw}) 超过额定转速对应 raw(${ratedRaw} = ${cfg.nameplate.ratedSpdRpm} rpm) → 超额定，拒绝`);
    if (cfg.expect.maxCeilingRaw > Math.min(ratedRaw, cfg.pn30dManualMaxRaw ?? ratedRaw)) {
      errs.push(`允许看到的 Pn-30D 上限(${cfg.expect.maxCeilingRaw}) 越过额定对应 raw(${ratedRaw})：限幅最高只应设成额定，多出来的数字没有安全意义只有失控意义`);
    }
  }
  if (!Number.isFinite(cfg.control.quantizeRaw) || cfg.control.quantizeRaw < 1) errs.push('quantizeRaw 必须是 ≥1 的有限数');
  if (!Number.isFinite(cfg.control.maxCmdRaw) || cfg.control.maxCmdRaw <= 0) errs.push(`maxCmdRaw 必须是正有限数（收到 ${cfg.control.maxCmdRaw}）`);
  if (!(cfg.control.loopHz > 0 && cfg.control.loopHz <= 30)) errs.push(`loopHz=${cfg.control.loopHz} 不合理：四口并行实测整机周期 35~45ms，20Hz 已到位`);
  if (!(Number.isFinite(cfg.control.inputTimeoutMs) && cfg.control.inputTimeoutMs > 0)) errs.push('inputTimeoutMs 必须是正有限数 —— 看门狗不允许被配成 0/无穷');
  if (!(Number.isFinite(cfg.control.commProbeMs) && cfg.control.commProbeMs > 0)) errs.push('commProbeMs 必须是正有限数 —— 不探测就发现不了断线');
  if (!(Number.isFinite(cfg.control.commFailLimit) && cfg.control.commFailLimit > 0)) errs.push('commFailLimit 必须是正有限数');
  if (!(Number.isFinite(cfg.control.rampRawPerTick) && cfg.control.rampRawPerTick > 0)) errs.push('rampRawPerTick 必须是正有限数');
  if (!Number.isFinite(cfg.control.minWriteIntervalMs) || cfg.control.minWriteIntervalMs < 0) errs.push('minWriteIntervalMs 不能为负/非有限数');
  if (!(Number.isFinite(cfg.mech.gearRatio) && cfg.mech.gearRatio > 0) || !(cfg.mech.wheelDiameterM > 0) || !(cfg.mech.lxPlusLyM > 0)) errs.push('机械参数必须为正有限数（gearRatio/wheelDiameterM/lxPlusLyM）');
  if (!Number.isFinite(cfg.rawPerRpm) || cfg.rawPerRpm <= 0) errs.push('rawPerRpm 必须为正有限数');
  return errs;
}

export const CONFIG = {
  rawPerRpm: RAW_PER_RPM,
  regs: { speed: SPEED_REG, mode: MODE_REG, ceiling: CEILING_REG, monitor: MONITOR_REG, monitorQty: MONITOR_QTY, mon: MON, nameplate: NAMEPLATE_REG, nameplateQty: NAMEPLATE_QTY },
  expect: EXPECT,
  wheels: WHEELS,
  index: WHEEL_INDEX,
  mech: MECH,
  control: CONTROL,
  nameplate: NAMEPLATE,
  pn30dManualMaxRaw: PN30D_MANUAL_MAX_RAW,
};
