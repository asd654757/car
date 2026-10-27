/**
 * 串口路径解析 —— 让同一份代码在 Windows 和 Ubuntu 上都不用改源码。
 *
 * 为什么单独一个文件：src/config.js 是"纯数据、可整体翻译成 C"的那一层，
 * 板级 C 有自己的引脚表，电脑侧的"口叫什名"属于部署配置，不该混进标定常量里。
 * 这里不用任何 Node 内置模块（只读全局 process），保持和 config.js 一样的可移植性。
 *
 * 三级覆盖，从高到低：
 *   1) 单轮覆盖   CHASSIS_PORT_FL / _FR / _BL / _BR
 *   2) 整串偏移   CHASSIS_PORT_FIRST = 1 号口的名字（Windows 例：COM7；Linux 例：/dev/ttyUSB0）
 *   3) 平台默认   win32 → COM12/13/14/15（2026-10 实机）；其他 → /dev/ttyCH343USB0..3
 *
 * ⚠ 覆盖顺序按**物理通道**（CH9114 的 1~4 号口）算，不是按轮子数组下标：
 *   实机标定是「n 号口 = 站 n」，FL/BL/BR/FR 分别是 1/2/3/4 号口。
 *   换线、换适配器、Linux 上枚举顺序变了，都只需要动这一处，别去改 config.js 的站号表。
 */

/** 实机标定的物理通道号：CH9114 的 n 号口挂着站号 n 的驱动器 */
export const WHEEL_CHANNEL = { FL: 1, BL: 2, BR: 3, FR: 4 };

/** 轮序固定按标定表 FL FR BL BR，别动 —— 运动学数组下标依赖它 */
export const WHEEL_ORDER = ['FL', 'FR', 'BL', 'BR'];

/** Windows 侧 2026-10 实机读数：COM12/13/14/15 = 1/2/3/4 号口 */
const WINDOWS_FIRST = 'COM12';
/**
 * Linux 侧 WCH 官方 ch343ser 驱动的节点名，一块 CH9114 的四个通道 = USB0..USB3。
 * 这条映射**还没在这台机器上验过**：枚举顺序、插拔后的编号都可能和想象不同，
 * 所以它只是"装完驱动后最可能对的默认值"。错了不会出危险 ——
 * 一口一从站的接线下，跑错口就是那一轮读不到应答，只读体检会明着报"无响应"，
 * 不会把指令送进别的轮子。验法：`node src/cli.js --list-ports` 再 `--preflight`。
 */
const LINUX_BASE = '/dev/ttyCH343USB';

const channelOf = (wheelId) => WHEEL_CHANNEL[wheelId];

/** 从 1 号口名字推出各轮路径：COM12 → COM13…；/dev/ttyUSB0 → …ttyUSB1（按通道-1 的偏移） */
function fromFirst(first) {
  const m = /^(.*?)(\d+)$/.exec(String(first).trim());
  if (!m) return null;
  const [, prefix, digits] = m;
  const base = Number(digits);
  const pad = digits.length > 1 ? digits.length : 0; /* COM12 这种补零写法要保住位数 */
  const ports = {};
  for (const id of WHEEL_ORDER) {
    const n = base + channelOf(id) - 1;
    ports[id] = prefix + (pad ? String(n).padStart(pad, '0') : String(n));
  }
  return ports;
}

function platformDefault(platform) {
  if (platform === 'win32') return fromFirst(WINDOWS_FIRST);
  const ports = {};
  for (const id of WHEEL_ORDER) ports[id] = `${LINUX_BASE}${channelOf(id) - 1}`;
  return ports;
}

/**
 * @param {object} env      默认 process.env
 * @param {string} platform 默认 process.platform
 * @returns {{ports:Object, source:string, warnings:string[]}}
 */
export function resolvePorts(env = process.env, platform = process.platform) {
  const warnings = [];
  let ports;
  let source;

  if (platform === 'win32') ports = fromFirst(WINDOWS_FIRST);
  else ports = platformDefault(platform);
  source = `平台默认(${platform})`;

  if (env.CHASSIS_PORT_FIRST) {
    const shifted = fromFirst(env.CHASSIS_PORT_FIRST);
    if (shifted) { ports = shifted; source = 'CHASSIS_PORT_FIRST'; }
    else warnings.push(`CHASSIS_PORT_FIRST="${env.CHASSIS_PORT_FIRST}" 解析不出编号，已忽略`);
  }

  const perWheel = [];
  for (const id of WHEEL_ORDER) {
    const k = `CHASSIS_PORT_${id}`;
    const v = env[k];
    if (v && String(v).trim()) { ports[id] = String(v).trim(); perWheel.push(id); }
  }
  if (perWheel.length) source += perWheel.length === 4 ? '，四轮全部由环境变量指定' : `，其中 ${perWheel.join('/')} 由环境变量指定`;

  // 一口一从站的硬约束：撞口 = 两个轮子抢同一条总线，config.validateConfig 会拦，这里提前说清是谁跟谁撞
  const used = new Map();
  for (const id of WHEEL_ORDER) {
    if (used.has(ports[id])) warnings.push(`${id} 和 ${used.get(ports[id])} 都指向 ${ports[id]}：CH9114 一口一从站，必须改回一个`);
    else used.set(ports[id], id);
  }

  return { ports, source, warnings };
}

/** 给人看的一行：FL=COM12  FR=COM15  BL=COM13  BR=COM14 */
export function formatPorts(ports) {
  return WHEEL_ORDER.map((id) => `${id}=${ports[id]}`).join('  ');
}
