#!/usr/bin/env node
/**
 * 台架入口：键盘 + 网页手柄，共用同一个控制核。
 *
 *   node src/cli.js --table          只打印四轮指令对照表然后退出（纯离线，最安全）
 *   node src/cli.js --dry-run        只算不发帧，用来看指令表（不碰硬件、不需要 serialport）
 *   node src/cli.js --preflight      只读体检四轮状态后退出（只发读帧，不下发任何指令）
 *   node src/cli.js --motors [--mps 1.0]  只读铭牌探测（Fn-1C 额定转速 + 当前 Pn-30D），提转速前的功课
 *   node src/cli.js                  按默认车速档（DEFAULT_TARGET_MPS = 0.5 m/s）体检并进入控制环（会动！）
 *   node src/cli.js --mps 0.3        换一档车速：raw/量化档/斜坡自动派生（额定转速是硬天花板）
 *   node src/cli.js --mps 0          回到台架点动档（裸限幅 300 raw ≈ 0.02 m/s），慢到几乎不会动
 *   node src/cli.js --skip-preflight 跳过体检（不建议）
 *   node src/cli.js --list-ports      只枚举系统串口并和映射对照（不 open 任何口、不动硬件）
 *
 * 串口名不写死在代码里，由 src/ports.js 解析：Windows 默认 COM12/13/14/15，
 * Linux 默认 /dev/ttyCH343USB0-3。换编号用环境变量，别改源码：
 *   CHASSIS_PORT_FL/_FR/_BL/_BR=…   单只轮子指到某个口
 *   CHASSIS_PORT_FIRST=…            整套平移（值 = 1 号口的名字，如 COM7 或 /dev/ttyUSB0）
 *
 * 改驱动器限幅（Pn-30D）不在本入口里：那是写参数，走 `node src/setceiling.js`，需 --confirm 明确确认。
 *
 * 键盘：W/S 前进后退  A/D 左移右移  Q/E 原地转  1-4 幅度档  空格 急停  R 解除闭锁  X 松指令  Ctrl+C 退出
 * 网页：http://127.0.0.1:8765  手柄（Xbox/PS）用 Gamepad API，20Hz POST /twist
 */

import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { CONFIG, DEFAULT_TARGET_MPS, validateConfig, PORTS, PORT_SOURCE, PORT_WARNINGS } from './config.js';
import { formatPorts } from './ports.js';
import { ChassisController } from './controller.js';
import { RtuBus } from './bus.js';
import { preflight, formatPreflight, readNameplate, formatNameplate } from './preflight.js';
import { fullScaleSpeedMps, describeTwist, twistToRaw, speedMpsFromRaw } from './kinematics.js';
import { quantize } from './limits.js';
import { dispatchKey, twistFromKeys, sweepHeld } from './keyinput.js';
import { speedPreset, ratedSpeedMps, suggestedStartGear, DEFAULT_ACCEL_MPS2 } from './preset.js';

const HERE = dirname(fileURLToPath(import.meta.url));
const argv = process.argv.slice(2);
const has = (f) => argv.includes(f);
const val = (f, d) => { const i = argv.indexOf(f); return i >= 0 && argv[i + 1] ? argv[i + 1] : d; };
const log = (m) => process.stdout.write(`[${new Date().toISOString().slice(11, 19)}] ${m}\n`);

const cfg = structuredClone(CONFIG);
cfg.control.loopHz = Number(val('--hz', cfg.control.loopHz));
// 车速档：只按 m/s 提要求，raw/量化档/斜坡由铭牌额定转速与机械参数派生（见 src/preset.js）。
// 不带 --mps 也走这一层：默认档就是 DEFAULT_TARGET_MPS（当前 0.5 m/s），"裸启动 = 台架蠕行 300 raw"
// 那个遗留行为已经取消了 —— 手柄一推只有 0.02 m/s 不叫开车。要临时回到那个最低风险档就写 --mps 0。
const mpsArg = val('--mps', String(DEFAULT_TARGET_MPS));
const wantMps = Number(mpsArg);
if (!Number.isFinite(wantMps) || wantMps < 0) {
  // 写了 --mps 却没解析成数（打错字、漏了值）时不许悄悄落回别的速度：
  // "我以为在 0.5 档、实际跑了 300 raw"和"我以为在 300、实际跑了 0.5"一样是事故现场。
  let ceiling = '未知（铭牌额定转速没读到 → 只跑 node src/cli.js --motors 看）';
  try { ceiling = `${ratedSpeedMps(CONFIG).toFixed(3)} m/s（Fn-1C ${CONFIG.nameplate.ratedSpdRpm} rpm 折算）`; } catch {}
  console.error(`--mps 必须是非负有限数（收到 "${mpsArg}"）。0 = 台架档 raw ${CONFIG.control.maxCmdRaw}，${DEFAULT_TARGET_MPS} = 当前默认档，硬天花板 ${ceiling}。`);
  process.exit(2);
}
const accelMps2 = Number(val('--accel', DEFAULT_ACCEL_MPS2));
let presetInfo = null;
if (Number.isFinite(wantMps) && wantMps > 0) {
  try {
    const next = speedPreset(cfg, wantMps, accelMps2);
    presetInfo = next.bench;
    Object.assign(cfg, next);
  } catch (e) { console.error(`车速档不成立：${e.message}`); process.exit(2); }
}
cfg.control.maxCmdRaw = Math.min(Number(val('--max', cfg.control.maxCmdRaw)), cfg.expect.maxCeilingRaw);
const cfgErrs = validateConfig(cfg);
if (cfgErrs.length) {
  console.error(`配置自检失败（改的是 src/config.js？）：\n - ${cfgErrs.join('\n - ')}`);
  process.exit(2);
}
/* 串口名是解析出来的，不是写死的 → 每次启动都把它和来源印出来。
 * 出问题时"车不动 / 某一轮无响应"必须能一眼对上"这条线本来该挂谁"，
 * 不印出来就变成猜。 */
for (const w of PORT_WARNINGS) log(`串口解析警告：${w}`);
log(`串口映射（来源 ${PORT_SOURCE}）：${formatPorts(PORTS)}`);
if (process.platform !== 'win32') {
  log('  Linux 侧 /dev/ttyCH343USB* 的编号顺序未经本机验证：先跑 node src/cli.js --list-ports 看节点在不在，再跑 --preflight 看每只轮子是不是在自己的口上应答。');
}
if (presetInfo) {
  const tag = mpsArg === String(DEFAULT_TARGET_MPS) ? `（默认档，没写 --mps 就是这个）` : '';
  log(`车速档 ${wantMps} m/s${tag}（期望加速度 ${accelMps2} m/s²）→ 满杆 ${cfg.control.maxCmdRaw} raw`
    + ` · 量化档 ${cfg.control.quantizeRaw} raw · 斜坡 ${cfg.control.rampRawPerTick} raw/拍`);
  log(`  额定折算物理上限 ${presetInfo.ratedMps.toFixed(3)} m/s = ${presetInfo.ratedRaw} raw；爬满 ${presetInfo.timeToFullS} s（${presetInfo.writesToFull} 次写入 × ${presetInfo.gateS * 1000} ms 限频）`);
  log(`  驱动器限幅允许看到 ≤ ${cfg.expect.maxCeilingRaw}。当前 Pn-30D 若低于满杆值，预检会拒绝启动 → 那一步要单独跑 node src/setceiling.js（写参数，需明确确认）`);
} else {
  log(`车速档 = --mps 0：不派生预设，用 src/config.js 里的裸限幅 ${cfg.control.maxCmdRaw} raw`
    + ` ≈ ${fullScaleSpeedMps(cfg).toFixed(4)} m/s（台架点动档）。量化/斜坡也一并保留裸值 ${cfg.control.quantizeRaw}/${cfg.control.rampRawPerTick}。`);
}
const DRY = has('--dry-run');
if (has('--motors') && DRY) { console.error('--motors 是只读探测，要走真串口，不能配 --dry-run'); process.exit(2); }
const WEB_PORT = Number(val('--port', 8765));

const bus = DRY ? null : new RtuBus(cfg.wheels, { baud: 9600, timeoutMs: 300 });
const ctl = new ChassisController({ cfg, bus, dryRun: DRY, log });

// 干净退出入口：raw 模式下的 Ctrl+C、网页的"退出"按钮、SIGINT 都走这一个（main 里赋值）。
let exitFn = null;
let cliLastLine = '';          // 非终端输出时的状态行去重

/* ---------------- 键盘输入（逻辑在 keyinput.js，本处只做接线） ---------------- */
const held = new Set();
let lastKeyAt = 0;             // 最近一次真正按下方向键的时刻（用于让位于手柄）
let gear = suggestedStartGear(cfg); // 1..4 → 25/50/75/100%；满杆 >0.3 m/s 时从 1/4 起步（见 preset.suggestedStartGear）
const keySeen = {};            // 终端 raw 模式没有 keyup，只能靠"自动重复还在不在"判断松开
const HOLD_GRACE_MS = 650;     // 系统自动重复的起始延迟约 500ms，留 150ms 余量
const keyboardTwist = () => twistFromKeys(held, gear);
function attachKeyboard() {
  if (!process.stdin.isTTY) { log('非交互终端，键盘输入不可用（可用网页手柄）'); return; }
  process.stdin.setRawMode(true); process.stdin.resume(); process.stdin.setEncoding('utf8');
  process.stdin.on('data', (chunk) => {
    const now = Date.now();
    for (const ch of chunk) {
      dispatchKey(ch, {
        exit: (sig) => { exitFn ? exitFn(sig) : process.exit(130); },
        estop: (why) => ctl.estop(why),
        release: () => { ctl.release(); log('闭锁已解除'); },
        zero: () => { ctl.zero(); log('指令已松'); },
        clearKeys: () => held.clear(),
        help: printHelp,
        snapshot: () => console.log('\n' + JSON.stringify(ctl.snapshot(), null, 2)),
        gear: (g) => { gear = g; log(`幅度档 = ${gear}/4 (${gear * 25}% × 上限 ${cfg.control.maxCmdRaw} raw)`); },
        press: (k) => { held.add(k); keySeen[k] = now; lastKeyAt = now; },
      });
    }
  });
}

function printHelp() {
  log('W/S 前进后退 · A/D 左移右移 · Q/E 原地转 · 1-4 幅度档 · 空格 急停 · R 解除闭锁 · X 松指令 · H 本帮助 · Ctrl+C 退出（先四轮归零）');
}

/* ---------------- 指令对照表（离线，不动硬件） ---------------- */
const ACTIONS = [
  ['前进', { vx: 1, vy: 0, wz: 0 }],
  ['后退', { vx: -1, vy: 0, wz: 0 }],
  ['左平移', { vx: 0, vy: 1, wz: 0 }],
  ['右平移', { vx: 0, vy: -1, wz: 0 }],
  ['原地逆时针', { vx: 0, vy: 0, wz: 1 }],
  ['原地顺时针', { vx: 0, vy: 0, wz: -1 }],
  ['前进+逆时针', { vx: 1, vy: 0, wz: 1 }],
  ['后退+左平移', { vx: -1, vy: 1, wz: 0 }],
];

function printTable(cfg) {
  const pad = (s, n) => String(s).padEnd(n, ' ');
  const cell = 18;
  const head = pad('动作', 14) + cfg.wheels.map((w) => pad(`${w.id} ${w.name} ${w.port}/站${w.slave}`, cell)).join('');
  const onWire = (v) => quantize(v, cfg.control.quantizeRaw, cfg.control.maxCmdRaw); // 表上印的必须是真发到 Pn-303 的那个数
  const wireMax = onWire(cfg.control.maxCmdRaw);
  const lines = [
    `满杆目标 = ${cfg.control.maxCmdRaw} raw = ${cfg.control.maxCmdRaw / cfg.rawPerRpm} rpm(电机侧)`
      + ` ≈ ${(cfg.control.maxCmdRaw / cfg.rawPerRpm / cfg.mech.gearRatio).toFixed(1)} rpm(轮子侧)`
      + ` ≈ ${fullScaleSpeedMps(cfg).toFixed(4)} m/s（减速比 ${cfg.mech.gearRatio}:1 为粗测值）`,
    `量化档 ${cfg.control.quantizeRaw} raw → 满杆实际下发 ${wireMax} raw = ${wireMax / cfg.rawPerRpm} rpm(电机侧)`
      + (wireMax !== cfg.control.maxCmdRaw ? ` ← 目标不是整档，越界时向下退一档，表上是这个数` : ''),
    '',
    head,
    '-'.repeat(head.length),
  ];
  for (const [name, tw] of ACTIONS) {
    const { raw } = twistToRaw(tw, cfg);
    const row = raw.map((v, i) => {
      const wire = onWire(v);
      const roll = wire * cfg.wheels[i].sign; // >0 = 该轮向前滚
      return pad(`${wire > 0 ? '+' : ''}${wire}  ${roll > 0 ? '向前滚' : roll < 0 ? '向后滚' : '停'}`, cell);
    });
    lines.push(pad(name, 14) + row.join(''));
  }
  lines.push('');
  lines.push('幅度档（键盘 1-4 / 手柄 RB-LB）：' + [1, 2, 3, 4].map((g) => {
    const wire = onWire(Math.round(cfg.control.maxCmdRaw * g / 4));
    return `${g}/4 → ${wire} raw ≈ ${speedMpsFromRaw(wire, cfg).toFixed(3)} m/s`;
  }).join('  '));
  const sg = suggestedStartGear(cfg);
  if (sg !== 2) lines.push(`起始幅度档 = ${sg}/4：满杆 ${fullScaleSpeedMps(cfg).toFixed(3)} m/s 已经不属于"手滑也无所谓"的速度，所以一进程序先给最低档（原界面固定 2/4，那是 0.02 m/s 台架档的遗留）。`);
  lines.push(cfg.control.maxCmdRaw === 300
    ? '自检锚点：满档时前进那一行应当等于 2026-10-09 14:58 四轮实物自检下发的 +300/+300/-300/-300。'
    : `符号锚点：满档前进那一行的**符号**必须仍是 前+/后−（幅值随 --mps 变，符号是实机标定出来的，不随档位变）。`);
  console.log(lines.join('\n'));
}

/**
 * /state 里那份配置在启动后就定死了，所以一次算好：
 * 请求路径上不现算换算 —— 铭牌/机械参数一旦被改坏，ratedSpeedMps 会抛，
 * 而看板正在跑的时候被打成 500 空白页，比少显示一个数严重得多。
 */
const webCfg = (() => {
  const o = {
    maxRaw: cfg.control.maxCmdRaw,
    fullScaleMps: fullScaleSpeedMps(cfg),
    mech: { gearRatio: cfg.mech.gearRatio, wheelDiameterM: cfg.mech.wheelDiameterM },
    quantizeRaw: cfg.control.quantizeRaw,
    rampRawPerTick: cfg.control.rampRawPerTick,
    startGear: suggestedStartGear(cfg), // 满杆 >0.3 m/s 时界面/键盘都从 1/4 起步
    wheels: cfg.wheels.map((w) => `${w.id} ${w.name} ${w.port}/站${w.slave} sign=${w.sign > 0 ? '+' : '-'}`),
  };
  try { o.ratedMps = presetInfo ? presetInfo.ratedMps : ratedSpeedMps(cfg); } catch { o.ratedMps = null; } // 铭牌 Fn-1C 折到车速：界面上的"物理天花板"
  o.gearMps = [1, 2, 3, 4].map((g) => speedMpsFromRaw(quantize(Math.round(o.maxRaw * g / 4), o.quantizeRaw, o.maxRaw), cfg));
  return o;
})();

/**
 * `--list-ports`：把"解析出来的映射"和"系统实际认到的串口"并排印出来，然后退出。
 * 只读枚举，不 open 任何端口（所以不会抖 DTR/RTS，也不会跟别的进程抢口）。
 * 换机器、换系统、换线之后先跑这个，再跑 --preflight。
 */
async function listPorts() {
  const wanted = new Map(cfg.wheels.map((w) => [w.port, `${w.id} ${w.name} 站${w.slave}`]));
  console.log(`解析出的映射（来源 ${PORT_SOURCE}）：`);
  for (const w of cfg.wheels) console.log(`  ${w.port.padEnd(20)} ${w.id} ${w.name} 站${w.slave}`);
  let list = [];
  try {
    const { SerialPort } = await import('serialport');
    list = await SerialPort.list();
  } catch (e) {
    console.log(`\n枚举失败：${e.message}\n（Windows 要装 CH343/CH9114 驱动；Linux 要装 WCH 的 ch343ser 内核模块，并把用户加进 dialout 组。）`);
    return;
  }
  console.log(`\n系统认到的串口 ${list.length} 个：`);
  if (!list.length) console.log('  （一个都没有 —— 驱动没加载，或线没插）');
  for (const p of list) {
    const who = wanted.get(p.path);
    const meta = [p.serialNumber && `SN:${p.serialNumber}`, p.vendorId && `VID:PID ${p.vendorId}:${p.productId}`, p.ppid && `ppid:${p.ppid}`, p.friendlyName]
      .filter(Boolean).join(' · ');
    console.log(`  ${who ? '→' : ' '} ${String(p.path).padEnd(20)}${who ? ` 我要的 ${who}` : '  （程序没用到）'}${meta ? `\n     ${meta}` : ''}`);
  }
  const present = new Set(list.map((p) => p.path));
  const missing = cfg.wheels.filter((w) => !present.has(w.port));
  if (missing.length) {
    console.log(`\n缺 ${missing.length} 个要用的口：${missing.map((w) => `${w.port}(${w.id})`).join(' ')}`);
    console.log('口在但编号不同 → 用环境变量覆盖，别改源码：单轮 CHASSIS_PORT_FL=/dev/ttyCH343USB2，整串偏移 CHASSIS_PORT_FIRST=COM7。');
    console.log('确认过映射再跑只读体检：node src/cli.js --preflight');
  } else {
    console.log('\n四个口都在。下一步只读体检：node src/cli.js --preflight');
  }
}

/* ---------------- 网页 / 手柄输入 ---------------- */
function startWeb() {
  const send = (res, code, obj, type = 'application/json') => {
    res.writeHead(code, { 'content-type': type, 'access-control-allow-origin': '*' });
    res.end(type.startsWith('application/json') ? JSON.stringify(obj, null, 2) : obj);
  };
  const readBody = (req) => new Promise((res) => { let b = ''; req.on('data', (c) => (b += c)); req.on('end', () => res(b)); });
  const server = createServer(async (req, res) => {
    if (req.method === 'OPTIONS') return send(res, 204, {});
    const url = new URL(req.url, 'http://x');
    try {
      if (url.pathname === '/') return send(res, 200, await readFile(join(HERE, '..', 'public', 'index.html'), 'utf8'), 'text/html; charset=utf-8');
      if (url.pathname === '/state') return send(res, 200, { snapshot: ctl.snapshot(), cfg: webCfg });
      if (req.method === 'POST' && url.pathname === '/twist') {
        const t = JSON.parse((await readBody(req)) || '{}');
        ctl.setTwist({ vx: Number(t.vx) || 0, vy: Number(t.vy) || 0, wz: Number(t.wz) || 0 });
        return send(res, 200, { ok: true, action: describeTwist(ctl.twist) });
      }
      if (req.method === 'POST' && url.pathname === '/stop') { ctl.zero(); return send(res, 200, { ok: true }); }
      if (req.method === 'POST' && url.pathname === '/estop') { ctl.estop('网页急停'); return send(res, 200, { ok: true }); }
      if (req.method === 'POST' && url.pathname === '/release') { ctl.release(); return send(res, 200, { ok: true }); }
      if (req.method === 'POST' && url.pathname === '/quit') {
        if (!exitFn) return send(res, 503, { ok: false, error: '退出入口还没就绪' });
        send(res, 200, { ok: true, note: '正在归零后退出' });
        exitFn('网页退出');
        return;
      }
      return send(res, 404, { error: 'not found' });
    } catch (e) { return send(res, 500, { error: String(e.message || e) }); }
  });
  server.listen(WEB_PORT, '127.0.0.1', () => log(`网页/手柄入口 http://127.0.0.1:${WEB_PORT}`));
  return server;
}

/* ---------------- 主流程 ---------------- */
async function main() {
  if (has('--list-ports')) { await listPorts(); process.exit(0); } // 纯枚举，不碰硬件
  printHelp();
  printTable(cfg); // 先核对指令表再进控制环：这一步不需要任何硬件动作
  if (has('--table')) process.exit(0);
  if (!DRY) {
    await bus.open();
    if (has('--motors')) {
      // 铭牌探测 = 只读，读完就退。放在预检**之前**：预检会因为 Pn-30D 偏高等原因拒绝放行，
      // 但"读回额定转速"这件事本身没有风险，被闸住反而看不到我要的那个数。
      // 目标车速与主流程同一个来源：探测时判的那一档，必须就是待会儿控制环要跑的那一档，
      // 否则"探测通过"和"启动被预检拒了"会同时发生。--mps 0 时不给目标 = 纯读铭牌。
      const target = wantMps;
      const np = await readNameplate(bus, cfg, Number.isFinite(target) && target > 0 ? target : null);
      console.log(formatNameplate(np));
      await bus.close();
      const needsWrite = !!np.need && np.rows.some((r) => r.info.overCeiling);
      console.log(`\n只读铭牌探测完成，本次未下发任何指令${target > 0 ? `；目标车速 ${target} m/s 只是换算，限幅未改、车未动` : ''}。`);
      // 退出码三种，别只看个数字就当成绿灯：0 读到了且目标在额定与当前限幅内，2 超额定/读不到（不许提），3 目标可行但要先写 Pn-30D（要单独确认）
      console.log(`退出码 = ${!np.ok ? '2（超额定或读不到额定转速 → 不许提速）' : needsWrite ? '3（可行，但要先改 Pn-30D：这是写参数，需要单独确认）' : '0（只读探测通过）'}`);
      process.exit(!np.ok ? 2 : needsWrite ? 3 : 0);
    }
    const p = await preflight(bus, cfg);
    console.log(formatPreflight(p));
    if (has('--preflight')) {
      // 体检模式 = 只读，读完就退，绝不进控制环、绝不下发一帧
      await bus.close();
      console.log(p.ok ? '\n只读体检完成：四轮状态允许进入控制环。本次未下发任何指令。' : '\n只读体检完成：存在问题，本次未下发任何指令。');
      process.exit(p.ok ? 0 : 2);
    }
    if (!p.ok && !has('--skip-preflight')) {
      await bus.close();
      console.log('\n预检不通过，拒绝进入控制环。先按上面的问题逐条处理。');
      process.exit(2);
    }
  } else {
    console.log('干跑模式：不发任何帧，只算指令表。');
  }

  attachKeyboard();
  const server = startWeb();
  await ctl.start();

  // 终端没有 keyup：靠"自动重复是否还在"判断松手。
  // 键盘只在"自己有用输入"时才写控制核，避免每拍把网页手柄的输入顶掉。
  const keepAlive = setInterval(() => {
    sweepHeld(held, keySeen, Date.now(), HOLD_GRACE_MS);
    if (held.size) ctl.setTwist(keyboardTwist());
    else if (Date.now() - lastKeyAt < 1000) ctl.zero();
  }, 80);

  let stopping = false;
  const shutdown = async (sig) => {
    if (stopping) return; stopping = true;
    log(`收到 ${sig}，先把四轮归零再退出`);
    clearInterval(keepAlive);
    server.close();
    // 归零这条路本身也可能卡住（串口挂着、驱动器不回帧）→ 给它 3 秒，超时就明着让人去切总开关，
    // 不能"退出"变成"永远退不出去"，那等于最后一次指令悬在半速。
    const hardExit = setTimeout(() => {
      log(`归零超过 3 秒没完成 —— 立刻用物理总开关，别等这个进程`);
      process.exit(1);
    }, 3000);
    await ctl.shutdown();
    if (!DRY) { try { await bus.close(); } catch {} }
    clearTimeout(hardExit);
    console.log(`\n写入统计：${JSON.stringify(ctl.throttle.counts)}`);
    process.exit(0);
  };
  exitFn = shutdown;
  process.on('SIGINT', () => shutdown('SIGINT'));
  process.on('SIGTERM', () => shutdown('SIGTERM'));
  process.on('uncaughtException', (e) => { log(`未捕获异常：${e.message}`); shutdown('uncaught'); });

  // 状态行
  setInterval(() => {
    const s = ctl.snapshot();
    const row = s.wheels.map((w) => `${w.id}${w.onWire > 0 ? '+' : ''}${w.onWire}`).join(' ');
    const line = `${s.state.padEnd(6)} ${s.action.padEnd(12)} 档${gear}/4 写入${s.eeprom.writes}次  ${row}`;
    if (process.stdout.isTTY) { process.stdout.write(`\r${line}   `); return; }
    // 重定向到文件时 \r 不会清行，几百拍会堆成一坨没法读 → 非终端就只在"变化时"打一行
    if (line !== cliLastLine) { cliLastLine = line; log(line); }
  }, 250);
}

main().catch(async (e) => { console.error(e); await ctl.shutdown(); process.exit(1); });
