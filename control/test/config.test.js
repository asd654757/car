import { test } from 'node:test';
import assert from 'node:assert/strict';
import { CONFIG, validateConfig, WHEELS } from '../src/config.js';
import { WHEEL_CHANNEL } from '../src/ports.js';

const clone = () => structuredClone(CONFIG);

test('出厂配置自身要过校验', () => {
  assert.deepEqual(validateConfig(CONFIG), []);
});

test('符号矩阵锁死在实机标定结果：前轮 +1 / 后轮 -1', () => {
  assert.deepEqual(WHEELS.map((w) => `${w.id}${w.sign > 0 ? '+' : '-'}`), ['FL+', 'FR+', 'BL-', 'BR-']);
  assert.deepEqual(WHEELS.map((w) => `${w.name}/站${w.slave}`), ['左前/站1', '右前/站4', '左后/站2', '右后/站3']);
});

/**
 * 串口名从 src/ports.js 解析（Windows 默认 COM12-15，Ubuntu 默认 /dev/ttyCH343USB0-3），
 * 所以这里不能把 "COM12" 这种字面量写进断言 —— 那会让测试在别的系统上假失败。
 * 真正要锁的是实机标定出来的那条物理关系：**n 号口挂站号 n**，以及四轮不能撞口。
 */
test('一口一从站的标定结论：物理通道号 == 站号', () => {
  for (const w of WHEELS) {
    assert.equal(WHEEL_CHANNEL[w.id], w.slave, `${w.id} 的通道号必须等于它的站号（改了接线要重测，不是改测试）`);
  }
  assert.equal(new Set(WHEELS.map((w) => w.port)).size, 4, '四个轮子必须落在四个不同的口上');
});

test('sign 被写成 0 或改轮序 → 立刻报错，不许静默降级', () => {
  const c = clone(); c.wheels[1].sign = 0;
  assert.match(validateConfig(c).join(';'), /sign 必须是 ±1/);
  const d = clone(); d.wheels.reverse();
  assert.ok(validateConfig(d).length, '轮序和 index 表必须一致');
});

test('端口/站号重复 → 报错（一口一从站是 CH9114 的硬约束）', () => {
  const c = clone(); c.wheels[1].port = c.wheels[0].port;
  assert.match(validateConfig(c).join(';'), /端口重复/);
  const d = clone(); d.wheels[1].slave = d.wheels[0].slave;
  assert.match(validateConfig(d).join(';'), /站号重复/);
});

test('想把限幅抬过 Pn-30D → 报错（提顶要改驱动器，得重新做安全确认）', () => {
  // 2026-10-09：四台 Pn-30D 已按确认写成 15000（额定对应值），所以"过不过"的界线跟着走，
  // 而不是留在台架那 300。301 现在是合法值，15001 才是越界。
  const ok = clone(); ok.control.maxCmdRaw = 301;
  assert.equal(validateConfig(ok).length, 0, '驱动器限幅已是 15000，脚本层 301 不算越界');
  const c = clone(); c.control.maxCmdRaw = CONFIG.expect.maxCeilingRaw + 1;
  assert.match(validateConfig(c).join(';'), /超过驱动器上限/);
  const d = clone(); d.control.maxCmdRaw = 30000;
  assert.match(validateConfig(d).join(';'), /超过驱动器上限/, '直接把满量程设成手册上限更是要拦的');
});

test('命令行传进来的是垃圾（NaN / 0 / 负数）也要拦住', () => {
  for (const bad of [NaN, undefined, 0, -300, 'x']) {
    const c = clone(); c.control.maxCmdRaw = bad;
    assert.ok(validateConfig(c).length, `maxCmdRaw=${bad} 居然过了校验`);
  }
  const h = clone(); h.control.loopHz = NaN;
  assert.ok(validateConfig(h).length, '--hz 传垃圾必须拒绝，否则 setInterval 周期变 NaN');
});

test('看门狗与心跳不允许被配成"永不生效"', () => {
  const c = clone(); c.control.inputTimeoutMs = 0;
  assert.match(validateConfig(c).join(';'), /inputTimeoutMs/);
  const d = clone(); d.control.commProbeMs = Infinity;
  assert.match(validateConfig(d).join(';'), /commProbeMs/);
});

test('单位换算锚点：300 raw = 60 rpm 电机侧，且这是**电机**转速不是轮子转速', () => {
  assert.equal(CONFIG.rawPerRpm, 5, 'Pn-3xx 单位 0.2 转/分');
  assert.equal(CONFIG.control.maxCmdRaw / CONFIG.rawPerRpm, 60);
  assert.ok(CONFIG.mech.gearRatio >= 28 && CONFIG.mech.gearRatio <= 32, '减速比粗测 30:1，±10% 之内');
  assert.equal(CONFIG.expect.mustIdleReg, CONFIG.regs.speed, '预检要盯的就是运行期会写的那个寄存器');
});
