import { test } from 'node:test';
import assert from 'node:assert/strict';
import { resolvePorts, formatPorts, WHEEL_CHANNEL } from '../src/ports.js';

/**
 * 串口名解析器的测试。这一层是"移植到 Ubuntu / 换适配器 / 临时改线"时唯一要动的地方，
 * 所以它的优先级、偏移、撞口检测都得有证据，不能靠人肉看一眼默认值。
 * 全部是纯函数：注入 env 和 platform，不碰 process、不碰串口。
 */

const portsOf = (env = {}, platform = 'win32') => resolvePorts(env, platform).ports;

test('Windows 默认沿用实机标定：COM12/13/14/15 = 1/2/3/4 号口', () => {
  const p = portsOf({}, 'win32');
  assert.deepEqual(p, { FL: 'COM12', BL: 'COM13', BR: 'COM14', FR: 'COM15' });
});

test('Linux 默认走 WCH ch343 节点名，通道号从 0 起算', () => {
  const p = portsOf({}, 'linux');
  assert.deepEqual(p, { FL: '/dev/ttyCH343USB0', BL: '/dev/ttyCH343USB1', BR: '/dev/ttyCH343USB2', FR: '/dev/ttyCH343USB3' });
  for (const id of Object.keys(WHEEL_CHANNEL)) {
    assert.equal(Number(p[id].match(/(\d+)$/)[1]), WHEEL_CHANNEL[id] - 1, `${id} 要落在 ${WHEEL_CHANNEL[id]} 号口对应的节点`);
  }
});

test('CHASSIS_PORT_FIRST 整串平移：轮序、站号都不动，只有口名跟着挪', () => {
  assert.deepEqual(portsOf({ CHASSIS_PORT_FIRST: 'COM7' }, 'win32'),
    { FL: 'COM7', BL: 'COM8', BR: 'COM9', FR: 'COM10' });
  assert.deepEqual(portsOf({ CHASSIS_PORT_FIRST: '/dev/ttyUSB0' }, 'linux'),
    { FL: '/dev/ttyUSB0', BL: '/dev/ttyUSB1', BR: '/dev/ttyUSB2', FR: '/dev/ttyUSB3' });
});

test('补零写法要保住位数（COM08 平移不能塌成 COM8）', () => {
  const p = portsOf({ CHASSIS_PORT_FIRST: 'COM08' }, 'win32');
  assert.deepEqual(p, { FL: 'COM08', BL: 'COM09', BR: 'COM10', FR: 'COM11' });
});

test('单轮覆盖优先级最高，其余轮保持默认', () => {
  const p = portsOf({ CHASSIS_PORT_FR: '/dev/ttyCH343USB1', CHASSIS_PORT_FIRST: '/dev/ttyUSB0' }, 'linux');
  assert.equal(p.FR, '/dev/ttyCH343USB1', '显式写的那一只压过整串偏移');
  assert.equal(p.FL, '/dev/ttyUSB0', '没写的还是跟着 FIRST 走');
});

test('FIRST 解析不出编号 → 忽略并明说，不许悄悄当成 0 号口', () => {
  const r = resolvePorts({ CHASSIS_PORT_FIRST: 'tty-nope' }, 'win32');
  assert.deepEqual(r.ports, { FL: 'COM12', BL: 'COM13', BR: 'COM14', FR: 'COM15' });
  assert.match(r.warnings.join(';'), /解析不出编号/);
});

test('两只轮子撞同一个口 → 当场警告（一口一从站，撞了就说明映射被改坏了）', () => {
  const r = resolvePorts({ CHASSIS_PORT_FR: 'COM12' }, 'win32');
  assert.equal(r.ports.FR, 'COM12');
  assert.match(r.warnings.join(';'), /FR 和 FL 都指向 COM12/);
});

test('空白值不算覆盖；来源要能打印出来给人核对', () => {
  const r = resolvePorts({ CHASSIS_PORT_FL: '   ' }, 'win32');
  assert.equal(r.ports.FL, 'COM12');
  assert.equal(formatPorts(r.ports), 'FL=COM12  FR=COM15  BL=COM13  BR=COM14');
  assert.match(r.source, /平台默认/);
});
