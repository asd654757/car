import { test } from 'node:test';
import assert from 'node:assert/strict';
import { CONFIG } from '../src/config.js';
import { twistToRaw, twistToWheelMps, wheelToTwist, scaleToCap, fullScaleSpeedMps, applySigns, rawForSpeedMps, speedMpsFromRaw } from '../src/kinematics.js';

const cap = CONFIG.control.maxCmdRaw;
const ids = CONFIG.wheels.map((w) => w.id);

test('轮序与标定表一致：FL FR BL BR', () => {
  assert.deepEqual(ids, ['FL', 'FR', 'BL', 'BR']);
  // 站号表是实机标定量；端口名随平台/环境变量变，不写死在这里（见 test/ports.test.js）
  assert.deepEqual(CONFIG.wheels.map((w) => w.slave), [1, 4, 2, 3]);
  assert.equal(new Set(CONFIG.wheels.map((w) => w.port)).size, 4);
});

test('符号矩阵 = 实机标定结果（前 + 后 −）', () => {
  assert.deepEqual(Object.fromEntries(CONFIG.wheels.map((w) => [w.id, w.sign])), { FL: 1, FR: 1, BL: -1, BR: -1 });
});

test('纯前进：四轮向前滚同号，落到 Pn-303 是前正后负', () => {
  const mps = twistToWheelMps({ vx: 1, vy: 0, wz: 0 }, CONFIG.mech);
  assert.deepEqual(mps, [1, 1, 1, 1], '直行时四个轮都必须向前滚');
  const { raw } = twistToRaw({ vx: 1, vy: 0, wz: 0 }, CONFIG);
  assert.deepEqual(raw, [cap, cap, -cap, -cap], '与 2026-10-09 四轮自检下发的 +300/+300/-300/-300 完全一致');
});

test('纯后退：整体反号', () => {
  const { raw } = twistToRaw({ vx: -1, vy: 0, wz: 0 }, CONFIG);
  assert.deepEqual(raw, [-cap, -cap, cap, cap]);
});

test('纯左移：对角同号（标准麦轮）', () => {
  const mps = twistToWheelMps({ vx: 0, vy: 1, wz: 0 }, CONFIG.mech);
  assert.deepEqual(mps, [-1, 1, 1, -1]);
});

test('原地逆时针：左右反号', () => {
  const mps = twistToWheelMps({ vx: 0, vy: 0, wz: 1 }, CONFIG.mech);
  assert.deepEqual(mps.map((v) => Math.sign(v)), [-1, 1, -1, 1]);
});

test('任意输入都不越过 maxCmdRaw，且等比缩放不改变方向', () => {
  for (const t of [{ vx: 1, vy: 1, wz: 1 }, { vx: 1, vy: -1, wz: 0.5 }, { vx: 0.2, vy: 0.9, wz: -1 }]) {
    const { raw, scaled } = twistToRaw(t, CONFIG);
    raw.forEach((v) => assert.ok(Math.abs(v) <= cap, `${JSON.stringify(t)} 越限 ${v}`));
    if (t.vx || t.vy || t.wz) assert.ok(scaled, `${JSON.stringify(t)} 应该触发缩放`);
  }
});

test('零输入 → 四轮零', () => {
  assert.deepEqual(twistToRaw({ vx: 0, vy: 0, wz: 0 }, CONFIG).raw, [0, 0, 0, 0]);
});

test('输入越界被夹到 ±1，NaN 当 0', () => {
  const a = twistToRaw({ vx: 9, vy: 0, wz: 0 }, CONFIG).raw;
  const b = twistToRaw({ vx: 1, vy: 0, wz: 0 }, CONFIG).raw;
  assert.deepEqual(a, b);
  assert.deepEqual(twistToRaw({ vx: NaN, vy: undefined, wz: 0 }, CONFIG).raw, [0, 0, 0, 0]);
});

test('正逆运动学往返一致', () => {
  const t = { vx: 0.3, vy: -0.1, wz: 0.05 };
  const back = wheelToTwist(twistToWheelMps(t, CONFIG.mech), CONFIG.mech);
  assert.ok(Math.abs(back.vx - t.vx) < 1e-9);
  assert.ok(Math.abs(back.vy - t.vy) < 1e-9);
  assert.ok(Math.abs(back.wz - t.wz) < 1e-9);
});

test('applySigns 只改符号不改幅值', () => {
  const raw = applySigns([10, 10, 10, 10], CONFIG.wheels.map((w) => w.sign));
  assert.deepEqual(raw, [10, 10, -10, -10]);
});

test('scaleToCap 峰值不超时原样返回', () => {
  const r = scaleToCap([1, -2, 3, -4], 10);
  assert.equal(r.scaled, false);
  assert.deepEqual(r.values, [1, -2, 3, -4]);
});

test('满杆速度是电机侧限幅折算出来的（Ø0.20 m、30:1、300 raw ≈ 2.1 cm/s）', () => {
  const v = fullScaleSpeedMps(CONFIG);
  // 300 raw = 60 rpm(电机侧) ÷ 30 = 2 rpm(轮子侧) × 周长 π×0.20 ÷ 60 ≈ 0.0209 m/s
  const byHand = (CONFIG.control.maxCmdRaw / CONFIG.rawPerRpm / CONFIG.mech.gearRatio) * (Math.PI * CONFIG.mech.wheelDiameterM) / 60;
  assert.ok(Math.abs(v - byHand) < 1e-9, `换算必须等于手算 ${byHand}，实际 ${v}`);
  assert.ok(v > 0.02 && v < 0.022, `当前台架档在 2 cm/s 量级，实际 ${v} —— 改了直径/减速比/限幅要同步这条`);
});

test('m/s ↔ raw 往返一致（--mps 反推限幅、界面显示车速都走这一对函数）', () => {
  for (const mps of [0.5, 1.0, 1.3, 2.0]) {
    const raw = rawForSpeedMps(mps, CONFIG);
    assert.ok(Math.abs(speedMpsFromRaw(raw, CONFIG) - mps) < 1e-9, `${mps} m/s 往返应当还原`);
  }
  // 1.0 m/s @30:1/Ø0.20：轮子 95.5 rpm → 电机 2865 rpm → 14324 raw，量级要对得上人工核算
  assert.equal(Math.round(rawForSpeedMps(1.0, CONFIG)), 14324);
});
