import { test } from 'node:test';
import assert from 'node:assert/strict';
import { CONFIG, DEFAULT_TARGET_MPS, validateConfig } from '../src/config.js';
import { ratedRaw, ratedSpeedMps, ceilingForRated, speedPreset, suggestedStartGear, SLOW_START_MPS, DEFAULT_ACCEL_MPS2 } from '../src/preset.js';
import { speedMpsFromRaw, twistToRaw } from '../src/kinematics.js';
import { makeThrottleState, planWrites, quantize } from '../src/limits.js';

// 铭牌实测（2026-10-09 四台一致）：Fn-1C = 3000 rpm → 15000 raw；Ø0.20 m、30:1 → 1.047 m/s
test('额定转速就是硬天花板：15000 raw / 1.047 m/s，四台读回来是这个数', () => {
  assert.equal(ratedRaw(CONFIG), 15000);
  assert.ok(Math.abs(ratedSpeedMps(CONFIG) - 1.047) < 0.002, `实际 ${ratedSpeedMps(CONFIG)}`);
  assert.equal(ceilingForRated(CONFIG), 15000, 'Pn-30D 建议值 = 额定，不多写一档');
  assert.ok(ceilingForRated(CONFIG) <= CONFIG.pn30dManualMaxRaw, '手册能填 30000，但那是"能填"不是"能跑"');
});

test('按车速给档：1.0 m/s → 14324 raw，量化档与斜坡自动放大', () => {
  const p = speedPreset(CONFIG, 1.0);
  assert.equal(p.control.maxCmdRaw, 14324);
  assert.equal(p.control.quantizeRaw, 145, '满量程 1%，低速小档时不会退化成"每拍都写"');
  assert.equal(p.bench.timeToFullS, 1.25, '1.0 m/s ÷ 0.8 m/s² = 1.25 秒爬满');
  assert.equal(p.bench.writesToFull, 5, '1.25 秒 ÷ 250ms 限频 = 5 次写入');
  assert.equal(p.bench.ratedRaw, 15000);
  assert.equal(p.control.rampRawPerTick, Math.ceil(14324 / 5), `斜坡要摊到 ${p.bench.writesToFull} 次写入上（受限频 250ms 约束，不是按拍数摊）`);
  assert.equal(p.expect.maxCeilingRaw, 15000);
  assert.deepEqual(validateConfig(p), [], '预设出来的配置必须自检通过');
  assert.ok(Math.abs(speedMpsFromRaw(p.control.maxCmdRaw, p) - 1.0) < 1e-4);
});

test('小车速档不许"一次跳到位"，量化档也保留原台架口径', () => {
  const p = speedPreset(CONFIG, 0.02);
  assert.equal(p.control.quantizeRaw, CONFIG.control.quantizeRaw, '1% 满量程比原档小时保留原档');
  assert.ok(p.control.rampRawPerTick >= CONFIG.control.rampRawPerTick, '小档的斜坡只许比原台架更稳，不许更猛');
  assert.equal(p.bench.timeToFullS, 1.0, '爬升时间有下限窗口：0.02 m/s 也不许一帧跳满');
  assert.ok(p.control.maxCmdRaw < 300);
});

test('目标超过额定折算车速 → 直接拒绝，不做"软件侧的超速"（1.3 m/s 就是当初被否掉的那个）', () => {
  assert.throws(() => speedPreset(CONFIG, 1.3), /超过额定折算上限/);
  assert.throws(() => speedPreset(CONFIG, 0), /正有限数/);
  assert.throws(() => speedPreset(CONFIG, NaN), /正有限数/);
  assert.throws(() => speedPreset(CONFIG, 1.0, 0), /加速度/);
  // 机械参数改小（更小减速比）之后，同一个 1.3 m/s 就可能合规 —— 天花板跟着实测值走，不是写死的数字
  const slower = { ...CONFIG, mech: { ...CONFIG.mech, gearRatio: 20 } };
  assert.equal(Math.round(speedPreset(slower, 1.3).control.maxCmdRaw), 12414, '1.3 m/s @20:1 → 2483 rpm 电机侧 = 12414 raw');
  assert.throws(() => speedPreset({ ...CONFIG, mech: { ...CONFIG.mech, gearRatio: 40 } }, 1.3), /超过额定折算上限/);
});

test('加速度越小爬得越缓（--accel 是给人调的，不是让斜坡固定死）', () => {
  const fast = speedPreset(CONFIG, 1.0, DEFAULT_ACCEL_MPS2);
  const slow = speedPreset(CONFIG, 1.0, 0.4);
  assert.equal(slow.bench.writesToFull, 10, '2.5 秒 ÷ 250ms 限频 = 10 次写入');
  assert.ok(slow.control.rampRawPerTick < fast.control.rampRawPerTick);
});

test('超额定的限幅/软顶在 validateConfig 就被拦住（改 config 也改不出超速）', () => {
  const over = structuredClone(CONFIG);
  over.control.maxCmdRaw = 16000; // > 15000 = 额定
  over.expect.maxCeilingRaw = 20000;
  const errs = validateConfig(over);
  assert.ok(errs.some((e) => /超额定/.test(e)), errs.join('；'));
  assert.ok(errs.some((e) => /越过额定对应 raw/.test(e)), errs.join('；'));
});

test('1.0 m/s 档：爬到满速只写几次，稳态零写入（EEPROM 约束在高转速下同样成立）', () => {
  const p = speedPreset(CONFIG, 1.0);
  const st = makeThrottleState(p.wheels.length);
  const target = [1, 1, -1, -1].map((s) => s * p.control.maxCmdRaw);
  let t = 0;
  const perTick = [];
  let lastWriteTick = 0;
  for (let i = 0; i < 200; i++) {
    const before = st.counts.writes;
    planWrites(st, target, t, p.control);
    perTick.push(st.counts.writes - before);
    if (perTick[i]) lastWriteTick = i;
    t += 1000 / p.control.loopHz;
  }
  // 量化越界时向下退一整档：14324 打不上去，落到 14210（≈0.992 m/s）才是线上的满杆值
  assert.deepEqual(st.written, [14210, 14210, -14210, -14210], `实际 ${st.written}`);
  assert.ok(Math.abs(speedMpsFromRaw(14210, p) - 0.992) < 0.002);
  assert.ok(st.counts.writes <= 24, `四轮各写 ${p.bench.writesToFull} 次就该到位，实际总共 ${st.counts.writes} 次`);
  assert.ok(lastWriteTick <= 5 * p.bench.writesToFull, `到位时间 ${lastWriteTick * 50} ms 应当 ≈ ${p.bench.timeToFullS * 1000} ms`);
  assert.equal(perTick.slice(100).reduce((a, b) => a + b, 0), 0, '稳态必须一字节都不发');
});

test('起始幅度档：慢速台架保留 2/4，高速档自动压到 1/4（界面与键盘共用同一个默认值）', () => {
  assert.equal(suggestedStartGear(CONFIG), 2, '60 rpm 架台档 ≈ 0.02 m/s，仍然 2/4 起步，老手感不变');
  assert.equal(suggestedStartGear(speedPreset(CONFIG, 0.3)), 2, `刚好 ${SLOW_START_MPS} m/s 不算快`);
  for (const m of [0.4, 0.5, 1.0]) assert.equal(suggestedStartGear(speedPreset(CONFIG, m)), 1, `${m} m/s 必须从最低档起步`);
  assert.equal(suggestedStartGear(speedPreset(CONFIG, 1.0), 1), 1, '只有一档时不许返回 2 把前端下标越界');
  assert.equal(suggestedStartGear(speedPreset(CONFIG, 1.0), 8), 1);
  assert.throws(() => suggestedStartGear(CONFIG, 0), /gears/);
  assert.throws(() => suggestedStartGear(CONFIG, NaN), /gears/);
});

// 手柄/键盘"裸启动"（不带 --mps）走的就是这一档。2026-10-09 离地四轮实测通过的就是它，
// 所以这几个数不是算出来的期望值，而是当时读回来的实测锚点 —— 改默认档必然要让测试跟着改，
// 目的是逼着人重新做一次安全确认，而不是让默认值被人悄悄挪走。
test('默认档就是实测过的 0.5 m/s：7162/70/1791，四档 0.127~0.498，起始 1/4', () => {
  assert.equal(DEFAULT_TARGET_MPS, 0.5, '默认档变了 = 安全确认要重做，先把这条测试改明白');
  const p = speedPreset(CONFIG, DEFAULT_TARGET_MPS);
  assert.equal(p.control.maxCmdRaw, 7162);
  assert.equal(p.control.quantizeRaw, 70);
  assert.equal(p.control.rampRawPerTick, 1791);
  assert.equal(p.bench.timeToFullS, 1);
  assert.equal(p.bench.writesToFull, 4, '实测：1/4→4/4 每档一次到位，约 1 秒爬满');
  assert.equal(suggestedStartGear(p), 1, '0.5 m/s 已经不属于"手滑也无所谓"，必须从最低档起步');
  assert.deepEqual(validateConfig(p), [], '默认档必须自检通过');
  const gearMps = [1, 2, 3, 4].map((g) => Number(speedMpsFromRaw(quantize(Math.round(p.control.maxCmdRaw * g / 4), p.control.quantizeRaw, p.control.maxCmdRaw), p).toFixed(3)));
  assert.deepEqual(gearMps, [0.127, 0.249, 0.376, 0.498], `实测四档 ${gearMps}`);
  assert.ok(gearMps[3] < ratedSpeedMps(p), '默认档满杆仍要在额定折算上限之下');
  // 4/4 前进那一行的线上值与转向：实测读回来是 7140/-7140，四轮全部向前滚
  const { raw } = twistToRaw({ vx: 1, vy: 0, wz: 0 }, p);
  const wire = raw.map((v) => quantize(v, p.control.quantizeRaw, p.control.maxCmdRaw));
  assert.deepEqual(wire, [7140, 7140, -7140, -7140], `实际 ${wire}`);
  assert.ok(wire.every((v, i) => v * p.wheels[i].sign > 0), '前进档四轮必须都向前滚（符号矩阵是实机标定的，不随档位变）');
});

test('想回到最低风险台架档要显式写 --mps 0：裸值 300 不能被默认档吃掉', () => {
  assert.equal(CONFIG.control.maxCmdRaw, 300, '300 = 2026-10-09 14:58 四轮自检那次的锚点，仍要能一键回到');
  assert.equal(suggestedStartGear(CONFIG), 2);
});
