// 把 test_chassis.c 里硬编码的期望值逐条与已实机验证的 JS 核对账。
// 这查的是"我有没有抄错数"，不是"C 代码对不对"（后者需要能跑的 C 编译器）。
import { CONFIG } from '../src/config.js';
import { twistToRaw, fullScaleSpeedMps, rawForSpeedMps, speedMpsFromRaw, twistToWheelMps, wheelToTwist } from '../src/kinematics.js';
import { makeThrottleState, planWrites, quantize, rampToward } from '../src/limits.js';
import { speedPreset, ratedRaw, ratedSpeedMps, ceilingForRated, suggestedStartGear, DEFAULT_ACCEL_MPS2 } from '../src/preset.js';
import { writeRequest, readRequest } from '../src/modbus.js';

let bad = 0, n = 0;
const eq = (name, got, want, tol = 0) => {
  n++;
  const ok = typeof want === 'number' ? Math.abs(got - want) <= tol : JSON.stringify(got) === JSON.stringify(want);
  if (!ok) { bad++; console.log('MISMATCH', name, 'C 写的是', JSON.stringify(want), 'JS 算出来是', JSON.stringify(got)); }
};

const p05 = speedPreset(CONFIG, 0.5, DEFAULT_ACCEL_MPS2);
const p10 = speedPreset(CONFIG, 1.0, DEFAULT_ACCEL_MPS2);
const p03 = speedPreset(CONFIG, 0.3, DEFAULT_ACCEL_MPS2);
const p02 = speedPreset(CONFIG, 0.02, DEFAULT_ACCEL_MPS2);
const T = (t, c) => twistToRaw(t, c).raw;

eq('kin fwd bench', T({ vx: 1, vy: 0, wz: 0 }, CONFIG), [300, 300, -300, -300]);
eq('kin strafe 0.5', T({ vx: 0, vy: 1, wz: 0 }, p05), [-7162, 7162, -7162, 7162]);
eq('kin spin 0.5', T({ vx: 0, vy: 0, wz: 1 }, p05), [-7162, 7162, 7162, -7162]);
eq('kin back 0.5', T({ vx: -1, vy: 0, wz: 0 }, p05), [-7162, -7162, 7162, 7162]);
eq('kin zero', T({ vx: 0, vy: 0, wz: 0 }, p05), [0, 0, 0, 0]);
eq('kin vx=9 clamp == vx=1', T({ vx: 9, vy: 0, wz: 0 }, p05), T({ vx: 1, vy: 0, wz: 0 }, p05));
eq('kin NaN', T({ vx: NaN, vy: undefined, wz: 0 }, p05), [0, 0, 0, 0]);
eq('kin fullScale', fullScaleSpeedMps(CONFIG), 0.020943951, 1e-8);
eq('kin rawFor1mps', Math.round(rawForSpeedMps(1.0, CONFIG)), 14324);
{
  const t = { vx: 0.3, vy: -0.1, wz: 0.05 };
  const b = wheelToTwist(twistToWheelMps(t, CONFIG.mech), CONFIG.mech);
  eq('kin roundtrip vx', b.vx, 0.3, 1e-9); eq('kin roundtrip vy', b.vy, -0.1, 1e-9); eq('kin roundtrip wz', b.wz, 0.05, 1e-9);
}

eq('lim q103', quantize(103, 10), 100);
eq('lim q106', quantize(106, 10), 110);
eq('lim q-104', quantize(-104, 10), -100);
eq('lim q4', quantize(4, 10), 0);
eq('lim q75 nocap', quantize(75, 10), 80);
eq('lim q75 cap75', quantize(75, 10, 75), 70);
eq('lim q-75 cap75', quantize(-75, 10, 75), -70);
eq('lim q74 cap75', quantize(74, 10, 75), 70);
eq('lim q106 cap100', quantize(106, 10, 100), 100);
eq('lim q103 cap100', quantize(103, 10, 100), 100);
eq('lim q20 cap5', quantize(20, 10, 5), 5);
eq('lim ramp up', rampToward(0, 300, 40), 40);
eq('lim ramp down', rampToward(0, -300, 40), -40);
eq('lim ramp last', rampToward(280, 300, 40), 300);
{
  const C = CONFIG.control, st = makeThrottleState(4), w = [];
  let t = 0;
  for (let i = 0; i < 2000; i++) { const b = st.counts.writes; planWrites(st, [300, 300, -300, -300], t, C); w.push(st.counts.writes - b); t += 50; }
  eq('lim 32 writes in 100s', st.counts.writes, 32);
  eq('lim steady silent', w.slice(200).reduce((a, b) => a + b, 0), 0);
  eq('lim settle', st.written, [300, 300, -300, -300]);
}
{
  const C = CONFIG.control, st = makeThrottleState(4);
  planWrites(st, [100, 100, -100, -100], 1000, C);
  const soon = planWrites(st, [200, 200, -200, -200], 1000 + C.minWriteIntervalMs - 50, C);
  eq('lim deferred count', soon.deferred, [0, 1, 2, 3]);
  eq('lim deferred nowrites', soon.writes.length, 0);
  const later = planWrites(st, [200, 200, -200, -200], 1000 + C.minWriteIntervalMs, C);
  eq('lim later writes', later.writes.length, 4);
  eq('lim counts deferred', st.counts.deferred, 4);
  eq('lim counts writes', st.counts.writes, 8);
}
{   // cap 75 收敛 70 + 贴档零写入
  const C = { ...CONFIG.control, maxCmdRaw: 75 }, st = makeThrottleState(4);
  let t = 0;
  for (let i = 0; i < 100; i++) { planWrites(st, [75, 75, -75, -75], t, C); t += 50; }
  eq('lim cap75 settle', st.written, [70, 70, -70, -70]);
  const b = st.counts.writes;
  planWrites(st, [75, 75, -75, -75], t, C);
  eq('lim cap75 steady', st.counts.writes, b);
}
{   // 归零特权：从满速一步到 0
  const C = CONFIG.control, st = makeThrottleState(4);
  let t = 0;
  for (let i = 0; i < 500; i++) { planWrites(st, [300, 300, -300, -300], t, C); t += 50; }
  const r = planWrites(st, [0, 0, 0, 0], st.lastWriteAt[0] + 1, C);
  eq('lim stop nowrites count', r.writes.length, 4);
  eq('lim stop values', r.writes.map((x) => x.value), [0, 0, 0, 0]);
  eq('lim stop deferred', r.deferred, []);
}
{   // 开机第一帧
  const st = makeThrottleState(4);
  eq('lim first frame', planWrites(st, [50, 50, -50, -50], 0, CONFIG.control).writes.length, 4);
}

eq('pre ratedRaw', ratedRaw(CONFIG), 15000);
eq('pre ratedMps', ratedSpeedMps(CONFIG), 1.047198, 0.002);
eq('pre ceiling', ceilingForRated(CONFIG), 15000);
eq('pre 0.5 maxCmdRaw', p05.control.maxCmdRaw, 7162);
eq('pre 0.5 quantize', p05.control.quantizeRaw, 70);
eq('pre 0.5 ramp', p05.control.rampRawPerTick, 1791);
eq('pre 0.5 timeToFull', p05.bench.timeToFullS, 1.0, 1e-9);
eq('pre 0.5 writesToFull', p05.bench.writesToFull, 4);
eq('pre 0.5 ratedRaw', p05.bench.ratedRaw, 15000);
eq('pre 0.5 startGear', suggestedStartGear(p05), 1);
eq('pre 1.0 maxCmdRaw', p10.control.maxCmdRaw, 14324);
eq('pre 1.0 quantize', p10.control.quantizeRaw, 145);
eq('pre 1.0 ramp', p10.control.rampRawPerTick, 2865);
eq('pre 1.0 timeToFull', p10.bench.timeToFullS, 1.25, 1e-9);
eq('pre 1.0 writesToFull', p10.bench.writesToFull, 5);
eq('pre 1.0@0.4accel writes', speedPreset(CONFIG, 1.0, 0.4).bench.writesToFull, 10);
eq('pre 0.02 quantize', p02.control.quantizeRaw, CONFIG.control.quantizeRaw);
eq('pre 0.02 timeToFull', p02.bench.timeToFullS, 1.0, 1e-9);
eq('pre 0.02 maxCmdRaw<300', p02.control.maxCmdRaw < 300, true);
eq('pre 0.3 startGear', suggestedStartGear(p03), 2);
eq('pre bench startGear', suggestedStartGear(CONFIG), 2);
eq('pre bench maxCmdRaw', CONFIG.control.maxCmdRaw, 300);
eq('pre slower1.3', Math.round(speedPreset({ ...CONFIG, mech: { ...CONFIG.mech, gearRatio: 20 } }, 1.3).control.maxCmdRaw), 12414);
eq('pre overrated1.3', (() => { try { speedPreset(CONFIG, 1.3); return 'no-throw'; } catch (e) { return /超过额定折算上限/.test(e.message) ? 'throws' : 'wrong-msg'; } })(), 'throws');
eq('pre gears0 throws', (() => { try { suggestedStartGear(CONFIG, 0); return 'no-throw'; } catch { return 'throws'; } })(), 'throws');
{
  const gearMps = [1, 2, 3, 4].map((g) => Number(speedMpsFromRaw(quantize(Math.round(p05.control.maxCmdRaw * g / 4), p05.control.quantizeRaw, p05.control.maxCmdRaw), p05).toFixed(3)));
  eq('pre gear ladder', gearMps, [0.127, 0.249, 0.376, 0.498]);
  const wire = T({ vx: 1, vy: 0, wz: 0 }, p05).map((v) => quantize(v, p05.control.quantizeRaw, p05.control.maxCmdRaw));
  eq('pre 4/4 wire', wire, [7140, 7140, -7140, -7140]);
  const st = makeThrottleState(4);
  let t = 0;
  const tgt = [1, 1, -1, -1].map((s) => s * p10.control.maxCmdRaw);
  for (let i = 0; i < 200; i++) { planWrites(st, tgt, t, p10.control); t += 50; }
  eq('pre 1.0 converge', st.written, [14210, 14210, -14210, -14210]);
  eq('pre 1.0 mps', speedMpsFromRaw(14210, p10), 0.992, 0.002);
  eq('pre 1.0 writes<=24', st.counts.writes <= 24, true);
}

eq('mb write +300', writeRequest(1, 0x0303, 300).toString('hex').toUpperCase(), '01060303012C79C3');
eq('mb write -7140', writeRequest(2, 0x0303, -7140).toString('hex').toUpperCase(), '02060303E41C3374');
eq('mb read monitor', readRequest(1, 0x0700, 24).toString('hex').toUpperCase(), '01030700001844B4');

console.log(`\n${n} 项对账，${bad} 项不一致`);
process.exit(bad ? 1 : 0);
