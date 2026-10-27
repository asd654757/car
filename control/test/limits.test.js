import { test } from 'node:test';
import assert from 'node:assert/strict';
import { CONFIG } from '../src/config.js';
import { makeThrottleState, planWrites, planStopAll, quantize, rampToward } from '../src/limits.js';

const C = CONFIG.control;
const N = CONFIG.wheels.length;

test('量化到 2 rpm 一档', () => {
  assert.equal(quantize(103, 10), 100);
  assert.equal(quantize(106, 10), 110);
  assert.equal(quantize(-104, 10), -100);
  assert.equal(quantize(4, 10), 0, '小于半档的抖动直接被吃掉');
});

test('量化不许越过 maxCmdRaw（1/4 档实测就是撞上这条）', () => {
  assert.equal(quantize(75, 10), 80, '不带 cap 时四舍五入确实会抬高');
  assert.equal(quantize(75, 10, 75), 70, '带 cap → 越界就向下退一整档，不贴 cap：74/75 抖动不会来回写');
  assert.equal(quantize(-75, 10, 75), -70);
  assert.equal(quantize(74, 10, 75), 70, 'cap 内正常量化与"越界退档"落在同一档 → 输入抖半档零写入');
  assert.equal(quantize(106, 10, 100), 100, '越过 cap 的量化值退到 cap 以下最大的整档');
  assert.equal(quantize(103, 10, 100), 100, 'cap 内的正常量化不受影响');
  assert.equal(quantize(20, 10, 5), 5, '一档都塞不进 cap 时才贴 cap，宁可慢也不越限幅');
  assert.throws(() => quantize(10, 10, 0), /cap 非法/);
  assert.throws(() => quantize(10, 10, Number.NaN), /cap 非法/);

  const st = makeThrottleState(N);
  let t = 0;
  for (let i = 0; i < 100; i++) { planWrites(st, [75, 75, -75, -75], t, { ...C, maxCmdRaw: 75 }); t += 50; }
  assert.deepEqual(st.written, [70, 70, -70, -70], '整条节流链路收敛在 70（≤ 限幅 75），不会停在 80');
  const before = st.counts.writes;
  planWrites(st, [75, 75, -75, -75], t, { ...C, maxCmdRaw: 75 });
  assert.equal(st.counts.writes, before, '贴到档上以后也是稳态：同一个输入永远得同一个值 → 零写入');
});

test('斜坡：每拍最多变 rampRawPerTick', () => {
  assert.equal(rampToward(0, 300, 40), 40);
  assert.equal(rampToward(0, -300, 40), -40);
  assert.equal(rampToward(280, 300, 40), 300, '最后一步不越界');
});

/** 让斜坡/限频跑到"线上值 = 量化目标"，返回收敛后的时间戳 */
function settle(st, target, cfg, startT = 0) {
  const want = target.map((v) => quantize(v, cfg.quantizeRaw));
  let t = startT;
  for (let i = 0; i < 5000; i++) {
    planWrites(st, target, t, cfg);
    t += 1000 / cfg.loopHz;
    if (st.written.every((v, k) => v === want[k])) break; // 只有"到位"才算收敛，被限频拦住的一拍不算
  }
  return t;
}

test('与线上值相同 → 一个字节都不发（EEPROM 保护的主路径）', () => {
  const st = makeThrottleState(N);
  settle(st, [100, 100, -100, -100], C);
  assert.deepEqual(st.written, [100, 100, -100, -100], '斜坡最终落到量化目标');
  const r = planWrites(st, [100, 100, -100, -100], 5000, C);
  assert.deepEqual(r.writes, [], '稳态行驶必须零写入');
});

test('非零写入受 minWriteIntervalMs 限频', () => {
  const st = makeThrottleState(N);
  planWrites(st, [100, 100, -100, -100], 1000, C);
  const soon = planWrites(st, [200, 200, -200, -200], 1000 + C.minWriteIntervalMs - 50, C);
  assert.equal(soon.writes.length, 0, '未到间隔 → 推迟');
  assert.deepEqual(soon.deferred, [0, 1, 2, 3]);
  const later = planWrites(st, [200, 200, -200, -200], 1000 + C.minWriteIntervalMs, C);
  assert.equal(later.writes.length, N, '到点就补上');
  assert.equal(st.counts.deferred, N, '被推迟的轮要计数，控制核靠这个数字判断"是不是写不过来了"');
  assert.equal(st.counts.writes, N + N, '计数只统计真正落到线上的次数');
});

test('归零不受限频与斜坡约束（急停必须最快）', () => {
  const st = makeThrottleState(N);
  settle(st, [300, 300, -300, -300], C);
  assert.deepEqual(st.written, [300, 300, -300, -300], '先爬到满速，才有"从满速急停"');
  const t = st.lastWriteAt[0] + 1; // 故意卡在限频窗口内
  const r = planWrites(st, [0, 0, 0, 0], t, C);
  assert.equal(r.writes.length, N, '紧接上一次写入也要立刻归零');
  assert.deepEqual(r.writes.map((w) => w.value), [0, 0, 0, 0], '一步到 0，不许被斜坡拆成 260/220/…');
  assert.deepEqual(r.deferred, []);

  const st2 = makeThrottleState(N);
  settle(st2, [300, 300, -300, -300], C);
  const near = planWrites(st2, [2, 2, -2, -2], st2.lastWriteAt[0] + 1, C); // 小于半档 → 量化为 0
  assert.deepEqual(near.writes.map((w) => w.value), [0, 0, 0, 0], '回中抖动也要立刻停，不能留在 260');
});

test('planStopAll 只写当前非零的那几路', () => {
  const st = makeThrottleState(N);
  st.written = [100, 0, -50, 0];
  const w = planStopAll(st);
  assert.deepEqual(w.map((x) => x.idx), [0, 2]);
  assert.deepEqual(st.written, [0, 0, 0, 0]);
  assert.deepEqual(planStopAll(st), [], '已经全零 → 不再写');
});

test('长时间恒定输入 → 写入次数是常数级，不随时间线性增长', () => {
  const st = makeThrottleState(N);
  const perTick = [];
  let t = 0;
  for (let i = 0; i < 2000; i++) { // 100 秒 @20 Hz
    const before = st.counts.writes;
    planWrites(st, [300, 300, -300, -300], t, C);
    perTick.push(st.counts.writes - before);
    t += 1000 / C.loopHz;
  }
  assert.equal(st.written[0], 300);
  const steps = Math.ceil(300 / C.rampRawPerTick);
  assert.equal(st.counts.writes, N * steps, `爬坡写入应恰好 = 轮数 × 斜坡档数 = ${N * steps}，实际 ${st.counts.writes}`);
  assert.equal(perTick.slice(200).reduce((a, b) => a + b, 0), 0, '稳态的 90 秒里一字节都不发');
  const lastWriteTick = perTick.reduce((last, v, i) => (v ? i : last), 0);
  assert.ok(lastWriteTick * (1000 / C.loopHz) <= C.minWriteIntervalMs * steps + 1000 / C.loopHz,
    `满速爬坡受"每拍最多变 ${C.rampRawPerTick} + 限频 ${C.minWriteIntervalMs}ms"双重约束，实际用了 ${lastWriteTick * 50} ms`);
});

test('开机第一帧不被限频吃掉（lastWriteAt 初值坑）', () => {
  const st = makeThrottleState(N);
  const r = planWrites(st, [50, 50, -50, -50], 0, C);
  assert.equal(r.writes.length, N, 't=0 就能下发，不能因为"上次写入时间是 0"而推迟');
});
