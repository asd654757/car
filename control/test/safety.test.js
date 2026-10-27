import { test } from 'node:test';
import assert from 'node:assert/strict';
import { CONFIG } from '../src/config.js';
import {
  STATE, makeSafetyState, noteInput, noteComm, latchEstop, latchFault, clearLatch, decide,
} from '../src/safety.js';

const C = CONFIG.control;
const T = C.inputTimeoutMs; // 400ms

test('刚开机、还没收到任何输入 → 不许动（默认停）', () => {
  const st = makeSafetyState();
  const d = decide(st, 1000, C);
  assert.equal(d.forceZero, true);
  assert.equal(d.state, STATE.IDLE);
});

test('输入新鲜 → RUN；超过 inputTimeoutMs 没输入 → 自动回 IDLE 并强制归零', () => {
  const st = makeSafetyState();
  noteInput(st, 1000);
  assert.equal(decide(st, 1000, C).forceZero, false);
  assert.equal(decide(st, 1000 + T - 10, C).forceZero, false, '刚好在窗口内还算新鲜');
  const d = decide(st, 1000 + T + 1, C);
  assert.equal(d.forceZero, true, '看门狗：手柄掉了/网页关了/进程卡住，车必须自己停');
  assert.equal(d.state, STATE.IDLE);
  assert.match(d.reason, /输入超时/);
});

test('持续喂输入可以一直保持 RUN（不会自己跳停）', () => {
  const st = makeSafetyState();
  let t = 0;
  for (let i = 0; i < 200; i++) { noteInput(st, t); assert.equal(decide(st, t, C).forceZero, false, `第 ${i} 拍不该停`); t += 1000 / C.loopHz; }
  assert.equal(st.state, STATE.RUN);
});

test('急停闭锁：有新输入也不许动，只有显式解除才恢复', () => {
  const st = makeSafetyState();
  noteInput(st, 1000);
  latchEstop(st, '按下急停');
  let t = 1010;
  for (let i = 0; i < 50; i++) {
    noteInput(st, t); // 输入一直新鲜
    const d = decide(st, t, C);
    assert.equal(d.forceZero, true, '闭锁期间任何拍都必须归零');
    assert.equal(d.state, STATE.ESTOP);
    t += 1000 / C.loopHz;
  }
  assert.equal(st.state, STATE.ESTOP, '闭锁不能自己解开');
  clearLatch(st);
  assert.equal(st.state, STATE.IDLE);
  assert.equal(decide(st, t, C).forceZero, false, '解除后靠新鲜输入恢复运行');
});

test('通讯连续失败到达上限 → 故障闭锁（这台驱动器不会自己停）', () => {
  const st = makeSafetyState();
  noteInput(st, 0);
  for (let i = 1; i < C.commFailLimit; i++) {
    noteComm(st, i * 100, false);
    assert.equal(decide(st, i * 100, C).forceZero, false, `失败 ${i} 次还没到上限 ${C.commFailLimit}`);
    assert.equal(st.state, STATE.RUN, '未到上限不该误锁');
  }
  for (let i = 0; i < C.commFailLimit; i++) noteComm(st, 1000 + i * 100, false);
  const d = decide(st, 1400, C);
  assert.equal(d.state, STATE.FAULT);
  assert.equal(d.forceZero, true);
  assert.match(d.reason, /通讯失败/);
});

test('一次成功通讯就把失败计数清零（偶发丢帧不会累积成故障）', () => {
  const st = makeSafetyState();
  noteInput(st, 0);
  for (let i = 0; i < C.commFailLimit - 1; i++) noteComm(st, i * 100, false);
  noteComm(st, 900, true);
  assert.equal(st.failStreak, 0);
  for (let i = 0; i < C.commFailLimit - 1; i++) noteComm(st, 1000 + i * 100, false);
  noteInput(st, 1490); // 输入仍然新鲜，排除看门狗干扰
  assert.equal(decide(st, 1500, C).state, STATE.RUN, '断续失败没连着 → 不闭锁');
});

test('故障闭锁同样要显式解除，解除前一直强制归零', () => {
  const st = makeSafetyState();
  noteInput(st, 5000);
  latchFault(st, '驱动器报警');
  assert.equal(decide(st, 5010, C).forceZero, true);
  assert.equal(decide(st, 5010, C).state, STATE.FAULT);
  clearLatch(st);
  noteComm(st, 5100, true);
  assert.equal(decide(st, 5100, C).forceZero, false);
});
