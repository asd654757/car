import { test } from 'node:test';
import assert from 'node:assert/strict';
import { dispatchKey, twistFromKeys, sweepHeld, KEYMAP, CTRL_C } from '../src/keyinput.js';

const NAMES = ['exit', 'estop', 'release', 'zero', 'clearKeys', 'help', 'snapshot', 'gear', 'press'];
function recorder() {
  const calls = [];
  const api = {};
  for (const name of NAMES) api[name] = (arg) => calls.push({ name, arg });
  return { calls, api };
}

test('raw 终端里的 Ctrl+C 必须走 exit —— 这是唯一不会留速度在线上的退法', () => {
  const { calls, api } = recorder();
  assert.equal(dispatchKey(CTRL_C, api), 'exit');
  assert.deepEqual(calls, [{ name: 'exit', arg: 'Ctrl+C' }]);
  // 0x03 若不单独认，就会被下面的 toLowerCase 分支放过：它不等于 'c'，也不在任何键位表里 → 永远退不出去
  assert.notEqual(CTRL_C, 'c');
  assert.equal(dispatchKey(CTRL_C, api) !== 'ignore', true);
});

test('功能键各归各位，顺序也有讲究（x 要先清空按键集合再归零）', () => {
  const a = recorder(); dispatchKey(' ', a.api);
  assert.deepEqual(a.calls, [{ name: 'estop', arg: '空格急停' }]);

  const b = recorder(); dispatchKey('x', b.api);
  assert.deepEqual(b.calls.map((c) => c.name), ['clearKeys', 'zero'], '先 clear 再 zero，否则 keepAlive 下一拍又把按住的方向键写回去');

  const c = recorder(); dispatchKey('r', c.api);
  assert.deepEqual(c.calls, [{ name: 'release', arg: undefined }]);
  const d = recorder(); dispatchKey('h', d.api);
  assert.deepEqual(d.calls, [{ name: 'help', arg: undefined }]);
  const e = recorder(); dispatchKey('c', e.api);
  assert.deepEqual(e.calls, [{ name: 'snapshot', arg: undefined }]);
});

test('幅度档只认 1-4；大小写都认；别的字节一律不碰硬件', () => {
  for (const [ch, expect] of [['1', 1], ['2', 2], ['3', 3], ['4', 4]]) {
    const r = recorder();
    assert.equal(dispatchKey(ch, r.api), 'gear');
    assert.deepEqual(r.calls, [{ name: 'gear', arg: expect }]);
  }
  for (const ch of ['5', '0']) {
    const r = recorder();
    assert.equal(dispatchKey(ch, r.api), 'ignore');
    assert.deepEqual(r.calls, [], '超出档位的数字不许把量程改掉');
  }
  for (const ch of ['W', 'A', 'q', 'E']) {
    const r = recorder();
    assert.equal(dispatchKey(ch, r.api), 'press');
    assert.equal(r.calls[0].arg, ch.toLowerCase(), '大写和 shift 组合要归成同一个小写键');
    assert.ok(KEYMAP[r.calls[0].arg], '按住的必须是指北的键');
  }
  for (const ch of ['\r', '\n', 'z', '	', '']) {
    const r = recorder();
    assert.equal(dispatchKey(ch, r.api), 'ignore');
    assert.deepEqual(r.calls, [], `回车/未知字节 ${JSON.stringify(ch)} 不能触发任何动作`);
  }
});

test('同轴两键互相抵消，档位按比例缩放且不吃溢', () => {
  assert.deepEqual(twistFromKeys(new Set(['w', 's']), 4), { vx: 0, vy: 0, wz: 0 }, 'W+S 同时按 = 不进不退');
  assert.deepEqual(twistFromKeys(new Set(['w']), 4), { vx: 1, vy: 0, wz: 0 });
  assert.deepEqual(twistFromKeys(new Set(['w']), 2), { vx: 0.5, vy: 0, wz: 0 });
  assert.deepEqual(twistFromKeys(new Set(['w', 'a', 'q']), 4), { vx: 1, vy: 1, wz: 1 }, '斜着走 = 三轴同时打满');
  assert.deepEqual(twistFromKeys(new Set(['s', 'd', 'e']), 4), { vx: -1, vy: -1, wz: -1 });
  assert.deepEqual(twistFromKeys(new Set(['w', 'w']), 4).vx, 1, 'Set 去重之外也要防：同键不会叠成 2');
  assert.deepEqual(twistFromKeys(new Set(['w']), 0), { vx: 0, vy: 0, wz: 0 }, '0 档 = 停');
  assert.deepEqual(twistFromKeys(new Set(['w']), 99).vx, 1, '档位被配坏也只能到满量程，不能超');
  assert.deepEqual(twistFromKeys(new Set(['w']), Number.NaN).vx, 0, 'NaN 档位 → 不动，不是 NaN 传播到四轮');
  assert.deepEqual(twistFromKeys(new Set(['不存在的键']), 4), { vx: 0, vy: 0, wz: 0 });
});

test('自动重复断了 = 已松手（raw 模式没有 keyup 的替代判据）', () => {
  const held = new Set(['w', 'a']);
  const seen = { w: 1000, a: 1400 };
  assert.deepEqual(sweepHeld(held, seen, 1000 + 650, 650), [], '刚好在宽限期内不算松手');
  assert.deepEqual(sweepHeld(held, seen, 1000 + 651, 650), ['w'], '只有 w 的重复断了 → 只掉 w');
  assert.deepEqual(Array.from(held), ['a']);
  assert.deepEqual(sweepHeld(held, seen, 9e9, 650), ['a']);
  assert.equal(held.size, 0);
  const empty = new Set();
  assert.deepEqual(sweepHeld(empty, {}, 5000, 650), [], '没按键时不该有事发生');
});
