import { test } from 'node:test';
import assert from 'node:assert/strict';
import { CONFIG } from '../src/config.js';
import { planCeilingWrite, formatCeilingPlan } from '../src/ceiling.js';

/** 一台"停着、额定 3000 rpm、限幅还是台架 300"的驱动器 */
const good = (over = {}) => ({ wheel: 'FL', name: '左前', port: 'COM12', slave: 1,
  info: { ratedSpd: 3000, ratedPlausible: true, ceiling: 300, preset: 0, spd: 0, ...over } });
const four = (specs = []) => ['FL', 'FR', 'BL', 'BR'].map((id, i) => {
  const r = good(specs[i] || {});
  r.wheel = id; r.port = `COM${12 + i}`; r.slave = i + 1;
  return r;
});

test('闸门全绿：目标 = 额定对应 15000 raw，四台都停着 → 允许写', () => {
  const p = planCeilingWrite(four(), CONFIG, 15000);
  assert.equal(p.ok, true, p.problems.join('；'));
  assert.equal(p.hardCap, 15000);
  assert.match(formatCeilingPlan(p, CONFIG), /闸门通过/);
  assert.match(formatCeilingPlan(p, CONFIG), /先把四台使能断开/, 'Sd-On 这一课要印在计划里：使能状态读不到，只能靠现场动作提示');
});

test('目标越过额定 → 拦：手册能填 30000，不代表能跑 6000 rpm', () => {
  const p = planCeilingWrite(four(), CONFIG, 30000);
  assert.equal(p.ok, false);
  assert.match(p.problems.join(' '), /超过允许值 15000/);
  assert.match(formatCeilingPlan(p, CONFIG), /✋/);
});

test('四台额定不一致 → 按最小的那台设天花板，最弱的不能被超额定', () => {
  const p = planCeilingWrite(four([{}, {}, { ratedSpd: 2500 }, {}]), CONFIG, 15000);
  assert.equal(p.hardCap, 12500, '2500 rpm × 5 = 12500 raw');
  assert.equal(p.ok, false);
  assert.match(p.problems.join(' '), /超过允许值 12500/);
});

test('线上还留着速度指令或在转 → 拦（Pn-303=0 不等于轮子停着，反过来也一样要防）', () => {
  assert.match(planCeilingWrite(four([{ preset: 300 }]), CONFIG, 15000).problems.join(' '), /还留着速度指令/);
  assert.match(planCeilingWrite(four([{ spd: -59 }]), CONFIG, 15000).problems.join(' '), /正在转/);
});

test('读不到就等于不安全：额定/限幅/Pn-303/转速 任一缺失都拦（这是"幽灵 -59"那一课的形状）', () => {
  const cases = [
    [{ ratedSpd: null, ratedPlausible: false }, /读不到 Fn-1C/],
    [{ ceiling: null }, /读不到当前 Pn-30D/],
    [{ preset: null }, /读不到 Pn-303/],
    [{ spd: null }, /读不到实际转速/],
  ];
  for (const [spec, re] of cases) {
    const rows = four([spec]);
    rows[0].info.ratedPlausible = spec.ratedPlausible ?? true;
    const p = planCeilingWrite(rows, CONFIG, 15000);
    assert.equal(p.ok, false, `${re} 应当拦下来`);
    assert.match(p.problems.join(' '), re);
  }
});

test('额定读数不合理（比如越界拿到的 10851）→ 拦，不当真', () => {
  const rows = four([{ ratedSpd: 10851 }]);
  rows[0].info.ratedPlausible = false;
  const p = planCeilingWrite(rows, CONFIG, 15000);
  assert.equal(p.ok, false);
  assert.match(p.problems.join(' '), /不在合理区间/);
});

test('输入本身不合法：探测行数不对 / 目标不是正整数 → 直接拦', () => {
  assert.match(planCeilingWrite(four().slice(0, 3), CONFIG, 15000).problems.join(' '), /不是 4 行/);
  for (const bad of [0, -1, 1.5, NaN, undefined]) {
    assert.equal(planCeilingWrite(four(), CONFIG, bad).ok, false, `${bad} 必须被拒`);
  }
});

test('静态 SPd=-1（Pn-303 已经是 0）不拦写参数，但要在计划里说明这是抖动不是"在转"', () => {
  // 2026-10-09 实机就卡在这：四台限幅仍是 300、传令全为 0，FL/BR 报 SPd=-1，旧闸门用"必须正好为 0"
  // 于是永远写不成 —— 逼人绕开闸门才是最危险的后果。
  const p = planCeilingWrite(four([{ spd: -1 }, {}, {}, { spd: -1 }]), CONFIG, 15000);
  assert.equal(p.ok, true, p.problems.join('；'));
  assert.match(formatCeilingPlan(p, CONFIG), /停转容差/);
  assert.match(formatCeilingPlan(p, CONFIG), /FL\(SPd=-1\) BR\(SPd=-1\)/);
  assert.match(planCeilingWrite(four([{ spd: -59 }]), CONFIG, 15000).problems.join(' '), /超出停转容差 2 rpm/);
});

test('当前限幅读数本身不可信（负数 / 越过手册 30000）→ 拦：不知道现状就不改现状', () => {
  for (const bad of [-5536, 32000]) {
    const p = planCeilingWrite(four([{ ceiling: bad }]), CONFIG, 15000);
    assert.equal(p.ok, false, `现状读成 ${bad} 时不许写`);
    assert.match(p.problems.join(' '), /超出手册范围/);
  }
  const flagged = four();
  flagged[2].info.ceilingPlausible = false; // 探测层已经判过"这读数不像"，闸门要尊重它
  assert.match(planCeilingWrite(flagged, CONFIG, 15000).problems.join(' '), /超出手册范围/);
});

test('退回小限幅也走同一套闸门（--to 300 --confirm 300 是合法操作）', () => {
  const p = planCeilingWrite(four(), CONFIG, 300);
  assert.equal(p.ok, true, p.problems.join('；'));
  assert.equal(p.target, 300);
});
