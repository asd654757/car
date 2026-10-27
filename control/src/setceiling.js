#!/usr/bin/env node
/**
 * 整个项目里唯一会**写驱动器参数**的入口：Pn-30D（内部速度限制）。
 *
 *   node src/setceiling.js                  只打印计划（一个字节都不写，退出码 2）
 *   node src/setceiling.js --confirm 15000   真的写（值必须和计划一致，否则拒绝）
 *   node src/setceiling.js --to 300 --confirm 300   退回台架小限幅
 *
 * 为什么独立一个脚本、还要 --confirm：
 *  - 控制环入口（cli.js）永远不写参数，就不会"顺手把限幅抬了"；提速只可能是显式执行的。
 *  - Pn-30D 是 EEPROM 参数（手册故障 08 = EEPROM 超出写入次数）：四台各一次 = 4 次写入，花掉就没了。
 *  - 限幅抬高后，驱动器自己不再拦大指令，唯一的闸是脚本层的 maxCmdRaw —— 所以目标值只能 ≤ 铭牌额定
 *    （Fn-1C 实机 3000 rpm），并且取四台里最小的那个额定，最弱的那台不能被打到超额定。
 *  - 判据全在 src/ceiling.js 的纯函数里（planCeilingWrite），有单测；本文件只做串口 IO 和确认流程。
 *  - 写完不必改回来：实际车速由 `node src/cli.js --mps <值>` 在软件侧限，不再消耗 EEPROM 写次数。
 */

import { CONFIG, validateConfig } from './config.js';
import { RtuBus } from './bus.js';
import { readNameplate } from './preflight.js';
import { planCeilingWrite, formatCeilingPlan } from './ceiling.js';
import { ceilingForRated } from './preset.js';

const argv = process.argv.slice(2);
const val = (f, d) => { const i = argv.indexOf(f); return i >= 0 && argv[i + 1] ? argv[i + 1] : d; };
const has = (f) => argv.includes(f);
const log = (m) => process.stdout.write(`[${new Date().toISOString().slice(11, 19)}] ${m}\n`);

const cfg = structuredClone(CONFIG);
const errs = validateConfig(cfg);
if (errs.length) { console.error(`配置自检失败：\n - ${errs.join('\n - ')}`); process.exit(2); }

const requested = Number(val('--to', ceilingForRated(cfg)));
if (!Number.isInteger(requested) || requested < 1) {
  console.error(`--to 必须是正整数 raw（收到 ${val('--to')}）；默认目标 = 额定对应 ${ceilingForRated(cfg)}`);
  process.exit(2);
}

/** 只读地把"写参数前要知道的事"凑齐：额定转速、当前限幅、线上指令、实际转速 */
async function gather(bus) {
  const np = await readNameplate(bus, cfg);
  const mon = await Promise.all(cfg.wheels.map(async (w, i) => {
    try {
      const r = await bus.readRegs(i, cfg.regs.monitor, cfg.regs.monitorQty);
      return { ok: r.ok, spd: r.ok ? r.regs[cfg.regs.mon.spd] : null, error: r.error };
    } catch (e) { return { ok: false, spd: null, error: String(e.message || e) }; }
  }));
  return np.rows.map((r, i) => ({ ...r, info: { ...r.info, spd: mon[i].spd } }));
}

async function main() {
  const bus = new RtuBus(cfg.wheels, { baud: 9600, timeoutMs: 300 });
  await bus.open();
  let code = 0;
  try {
    const rows = await gather(bus);
    const plan = planCeilingWrite(rows, cfg, requested);
    console.log(formatCeilingPlan(plan, cfg));
    if (!plan.ok) {
      console.log('\n✋ 闸门没过，一个字节都没写。逐条处理后重跑本命令。');
      code = 2; return;
    }
    console.log(`  写入次数：${rows.length} 次（每台 1 次）。写完不必改回，车速档由 --mps 在软件侧切。`);
    console.log('  ⚠ 改完之后唯一的闸是脚本层 maxCmdRaw：轮子离地或场地清空之后再进控制环，第一档从 --mps 0.2~0.3 起。');
    if (String(val('--confirm', '')) !== String(requested)) {
      console.log(`\n未确认，什么都没写。确认执行：node src/setceiling.js ${has('--to') ? `--to ${requested} ` : ''}--confirm ${requested}`);
      code = 2; return;
    }

    /** 回读一次；EEPROM 落盘可能比应答慢，所以读不到就退避着重读，最多三次 */
    const readBack = async (i) => {
      for (const wait of [250, 500, 900]) {
        await new Promise((r) => setTimeout(r, wait));
        const b = await bus.readRegs(i, cfg.regs.ceiling, 1);
        if (b.ok) return b.regs[cfg.regs.ceiling];
      }
      return null;
    };

    // 只写"还没到位"的那几台：已经等于目标值的重复写，白烧一次 EEPROM 写次数。
    const todo = rows.map((r, i) => ({ i, wheel: cfg.wheels[i], from: r.info.ceiling }))
      .filter((t) => t.from !== requested);
    if (todo.length !== rows.length) {
      log(`${rows.length - todo.length} 台当前已是 ${requested}，跳过不重复写（EEPROM 写次数是有限资源）`);
    }
    if (!todo.length) {
      console.log(`\n四台限幅已经是 ${requested} raw，本次一次都没写。控制环仍需显式给车速档：node src/cli.js --mps 0.3 起步。`);
      code = 0; return;
    }
    log(`开始写入 Pn-30D = ${requested}（${todo.length} 台并发各一次）`);
    const results = await Promise.all(todo.map(async ({ i, wheel }) => {
      try {
        const r = await bus.writeReg(i, cfg.regs.ceiling, requested, { eeprom: true });
        // 回显失败**不等于没写进去**：这类参数是"先落盘、后应答"，应答可能比超时晚到。
        // 所以不管回显如何都回读，以读回来的值为准 —— 否则会出现"脚本报失败、驱动器其实已经改好"
        // 这种最危险的状态不一致。
        const got = await readBack(i);
        if (got === requested) return { wheel: wheel.id, ok: true, got, note: r.ok ? null : `回显异常但值已写入：${r.error}` };
        return { wheel: wheel.id, ok: false, got, error: r.ok ? `回显说成功，回读却是 ${got ?? '读不到'}` : r.error };
      } catch (e) {
        // 单个口的异常（打开失败/写抛错）不能把四台的结论一起带走：那样连"哪台改了哪台没改"都查不到。
        return { wheel: wheel.id, ok: false, got: null, error: `本台异常：${e.message || e}（其余台的结果仍然有效）` };
      }
    }));
    for (const t of todo) {
      const r = results.find((x) => x.wheel === t.wheel.id);
      console.log(`  ${r.ok ? '✓' : '✗'} ${r.wheel} ${t.wheel.name} ${t.wheel.port}/站${t.wheel.slave}：${t.from} → 回读 ${r.got ?? '读不到'}`
        + (r.ok ? (r.note ? `  （${r.note}）` : '') : `  （${r.error}）`));
    }
    const bad = results.filter((r) => !r.ok);
    if (bad.length) {
      console.log(`\n⚠ ${bad.length} 台没确认到位。别在"限幅不一致"的状态下跑控制环 —— 预检会因为 Pn-30D 低于软顶而拒绝，这正是要的行为。`);
      console.log('每行括号里那句现在会带回收到的字节（或"无响应"），按它对照：');
      console.log('  · 报 exception 0x06 / Sd-On → 手册原文"伺服器正在运行中，不能修改该数据"：先把四台使能关掉（面板 SON 断开）再重跑本命令');
      console.log('  · 报 无响应（超时）→ 帧发出去了但它没答：写 EEPROM 期间有的固件就是不回应，重跑一次看回读值；仍无响应再查 485 接线与共地');
      console.log('  · 报 exception 0x02/0x03 → 地址或值不对（不该发生，手册写的是 0~30000 可写）：把上面那行十六进制原样发我，别再重试');
      console.log('  · 想确认现在到底是什么状态：node src/cli.js --motors   （只读，一个字节都不写）');
      code = 3;
    } else {
      console.log(`\n四台限幅已确认为 ${requested} raw。控制环仍需显式给车速档：node src/cli.js --mps 0.3 起步，逐档重新做安全确认。`);
    }
  } catch (e) {
    console.error(`异常，未继续写入：${e.message}`);
    code = 1;
  } finally {
    try { await bus.close(); } catch { /* 关不掉也不能盖掉主结果的退出码 */ }
  }
  process.exit(code);
}

main().catch(() => process.exit(1));
