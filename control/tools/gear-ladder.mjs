// 一次性安全确认脚本（只跟本地控制环说话，跑完即删）。
// 节奏：喂 1/4 档前进 3 秒 → 采样遥测 → 归零 → 再采样确认线上是 0。
const B = 'http://127.0.0.1:8765';
const post = (p, body) => fetch(B + p, { method: 'POST', body: body ? JSON.stringify(body) : '' }).then((r) => r.json());
const get = (p) => fetch(B + p).then((r) => r.json());
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

const brief = (s) => `${s.state}/${s.action} 目标[${s.target.join(',')}] 线上[${s.onWire.join(',')}] `
  + `SPd[${s.wheels.map((w) => w.driveSpd).join(',')}] 执行中命令[${s.wheels.map((w) => w.driveCmd).join(',')} `
  + `] 写${s.eeprom?.writes ?? '?'}次 延后${s.eeprom?.deferred ?? '?'} 失败${s.eeprom?.failed ?? '?'}`;

const st0 = await get('/state');
console.log('cfg 满杆', st0.cfg.maxRaw, 'raw =', st0.cfg.fullScaleMps.toFixed(3), 'm/s；各档 raw:',
  st0.cfg.gearMps.map((m, i) => `${i + 1}/4≈${m.toFixed(3)}`).join(' '));
console.log('进档前：', brief(st0.snapshot));

const gear = Number(process.argv[2] || 1);
const frac = gear / 4;
// 注意：/twist 收的是 **满杆比例 [-1,1]**，不是 m/s（controller.setTwist 的口径）。
const vx = frac;
const mps = st0.cfg.fullScaleMps * frac;
const holdMs = Number(process.argv[3] || 3000);
console.log(`喂入 ${gear}/4 档前进 vx=${vx}(比例) ≈ ${mps.toFixed(3)} m/s，持续 ${holdMs} ms（看门狗 400 ms，所以每 200 ms 续一次）`);

const t0 = Date.now();
const samples = [];
while (Date.now() - t0 < holdMs) {
  await post('/twist', { vx, vy: 0, wz: 0 });
  await sleep(200);
  samples.push((await get('/state')).snapshot);
}
console.log('喂入过程中（每 5 个样本取一个）：');
samples.forEach((s, i) => { if (i % 5 === 0 || i === samples.length - 1) console.log(`  +${((i + 1) * 200)}ms`, brief(s)); });

const peak = samples.reduce((a, b) => (Math.max(...b.onWire.map(Math.abs)) > Math.max(...a.onWire.map(Math.abs)) ? b : a), samples[0]);
console.log('峰值那拍：', brief(peak));

await post('/stop');
await sleep(800);
console.log('归零后：', brief((await get('/state')).snapshot));
await sleep(800);
console.log('再等一拍：', brief((await get('/state')).snapshot));
