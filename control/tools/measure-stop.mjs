// 量一件事：从"指令归零"到"轮子真的停下"要多久（离地、无摩擦负载 → 这是最慢的情况）。
// 注意：idle 时控制环每 1 秒才做一次只读心跳，所以采样点会一跳一跳的，精度约 ±1 s。
const B = 'http://127.0.0.1:8765';
const post = (p) => fetch(B + p, { method: 'POST', body: '' }).then((r) => r.json());
const get = (p) => fetch(B + p).then((r) => r.json());
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const spdOf = (s) => s.wheels.map((w) => w.driveSpd);

const st = await get('/state');
const full = st.cfg.fullScaleMps;
console.log(`满杆 ${full.toFixed(3)} m/s（${st.cfg.maxRaw} raw），先跑到 4/4 档稳住`);
const t0 = Date.now();
while (Date.now() - t0 < 3500) {
  await fetch(B + '/twist', { method: 'POST', body: JSON.stringify({ vx: 1, vy: 0, wz: 0 }) });
  await sleep(200);
}
const run = (await get('/state')).snapshot;
console.log(`稳住时：线上[${run.onWire.join(',')}] 自报命令[${run.wheels.map((w) => w.driveCmd).join(',')}] 实际[${spdOf(run).join(',')}]`);

const tStop = Date.now();
await post('/stop');
console.log('已下发归零，开始采样实际转速：');
for (let i = 0; i < 20; i++) {
  const s = (await get('/state')).snapshot;
  const spd = spdOf(s);
  const mx = Math.max(...spd.map(Math.abs));
  console.log(`  +${Date.now() - tStop} ms [${spd.join(',')}] ${s.state} ${s.reason || ''}`);
  if (mx === 0 && i > 2) break;
  await sleep(300);
}
console.log(`从归零到全停用时约 ${Date.now() - tStop} ms（离地空转；落地有摩擦只会更快）`);
