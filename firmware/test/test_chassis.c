/* 控制核 C 版的回归测试。期望值不是手算的，是从已实机验证的 JS 控制核里跑出来抄写的
 * （和 src/ 那 120 条同批锚点），所以这份文件同时是"JS → C 翻译没走样"的证据。
 * 有宿主机 C 编译器就能跑：
 *   cc -I../core -I../port test_chassis.c （core 下所有 .c）../port/chassis_transport.c -lm && ./a.out
 * （传输层的测试要真链路，所以连 chassis_transport.c 一起编，板级 uart 层不参与）
 * 没有编译器时先用 tools/check_build.ps1 以 armclang 做编译检查。 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "chassis_config.h"
#include "chassis_controller.h"
#include "chassis_kinematics.h"
#include "chassis_limits.h"
#include "chassis_preset.h"
#include "chassis_transport.h"
#include "modbus_rtu.h"

static int g_fail = 0;
static int g_checks = 0;

#define CHECK(cond, name) do { \
    g_checks++; \
    if (!(cond)) { g_fail++; printf("FAIL  %s\n", name); } \
} while (0)

static bool eq4i(const int a[4], int b0, int b1, int b2, int b3)
{
    return a[0] == b0 && a[1] == b1 && a[2] == b2 && a[3] == b3;
}

static bool near(double a, double b, double tol) { return fabs(a - b) <= tol; }

/* ---------- 运动学 ---------- */

static void test_kinematics(void)
{
    chassis_config_t cfg, p05;
    chassis_twist_t fwd = { 1, 0, 0 }, strafe = { 0, 1, 0 }, spin = { 0, 0, 1 };
    chassis_twist_t back = { -1, 0, 0 }, none = { 0, 0, 0 }, over = { 9, 0, 0 };
    chassis_twist_t nan_in = { 0.0 / 0.0, 0.0, 0.0 };
    chassis_wheel_cmd_t cmd, cmd2;
    chassis_config_default(&cfg);

    CHECK(chassis_twist_to_raw(&fwd, &cfg, &cmd) == CH_OK, "kin: 直行不报错");
    CHECK(eq4i(cmd.raw, 300, 300, -300, -300),
          "kin: 台架档直行 = 2026-10-09 四轮自检下发的 +300/+300/-300/-300");

    CHECK(chassis_speed_preset(&cfg, 0.5, CH_DEFAULT_ACCEL_MPS2, &p05, NULL) == CH_OK, "kin: 0.5 档合法");
    chassis_twist_to_raw(&strafe, &p05, &cmd);
    CHECK(eq4i(cmd.raw, -7162, 7162, -7162, 7162), "kin: 纯左移对角同号");
    chassis_twist_to_raw(&spin, &p05, &cmd);
    CHECK(eq4i(cmd.raw, -7162, 7162, 7162, -7162), "kin: 原地逆时针左右反号");
    chassis_twist_to_raw(&back, &p05, &cmd);
    CHECK(eq4i(cmd.raw, -7162, -7162, 7162, 7162), "kin: 纯后退整体反号");
    chassis_twist_to_raw(&none, &p05, &cmd);
    CHECK(eq4i(cmd.raw, 0, 0, 0, 0), "kin: 零输入四轮零");

    chassis_twist_to_raw(&over, &p05, &cmd);
    chassis_twist_to_raw(&fwd, &p05, &cmd2);
    CHECK(memcmp(cmd.raw, cmd2.raw, sizeof(cmd.raw)) == 0, "kin: vx=9 与 vx=1 同结果（先夹到 ±1）");
    chassis_twist_to_raw(&nan_in, &p05, &cmd);
    CHECK(eq4i(cmd.raw, 0, 0, 0, 0), "kin: NaN 当 0");

    double cap_in[4] = { 1, -2, 3, -4 };
    chassis_scaled4_t sc = chassis_scale_to_cap(cap_in, 10);
    CHECK(!sc.scaled && sc.values[0] == 1 && sc.values[3] == -4, "kin: 峰值不超时原样返回");

    double v_full = chassis_full_scale_speed_mps(&cfg);
    CHECK(near(v_full, 0.020943951, 1e-8), "kin: 满杆 300 raw ≈ 0.0209 m/s（Ø0.20、30:1）");

    for (double m = 0.5; m < 2.1; m += 0.5) {
        CHECK(near(chassis_speed_mps_from_raw(chassis_raw_for_speed_mps(m, &cfg), &cfg), m, 1e-9),
              "kin: m/s ↔ raw 往返一致");
    }
    CHECK((int)chassis_js_round(chassis_raw_for_speed_mps(1.0, &cfg)) == 14324,
          "kin: 1.0 m/s @30:1/Ø0.20 → 14324 raw");

    chassis_twist_t t = { 0.3, -0.1, 0.05 }, rt;
    double mps[4];
    chassis_twist_to_wheel_mps(&t, &cfg.mech, mps);
    chassis_wheel_to_twist(mps, &cfg.mech, &rt);
    CHECK(near(rt.vx, t.vx, 1e-9) && near(rt.vy, t.vy, 1e-9) && near(rt.wz, t.wz, 1e-9),
          "kin: 正逆运动学往返一致");

    chassis_twist_t combos[3] = { { 1, 1, 1 }, { 1, -1, 0.5 }, { 0.2, 0.9, -1 } };
    bool within = true, all_scaled = true;
    for (int i = 0; i < 3; i++) {
        chassis_twist_to_raw(&combos[i], &p05, &cmd);
        for (int k = 0; k < CHASSIS_WHEELS; k++) {
            if (abs(cmd.raw[k]) > p05.control.max_cmd_raw) within = false;
        }
        if (!cmd.scaled) all_scaled = false;
    }
    CHECK(within, "kin: 任何组合都不越过 maxCmdRaw");
    CHECK(all_scaled, "kin: 三个组合都应触发等比缩放（保方向，不逐路削顶）");

    char errs[8][96];
    chassis_config_t dup = cfg;
    dup.wheels[1].slave = cfg.wheels[0].slave;
    dup.wheels[1].channel = cfg.wheels[0].channel;      /* 同一路 + 同一站号 = 两个从站同时应答 */
    CHECK(chassis_validate_config(&dup, errs, 8) >= 1, "cfg: 同路同站号要拦");
    chassis_config_t multi = cfg;                        /* 一条总线挂两台：合法接法 */
    multi.wheels[1].channel = cfg.wheels[0].channel;
    multi.wheels[3].channel = cfg.wheels[2].channel;
    CHECK(chassis_validate_config(&multi, errs, 8) == 0, "cfg: 共用通道但站号不同 → 放行");
    chassis_config_t reuse = cfg;                        /* 两条总线各用站号 1/4 也不冲突 */
    reuse.wheels[2].slave = cfg.wheels[0].slave;
    reuse.wheels[3].slave = cfg.wheels[1].slave;
    CHECK(chassis_validate_config(&reuse, errs, 8) == 0, "cfg: 不同路重用站号 → 放行");
    chassis_config_t badsign = cfg;
    badsign.wheels[2].sign = 0;
    CHECK(chassis_validate_config(&badsign, errs, 8) >= 1, "cfg: sign 不是 ±1 要拦");
    chassis_config_t nowatch = cfg;
    nowatch.control.input_timeout_ms = 0;
    CHECK(chassis_validate_config(&nowatch, errs, 8) >= 1, "cfg: 看门狗被配成 0 要拦");
}

/* ---------- 写入节流 ---------- */

static void test_limits(void)
{
    chassis_config_t cfg;
    chassis_config_default(&cfg);
    chassis_control_t *C = &cfg.control;
    chassis_write_t w[4]; int nw; int df[4]; int nd;
    int q;

    CHECK(chassis_quantize(103, 10, false, 0, &q) == CH_OK && q == 100, "lim: 103→100");
    CHECK(chassis_quantize(106, 10, false, 0, &q) == CH_OK && q == 110, "lim: 106→110");
    CHECK(chassis_quantize(-104, 10, false, 0, &q) == CH_OK && q == -100, "lim: -104→-100");
    CHECK(chassis_quantize(4, 10, false, 0, &q) == CH_OK && q == 0, "lim: 小于半档的抖动被吃掉");
    CHECK(chassis_quantize(75, 10, false, 0, &q) == CH_OK && q == 80, "lim: 不带 cap 时四舍五入确实抬高");
    CHECK(chassis_quantize(75, 10, true, 75, &q) == CH_OK && q == 70, "lim: 越 cap → 向下退一整档，不贴 cap");
    CHECK(chassis_quantize(-75, 10, true, 75, &q) == CH_OK && q == -70, "lim: 负方向同口径");
    CHECK(chassis_quantize(74, 10, true, 75, &q) == CH_OK && q == 70, "lim: cap 内正常量化与退档同档 → 抖半档零写入");
    CHECK(chassis_quantize(106, 10, true, 100, &q) == CH_OK && q == 100, "lim: 退到 cap 以下最大整档");
    CHECK(chassis_quantize(103, 10, true, 100, &q) == CH_OK && q == 100, "lim: cap 内不受影响");
    CHECK(chassis_quantize(20, 10, true, 5, &q) == CH_OK && q == 5, "lim: 一档都塞不进才贴 cap");
    CHECK(chassis_quantize(10, 10, true, 0, &q) == CH_ERR_CAP, "lim: cap=0 非法");
    CHECK(chassis_quantize(10, 0, true, 100, &q) == CH_ERR_STEP, "lim: step<1 非法");
    CHECK(chassis_quantize(0.0 / 0.0, 10, false, 0, &q) == CH_ERR_NOT_FINITE, "lim: NaN 拒绝");

    CHECK(chassis_ramp_toward(0, 300, 40) == 40, "lim: 斜坡上行每拍 40");
    CHECK(chassis_ramp_toward(0, -300, 40) == -40, "lim: 斜坡下行每拍 -40");
    CHECK(chassis_ramp_toward(280, 300, 40) == 300, "lim: 最后一步不越界");

    /* 整条链路收敛在 70（≤ 限幅 75），贴档之后零写入 */
    chassis_throttle_t st;
    chassis_throttle_init(&st);
    int t75[4] = { 75, 75, -75, -75 };
    chassis_control_t c75 = *C; c75.max_cmd_raw = 75;
    for (int i = 0; i < 100; i++) {
        chassis_plan_writes(&st, t75, (uint32_t)(i * 50), &c75, w, &nw, df, &nd);
    }
    CHECK(eq4i(st.written, 70, 70, -70, -70), "lim: 收敛在 70，不会停在 80");
    int before = st.counts.writes;
    chassis_plan_writes(&st, t75, 5000, &c75, w, &nw, df, &nd);
    CHECK(st.counts.writes == before && nw == 0, "lim: 同一个输入永远同一个值 → 稳态零写入");

    /* 与线上值相同 → 一个字节都不发 */
    chassis_throttle_init(&st);
    int tgt[4] = { 100, 100, -100, -100 };
    for (int i = 0; i < 200; i++) {
        chassis_plan_writes(&st, tgt, (uint32_t)(i * 50), C, w, &nw, df, &nd);
    }
    CHECK(eq4i(st.written, 100, 100, -100, -100), "lim: 斜坡最终落到量化目标");
    chassis_plan_writes(&st, tgt, 5000, C, w, &nw, df, &nd);
    CHECK(nw == 0, "lim: 稳态行驶必须零写入（EEPROM 保护主路径）");

    /* 非零写入受限频 */
    int tgt2[4] = { 200, 200, -200, -200 };
    chassis_throttle_init(&st);
    chassis_plan_writes(&st, tgt, 1000, C, w, &nw, df, &nd);
    chassis_plan_writes(&st, tgt2, (uint32_t)(1000 + C->min_write_interval_ms - 50), C, w, &nw, df, &nd);
    CHECK(nw == 0 && nd == 4, "lim: 未到间隔 → 四路全推迟");
    chassis_plan_writes(&st, tgt2, (uint32_t)(1000 + C->min_write_interval_ms), C, w, &nw, df, &nd);
    CHECK(nw == 4, "lim: 到点就补上");
    CHECK(st.counts.deferred == 4, "lim: 推迟要计数（上层靠它判断是否写不过来）");
    CHECK(st.counts.writes == 8, "lim: 计数只统计真正落线的次数");

    /* 毫秒计数跑满约 49.7 天会回绕：限频要按"真实经过的时间"判断。
     * 用有符号差值时 now 变小 → 差值是巨大负数 → 永远"还没到间隔"，表现是只能停不能动。 */
    chassis_throttle_init(&st);
    chassis_plan_writes(&st, tgt, 0xFFFFFFFFu, C, w, &nw, df, &nd);
    CHECK(nw == 4, "lim: 回绕前一刻正常写入");
    chassis_plan_writes(&st, tgt2, (uint32_t)C->min_write_interval_ms - 140u, C, w, &nw, df, &nd);
    CHECK(nw == 0 && nd == 4, "lim: 回绕后真实只过 110ms → 仍然推迟（不是放水）");
    chassis_plan_writes(&st, tgt2, (uint32_t)C->min_write_interval_ms + 10u, C, w, &nw, df, &nd);
    CHECK(nw == 4, "lim: 回绕后到点必须补得上");

    /* 归零不受限频与斜坡约束 */
    int full[4] = { 300, 300, -300, -300 };
    chassis_throttle_init(&st);
    for (int i = 0; i < 500; i++) {
        chassis_plan_writes(&st, full, (uint32_t)(i * 50), C, w, &nw, df, &nd);
    }
    CHECK(eq4i(st.written, 300, 300, -300, -300), "lim: 先爬到满速才有急停可测");
    uint32_t tight = st.last_write_at[0] + 1;
    chassis_plan_writes(&st, (int[4]){ 0, 0, 0, 0 }, tight, C, w, &nw, df, &nd);
    CHECK(nw == 4 && nd == 0, "lim: 紧接上一次写入也要立刻归零");
    bool all_zero = true;
    for (int i = 0; i < nw; i++) if (w[i].value != 0) all_zero = false;
    CHECK(all_zero, "lim: 一步到 0，不被斜坡拆成 260/220");

    chassis_throttle_init(&st);
    for (int i = 0; i < 500; i++) {
        chassis_plan_writes(&st, full, (uint32_t)(i * 50), C, w, &nw, df, &nd);
    }
    chassis_plan_writes(&st, (int[4]){ 2, 2, -2, -2 }, tight, C, w, &nw, df, &nd);
    all_zero = true;
    for (int i = 0; i < nw; i++) if (w[i].value != 0) all_zero = false;
    CHECK(nw == 4 && all_zero, "lim: 回中抖动也立刻停，不留 260");

    /* plan_stop_all 只写非零的那几路 */
    chassis_throttle_init(&st);
    st.written[0] = 100; st.written[2] = -50;
    int n = chassis_plan_stop_all(&st, w);
    CHECK(n == 2 && w[0].idx == 0 && w[1].idx == 2, "lim: stop_all 只挑非零路");
    CHECK(eq4i(st.written, 0, 0, 0, 0), "lim: stop_all 之后全零");
    CHECK(chassis_plan_stop_all(&st, w) == 0, "lim: 已经全零 → 不再写");

    /* 长时间恒定输入 → 写入次数是常数级，不随时间线性增长 */
    chassis_throttle_init(&st);
    int per_tick[2000];
    for (int i = 0; i < 2000; i++) {
        int b = st.counts.writes;
        chassis_plan_writes(&st, full, (uint32_t)(i * 50), C, w, &nw, df, &nd);
        per_tick[i] = st.counts.writes - b;
    }
    CHECK(st.counts.writes == 32, "lim: 爬坡写入 = 4 轮 × ceil(300/40) = 32 笔，之后不再增长");
    int tail = 0;
    for (int i = 200; i < 2000; i++) tail += per_tick[i];
    CHECK(tail == 0, "lim: 稳态的 90 秒一字节都不发");

    /* 开机第一帧不被限频吃掉（lastWriteAt 初值坑） */
    chassis_throttle_init(&st);
    chassis_plan_writes(&st, (int[4]){ 50, 50, -50, -50 }, 0, C, w, &nw, df, &nd);
    CHECK(nw == 4, "lim: t=0 就能下发");

    /* 发帧失败 → 退回线上值，下一拍自动重发 */
    chassis_throttle_init(&st);
    chassis_plan_writes(&st, full, 0, C, w, &nw, df, &nd);
    int first_value = w[0].value;
    chassis_rollback_writes(&st, w, 1);
    CHECK(st.written[0] == 0, "lim: 回滚把认为的线上值退回 0");
    chassis_plan_writes(&st, full, 1000, C, w, &nw, df, &nd);
    CHECK(nw > 0 && w[0].value == first_value, "lim: 回滚后下一拍重发同一笔");
    CHECK(st.counts.failed == 1, "lim: 失败要计数");
}

/* ---------- 车速档 ---------- */

static void test_preset(void)
{
    chassis_config_t cfg, p;
    chassis_bench_t b;
    chassis_config_default(&cfg);
    int errs_n;
    char errs[8][96];

    int rr, ceil_;
    double rms;
    CHECK(chassis_rated_raw(&cfg, &rr) == CH_OK && rr == 15000, "pre: 额定 = 15000 raw（Fn-1C 3000 rpm）");
    CHECK(chassis_rated_speed_mps(&cfg, &rms) == CH_OK && near(rms, 1.047198, 0.002), "pre: 额定折算 1.047 m/s 是硬天花板");
    CHECK(chassis_ceiling_for_rated(&cfg, &ceil_) == CH_OK && ceil_ == 15000, "pre: Pn-30D 建议值 = 额定，不多写一档");
    CHECK(ceil_ <= CH_PN30D_MANUAL_MAX, "pre: 不越手册上限 30000");

    CHECK(chassis_speed_preset(&cfg, 0.5, CH_DEFAULT_ACCEL_MPS2, &p, &b) == CH_OK, "pre: 0.5 档合法");
    CHECK(p.control.max_cmd_raw == 7162, "pre: 0.5 m/s → 7162 raw");
    CHECK(p.control.quantize_raw == 70, "pre: 量化档 70");
    CHECK(p.control.ramp_raw_per_tick == 1791, "pre: 斜坡 1791");
    CHECK(near(b.time_to_full_s, 1.0, 1e-9), "pre: 0.5/0.8<1s → 取窗口下限，不许一帧跳满");
    CHECK(b.writes_to_full == 4, "pre: 实测 1/4→4/4 每档一次到位，约 1 秒爬满");
    CHECK(b.rated_raw == 15000, "pre: bench 带额定 raw");
    int g;
    CHECK(chassis_suggested_start_gear(&p, 4, &g) == CH_OK && g == 1, "pre: 0.5 m/s 从最低档起步");
    errs_n = chassis_validate_config(&p, errs, 8);
    CHECK(errs_n == 0, "pre: 默认档自检通过");

    double gear_mps[4];
    for (int i = 0; i < 4; i++) {
        int raw = (int)chassis_js_round((double)p.control.max_cmd_raw * (i + 1) / 4.0), q;
        CHECK(chassis_quantize((double)raw, p.control.quantize_raw, true, p.control.max_cmd_raw, &q) == CH_OK,
              "pre: 档位量化合法");
        gear_mps[i] = chassis_speed_mps_from_raw((double)q, &p);
    }
    CHECK(near(gear_mps[0], 0.127, 5e-4) && near(gear_mps[1], 0.249, 5e-4) &&
          near(gear_mps[2], 0.376, 5e-4) && near(gear_mps[3], 0.498, 5e-4), "pre: 四档 0.127/0.249/0.376/0.498");
    CHECK(gear_mps[3] < rms, "pre: 默认档满杆仍在额定折算上限之下");

    chassis_twist_t fwd = { 1, 0, 0 };
    chassis_wheel_cmd_t cmd;
    chassis_twist_to_raw(&fwd, &p, &cmd);
    int wire[4];
    for (int i = 0; i < 4; i++) {
        chassis_quantize((double)cmd.raw[i], p.control.quantize_raw, true, p.control.max_cmd_raw, &wire[i]);
    }
    CHECK(eq4i(wire, 7140, 7140, -7140, -7140), "pre: 4/4 前进线上值 = 实机读回的 7140/-7140");
    bool forward = true;
    for (int i = 0; i < 4; i++) if (wire[i] * p.wheels[i].sign <= 0) forward = false;
    CHECK(forward, "pre: 前进档四轮都向前滚（符号矩阵不随档位变）");

    CHECK(chassis_speed_preset(&cfg, 1.0, CH_DEFAULT_ACCEL_MPS2, &p, &b) == CH_OK, "pre: 1.0 档");
    CHECK(p.control.max_cmd_raw == 14324 && p.control.quantize_raw == 145, "pre: 14324 raw / 量化档 145");
    CHECK(p.control.ramp_raw_per_tick == 2865, "pre: 斜坡 = ceil(14324/5)，摊到写入次数而不是拍数");
    CHECK(near(b.time_to_full_s, 1.25, 1e-9) && b.writes_to_full == 5, "pre: 1.0/0.8 = 1.25 s ÷ 250 ms = 5 次");

    chassis_throttle_t st;
    chassis_throttle_init(&st);
    int tgt[4] = { p.control.max_cmd_raw, p.control.max_cmd_raw,
                   -p.control.max_cmd_raw, -p.control.max_cmd_raw };
    chassis_write_t w[4]; int nw, df[4], nd;
    for (int i = 0; i < 200; i++) chassis_plan_writes(&st, tgt, (uint32_t)(i * 50), &p.control, w, &nw, df, &nd);
    CHECK(eq4i(st.written, 14210, 14210, -14210, -14210), "pre: 14324 打不上去，退档落在 14210");
    CHECK(near(chassis_speed_mps_from_raw(14210, &p), 0.992, 0.002), "pre: 14210 ≈ 0.992 m/s");
    CHECK(st.counts.writes <= 24, "pre: 四轮各写几次就到位");

    CHECK(chassis_speed_preset(&cfg, 1.3, CH_DEFAULT_ACCEL_MPS2, &p, &b) == CH_ERR_OVER_RATED,
          "pre: 1.3 m/s 超额定 → 拒绝，软件不给绕");
    CHECK(chassis_speed_preset(&cfg, 0, CH_DEFAULT_ACCEL_MPS2, &p, &b) == CH_ERR_MPS, "pre: mps=0 拒绝");
    CHECK(chassis_speed_preset(&cfg, 0.0 / 0.0, CH_DEFAULT_ACCEL_MPS2, &p, &b) == CH_ERR_MPS, "pre: mps=NaN 拒绝");
    CHECK(chassis_speed_preset(&cfg, 1.0, 0, &p, &b) == CH_ERR_ACCEL, "pre: accel=0 拒绝");
    CHECK(chassis_suggested_start_gear(&cfg, 0, &g) == CH_ERR_GEARS, "pre: gears=0 拒绝");

    chassis_config_t slower = cfg; slower.mech.gear_ratio = 20.0;
    CHECK(chassis_speed_preset(&slower, 1.3, CH_DEFAULT_ACCEL_MPS2, &p, &b) == CH_OK &&
          p.control.max_cmd_raw == 12414, "pre: 1.3 m/s @20:1 → 12414 raw（天花板跟着实测值走）");
    chassis_config_t steeper = cfg; steeper.mech.gear_ratio = 40.0;
    CHECK(chassis_speed_preset(&steeper, 1.3, CH_DEFAULT_ACCEL_MPS2, &p, &b) == CH_ERR_OVER_RATED,
          "pre: @40:1 时同一个 1.3 又超额定了");

    CHECK(chassis_speed_preset(&cfg, 0.02, CH_DEFAULT_ACCEL_MPS2, &p, &b) == CH_OK, "pre: 0.02 m/s 档");
    CHECK(p.control.quantize_raw == cfg.control.quantize_raw, "pre: 1% 满量程比原档小时保留原档");
    CHECK(p.control.ramp_raw_per_tick >= cfg.control.ramp_raw_per_tick, "pre: 小档斜坡只许更稳不许更猛");
    CHECK(near(b.time_to_full_s, 1.0, 1e-9), "pre: 爬升时间有下限窗口");
    CHECK(p.control.max_cmd_raw < 300, "pre: 0.02 m/s 比台架档还慢");

    CHECK(chassis_speed_preset(&cfg, 1.0, 0.4, &p, &b) == CH_OK && b.writes_to_full == 10,
          "pre: 加速度调小 → 2.5 s ÷ 250 ms = 10 次写入");
    chassis_config_t fast;
    CHECK(chassis_speed_preset(&cfg, 1.0, CH_DEFAULT_ACCEL_MPS2, &fast, &b) == CH_OK, "pre: 参照档");
    chassis_config_t slow;
    CHECK(chassis_speed_preset(&cfg, 1.0, 0.4, &slow, &b) == CH_OK, "pre: 缓加速档");
    CHECK(slow.control.ramp_raw_per_tick < fast.control.ramp_raw_per_tick, "pre: 缓加速的斜坡一定更小");

    chassis_config_t over = cfg;
    over.control.max_cmd_raw = 16000;
    over.expect.max_ceiling_raw = 20000;
    CHECK(chassis_validate_config(&over, errs, 8) == 2, "pre: 超额定 + 越额定 raw 两条都要报");
    CHECK(chassis_validate_config(&cfg, errs, 8) == 0, "cfg: 出厂默认自检通过");

    CHECK(cfg.control.max_cmd_raw == 300, "pre: 台架档 300 raw 保留（四轮自检那次的锚点）");
    CHECK(chassis_suggested_start_gear(&cfg, 4, &g) == CH_OK && g == 2, "pre: 慢速台架仍 2/4 起步，老手感不变");
    CHECK(chassis_suggested_start_gear(&cfg, 1, &g) == CH_OK && g == 1, "pre: 只有一档时不许返回 2");
    CHECK(chassis_speed_preset(&cfg, 0.3, CH_DEFAULT_ACCEL_MPS2, &p, &b) == CH_OK &&
          chassis_suggested_start_gear(&p, 4, &g) == CH_OK && g == 2, "pre: 刚好 0.3 m/s 不算快");
    for (double m = 0.4; m <= 1.01; m += 0.1) {
        chassis_speed_preset(&cfg, m, CH_DEFAULT_ACCEL_MPS2, &p, &b);
        chassis_suggested_start_gear(&p, 4, &g);
        CHECK(g == 1, "pre: 0.4~1.0 m/s 都必须从最低档起步");
    }
    int errs_dummy; (void)errs_dummy;
}

/* ---------- Modbus 帧 ---------- */

static void test_modbus(void)
{
    uint8_t f[8];
    ch_mb_write_request(f, 1, 0x0303, 300);
    const uint8_t want_pos[8] = { 0x01, 0x06, 0x03, 0x03, 0x01, 0x2C, 0x79, 0xC3 };
    CHECK(memcmp(f, want_pos, 8) == 0, "mb: 写 1 号 +300 与实机 PowerShell 脚本逐字节一致");

    ch_mb_write_request(f, 2, 0x0303, -7140);
    const uint8_t want_neg[8] = { 0x02, 0x06, 0x03, 0x03, 0xE4, 0x1C, 0x33, 0x74 };
    CHECK(memcmp(f, want_neg, 8) == 0, "mb: 负值转补码一致");

    ch_mb_read_request(f, 1, 0x0700, 24);
    const uint8_t want_rd[8] = { 0x01, 0x03, 0x07, 0x00, 0x00, 0x18, 0x44, 0xB4 };
    CHECK(memcmp(f, want_rd, 8) == 0, "mb: 读监控组 0x0700×24 一致");

    CHECK(ch_mb_write_request(f, 1, 0x0303, 70000) == CH_ERR_CAP, "mb: 超 16 位拒绝");

    ch_mb_write_request(f, 1, 0x0303, 100);
    uint8_t echo[8];
    memcpy(echo, f, 8);
    CHECK(ch_mb_parse_write_response(echo, 8, f).ok, "mb: 回显判成功");
    echo[7] ^= 0xFF;
    CHECK(!ch_mb_parse_write_response(echo, 8, f).ok, "mb: CRC 被破坏要报");
    echo[7] ^= 0xFF;
    echo[4] ^= 0x01;
    CHECK(!ch_mb_parse_write_response(echo, 8, f).ok, "mb: 回显不符要报");

    /* 异常帧只有 5 字节，必须比长度先认出来（当年四台全报 short-response 就是这么丢了异常码） */
    uint8_t ex[5] = { 0x01, 0x86, 0x03, 0x00, 0x00 };
    ch_mb_result_t r = ch_mb_parse_write_response(ex, 5, f);
    CHECK(!r.ok && r.exception == 3, "mb: 0x86 帧认出异常码 3，而不是压成 short-response");
    CHECK(strstr(ch_mb_exception_text(6), "Sd-On") != NULL, "mb: 异常码 6 = 运行中不能写，先关使能");
    CHECK(ch_mb_parse_read_response(ex, 5, 1, 0x0706, 2, (int16_t[2]){ 0, 0 }).exception == 3,
          "mb: 读也一样先认异常");

    uint8_t rd[9];
    rd[0] = 1; rd[1] = 3; rd[2] = 4;
    rd[3] = 0x01; rd[4] = 0x64;      /* 356 */
    rd[5] = 0xFF; rd[6] = 0x38;      /* -200 */
    uint16_t crc = ch_crc16_modbus(rd, 7);
    rd[7] = (uint8_t)(crc & 0xFF);
    rd[8] = (uint8_t)(crc >> 8);
    int16_t regs[2];
    r = ch_mb_parse_read_response(rd, 9, 1, 0x0706, 2, regs);
    CHECK(r.ok && regs[0] == 356 && regs[1] == -200, "mb: 读响应解析 + 有符号还原");
    crc = ch_crc16_modbus(rd, 7) ^ 0x100;
    rd[8] = (uint8_t)(crc >> 8);
    CHECK(!ch_mb_parse_read_response(rd, 9, 1, 0x0706, 2, regs).ok, "mb: CRC 不符要报");
    rd[8] = (uint8_t)((ch_crc16_modbus(rd, 7) ^ 0x100) >> 8);
    rd[8] = (uint8_t)(ch_crc16_modbus(rd, 7) >> 8);
    CHECK(!ch_mb_parse_read_response(rd, 9, 2, 0x0706, 2, regs).ok, "mb: 站号不符要报");
    r = ch_mb_parse_read_response(NULL, 0, 1, 0x0706, 2, regs);
    CHECK(!r.ok && strcmp(r.error, "no-response") == 0, "mb: 超时是一等公民，不能当成功");
    rd[2] = 6;
    r = ch_mb_parse_read_response(rd, 9, 1, 0x0706, 2, regs);
    CHECK(!r.ok && strcmp(r.error, "byte-count") == 0, "mb: 字节数不符要报");
    rd[2] = 4;
    r = ch_mb_parse_read_response(rd, 8, 1, 0x0706, 2, regs);
    CHECK(!r.ok && strcmp(r.error, "truncated") == 0, "mb: 半帧要报");

    char note[64];
    ch_mb_frame_note(rd, 0, note, sizeof(note));
    CHECK(strstr(note, "无响应") != NULL, "mb: 无响应给中文说明");
    ch_mb_frame_note(f, 8, note, sizeof(note));
    CHECK(strstr(note, "0106") != NULL, "mb: 十六进制帧能原样打出来");
}

/* ---------- 传输层（假串口：脚本化字节流 + 帧间静默） ---------- */

typedef struct {
    const uint8_t *bytes;
    int n;
    int at;
    int gap_at;      /* 在这个下标处先制造一次帧间静默，模拟真总线上两帧之间的 3.5 字符 */
    uint32_t t;
    int dir_tx;      /* 方向脚被切到发送态的次数 */
} fake_uart_t;

static void f_dir(void *ctx, bool tx) { fake_uart_t *f = ctx; if (tx) f->dir_tx++; }
static int f_write(void *ctx, const uint8_t *buf, int n) { (void)ctx; (void)buf; return n; }

static int f_read(void *ctx, uint8_t *out, int timeout_ms)
{
    fake_uart_t *f = ctx;
    if (f->at == f->gap_at) {              /* 帧间静默：这一帧说完了 */
        f->gap_at = -1;
        f->t += (uint32_t)timeout_ms;
        return 0;
    }
    if (f->at >= f->n) {                   /* 没有字节了 = 到时限 */
        f->t += (uint32_t)timeout_ms;
        return 0;
    }
    *out = f->bytes[f->at++];
    f->t += 2;                             /* 9600 8-N-2 一个字节 ≈ 1.15 ms */
    return 1;
}

static uint32_t f_now(void *ctx) { return ((fake_uart_t *)ctx)->t; }

static chassis_uart_t make_fake_uart(fake_uart_t *f)
{
    chassis_uart_t u;
    memset(&u, 0, sizeof(u));
    u.ctx = f;
    u.set_direction_tx = f_dir;
    u.write_bytes = f_write;
    u.read_byte = f_read;
    u.now_ms = f_now;
    return u;
}

static void test_transport(void)
{
    uint8_t foreign[CH_MB_REQUEST_LEN], ours[CH_MB_REQUEST_LEN];
    uint8_t stream[2 * CH_MB_REQUEST_LEN];
    uint8_t rsp[64];
    int len = -1;

    /* 一条总线挂两台：上一笔问的是 4 号，回执迟了一拍、压在这一笔里回来，
     * 紧接着才是我们要的 1 号回执。按从站号认帧，别把 4 号的应答当成 1 号的。 */
    ch_mb_write_request(foreign, 4, CH_REG_SPEED, 70);
    ch_mb_write_request(ours, 1, CH_REG_SPEED, 70);
    memcpy(stream, foreign, CH_MB_REQUEST_LEN);
    memcpy(stream + CH_MB_REQUEST_LEN, ours, CH_MB_REQUEST_LEN);

    fake_uart_t f1 = { stream, (int)sizeof(stream), 0, CH_MB_REQUEST_LEN, 0, 0 };
    chassis_uart_t u1 = make_fake_uart(&f1);
    chassis_err_t e = chassis_modbus_transfer(&u1, ours, CH_MB_REQUEST_LEN, rsp,
                                              (int)sizeof(rsp), 100, &len);
    CHECK(e == CH_OK && len == CH_MB_REQUEST_LEN && rsp[0] == 1,
          "xfer: 别人家的迟到回帧被丢掉，仍然拿到自己这一笔");
    CHECK(f1.dir_tx == 1, "xfer: 一笔传输只切一次发送方向");

    /* 总线上只有别家的应答 → 一个字节都不该算成我们的，上层据此记一次通讯失败 */
    fake_uart_t f2 = { stream, CH_MB_REQUEST_LEN, 0, -1, 0, 0 };
    chassis_uart_t u2 = make_fake_uart(&f2);
    len = -1;
    e = chassis_modbus_transfer(&u2, ours, CH_MB_REQUEST_LEN, rsp, (int)sizeof(rsp), 100, &len);
    CHECK(e == CH_OK && len == 0, "xfer: 只有别家应答 → 按无响应交回上层");

    /* 正常一笔：1 号自己的写回执，认出功能码后收到 8 字节就停，不白等静默 */
    fake_uart_t f3 = { ours, CH_MB_REQUEST_LEN, 0, -1, 0, 0 };
    chassis_uart_t u3 = make_fake_uart(&f3);
    len = -1;
    e = chassis_modbus_transfer(&u3, ours, CH_MB_REQUEST_LEN, rsp, (int)sizeof(rsp), 100, &len);
    CHECK(e == CH_OK && len == 8, "xfer: 写回执收满 8 字节即止");
    CHECK(ch_mb_parse_write_response(rsp, len, ours).ok, "xfer: 收到的那帧能被自己解析通过");
}

/* ---------- 安全层 ---------- */
static void test_safety(void)
{
    chassis_config_t cfg;
    chassis_config_default(&cfg);
    chassis_control_t *C = &cfg.control;
    chassis_safety_t st;
    chassis_decision_t d;

    chassis_safety_init(&st);
    d = chassis_safety_decide(&st, 1000, C);
    CHECK(d.force_zero && d.reason == CH_REASON_WAIT_INPUT, "saf: 没输入 → 停（默认停）");

    chassis_safety_note_input(&st, 1000);
    d = chassis_safety_decide(&st, 1100, C);
    CHECK(!d.force_zero && d.state == CH_STATE_RUN, "saf: 输入新鲜 → run");

    d = chassis_safety_decide(&st, (uint32_t)(1000 + C->input_timeout_ms + 1), C);
    CHECK(d.force_zero && d.reason == CH_REASON_INPUT_TIMEOUT, "saf: 超时 → 停（看门狗）");
    CHECK(near((double)st.input_age_ms, 401.0, 1.0), "saf: 超时时长可日志（判据不是肉眼）");

    chassis_safety_latch_estop(&st, CH_REASON_MANUAL_ESTOP);
    d = chassis_safety_decide(&st, 2000, C);
    CHECK(d.force_zero && st.state == CH_STATE_ESTOP, "saf: 急停闭锁");
    chassis_safety_note_input(&st, 2010);
    CHECK(chassis_safety_decide(&st, 2010, C).force_zero, "saf: 闭锁期间喂输入也不动（不自动解锁）");
    CHECK(chassis_safety_clear_latch(&st), "saf: 显式解除");
    CHECK(st.state == CH_STATE_IDLE && st.reason == CH_REASON_CLEARED, "saf: 解除后等输入");
    CHECK(!chassis_safety_clear_latch(&st), "saf: 没闭锁时解除是空操作");

    chassis_safety_init(&st);
    chassis_safety_note_input(&st, 0);
    for (int i = 0; i < C->comm_fail_limit; i++) chassis_safety_note_comm(&st, 0, false);
    d = chassis_safety_decide(&st, 10, C);
    CHECK(d.force_zero && st.state == CH_STATE_FAULT, "saf: 连续 3 次通讯失败 → 故障闭锁");
    chassis_safety_note_comm(&st, 20, true);
    CHECK(st.fail_streak == 0, "saf: 一次成功就清零");

    chassis_safety_init(&st);
    chassis_safety_note_input(&st, 5000);
    CHECK(chassis_safety_decide(&st, 100, C).force_zero, "saf: 时钟回跳按超时处理，宁停不误");
}

/* ---------- 控制环（假总线） ---------- */

typedef struct {
    int writes;
    int reads;
    int fail_writes;
    int fail_reads;
    int last_value[4];
    int zero_writes;
} fake_bus_t;

static int fake_write(void *ctx, uint8_t channel, uint8_t slave, uint16_t reg, int16_t value)
{
    fake_bus_t *b = (fake_bus_t *)ctx;
    (void)reg;
    b->writes++;
    if (b->fail_writes) return -1;
    b->last_value[channel] = value;
    if (value == 0) b->zero_writes++;
    if (slave == 0) return -2;
    return 0;
}

static int fake_read(void *ctx, uint8_t channel, uint8_t slave, uint16_t reg,
                     uint16_t qty, int16_t *out)
{
    fake_bus_t *b = (fake_bus_t *)ctx;
    (void)channel; (void)reg;
    b->reads++;
    if (b->fail_reads) return -1;
    for (int i = 0; i < (int)qty; i++) out[i] = (i == (CH_MON_SPD - CH_REG_MONITOR)) ? 356 : 0;
    return slave == 0 ? -2 : 0;
}

typedef struct { chassis_event_code_t last; int aux; int n; } ev_log_t;

static void ev_cb(void *ctx, chassis_event_code_t code, int aux)
{
    ev_log_t *l = (ev_log_t *)ctx;
    l->last = code;
    l->aux = aux;
    l->n++;
}

static void test_controller(void)
{
    chassis_config_t cfg;
    chassis_config_default(&cfg);
    chassis_bus_t bus;
    fake_bus_t fb;
    ev_log_t ev;
    chassis_ctrl_t c;
    chassis_step_result_t r;

    memset(&bus, 0, sizeof(bus));
    bus.write_reg = fake_write;
    bus.read_block = fake_read;
    chassis_twist_t fwd = { 1, 0, 0 };

    memset(&fb, 0, sizeof(fb));
    bus.ctx = &fb;
    memset(&ev, 0, sizeof(ev));
    chassis_ctrl_init(&c, &cfg, &bus);
    c.on_event = ev_cb;
    c.event_ctx = &ev;

    chassis_ctrl_step(&c, 0, &r);
    CHECK(r.decision.force_zero && r.n_writes == 0 && fb.writes == 0, "ctl: 开机没输入 → 总线上一个指令都没有");
    CHECK(c.force_zero, "ctl: 默认停");

    /* 前进：符号矩阵落到四轮 */
    uint32_t t = 100;
    chassis_ctrl_set_twist(&c, &fwd, t);
    for (int i = 0; i < 20; i++) { chassis_ctrl_step(&c, t, &r); t += 50; chassis_ctrl_set_twist(&c, &fwd, t); }
    CHECK(fb.writes > 0, "ctl: 有输入才开始下发");
    CHECK(fb.last_value[0] > 0 && fb.last_value[1] > 0, "ctl: 前轮写正");
    CHECK(fb.last_value[2] < 0 && fb.last_value[3] < 0, "ctl: 后轮写负（实机标定的符号矩阵）");
    CHECK(fb.zero_writes == 0, "ctl: 前进途中不该有归零");

    /* 看门狗：停止喂输入 → 立刻四轮归零，不受限频 */
    t += 500;
    chassis_ctrl_step(&c, t, &r);
    CHECK(r.decision.force_zero && r.n_writes == 4, "ctl: 输入超时 → 四路一步归零");
    CHECK(fb.zero_writes == 4 && eq4i(c.throttle.written, 0, 0, 0, 0), "ctl: 归零真的落线，线上值回 0");

    /* 稳态：爬坡 32 笔之后一字节都不发，靠只读心跳发现断线 */
    int w_before = fb.writes, rd_before = fb.reads;
    for (int i = 0; i < 100; i++) {
        t += 50;
        chassis_ctrl_set_twist(&c, &fwd, t);
        chassis_ctrl_step(&c, t, &r);
    }
    CHECK(fb.writes - w_before == 32, "ctl: 20 Hz 连跑 5 秒只写 32 笔（EEPROM 硬约束）");
    CHECK(fb.reads > rd_before, "ctl: 稳态没写入时靠只读心跳发现链路断");
    CHECK(c.monitor_valid[0] && c.monitor[0][CH_MON_SPD - CH_REG_MONITOR] == 356,
          "ctl: 驱动器自报转速入库（判转没转靠它，不靠肉眼）");
    int16_t spd;
    CHECK(chassis_ctrl_drive_spd(&c, 0, &spd) && spd == 356, "ctl: driveSpd 取得到");

    /* 满杆不该越过限幅 */
    CHECK(abs(c.last_target[0]) <= cfg.control.max_cmd_raw, "ctl: 目标不越 maxCmdRaw");

    /* 急停闭锁：满杆输入也只出 0 */
    chassis_ctrl_estop(&c, CH_REASON_MANUAL_ESTOP);
    t += 50;
    chassis_ctrl_set_twist(&c, &fwd, t);
    chassis_ctrl_step(&c, t, &r);
    CHECK(r.decision.force_zero && eq4i(r.target, 0, 0, 0, 0), "ctl: 急停期间满杆也只出 0");
    CHECK(ev.last == CH_EV_LATCHED, "ctl: 闭锁发事件给上层");
    CHECK(chassis_ctrl_release(&c), "ctl: 显式解除");

    /* 死链路 → 故障闭锁（这台驱动器断了线不会自己停） */
    memset(&fb, 0, sizeof(fb));
    fb.fail_writes = 1;
    fb.fail_reads = 1;
    memset(&ev, 0, sizeof(ev));
    chassis_ctrl_init(&c, &cfg, &bus);
    c.on_event = ev_cb;
    c.event_ctx = &ev;
    for (int i = 0; i < 8; i++) {
        uint32_t ts = (uint32_t)(i * 1000);
        chassis_ctrl_set_twist(&c, &fwd, ts);
        chassis_ctrl_step(&c, ts, &r);
    }
    CHECK(c.safety.state == CH_STATE_FAULT, "ctl: 连续通讯失败 → 故障闭锁");
    CHECK(eq4i(r.target, 0, 0, 0, 0), "ctl: 闭锁后强制归零");

    /* 写失败 → 退回线上值，不能算已生效 */
    memset(&fb, 0, sizeof(fb));
    fb.fail_writes = 1;
    memset(&ev, 0, sizeof(ev));
    chassis_ctrl_init(&c, &cfg, &bus);
    c.on_event = ev_cb;
    c.event_ctx = &ev;
    chassis_ctrl_set_twist(&c, &fwd, 0);
    chassis_ctrl_step(&c, 0, &r);
    CHECK(c.throttle.counts.writes == 0 && c.throttle.written[0] == 0,
          "ctl: 写失败退回，节流器不会以为命令已生效");
    CHECK(c.throttle.counts.failed == 4 && ev.last == CH_EV_WRITE_FAIL && ev.aux == 4, "ctl: 写失败计数并上报");

    /* 干跑：没有总线时只算不发 */
    chassis_ctrl_t dry;
    chassis_ctrl_init(&dry, &cfg, NULL);
    chassis_ctrl_set_twist(&dry, &fwd, 0);
    chassis_ctrl_step(&dry, 0, &r);
    CHECK(r.n_writes > 0 && dry.throttle.counts.writes > 0, "ctl: 干跑照样算目标并记账（台架前先看一遍）");
    CHECK(dry.throttle.written[0] > 0, "ctl: 干跑的线上值是算出来的，不是发出去的");

    /* 退出清理：绕过节流四轮归零 */
    memset(&fb, 0, sizeof(fb));
    chassis_ctrl_init(&c, &cfg, &bus);
    for (uint32_t i = 0; i < 10; i++) {
        chassis_ctrl_set_twist(&c, &fwd, i * 50);
        chassis_ctrl_step(&c, i * 50, &r);
    }
    CHECK(chassis_ctrl_stop_all(&c) == 0 && eq4i(c.throttle.written, 0, 0, 0, 0), "ctl: 退出清理四轮归零");
    CHECK(fb.zero_writes == 4, "ctl: 清理帧真的发出去了");

    /* 归零也发不出去 → 必须报警，这是"可能停在半速"的唯一线索 */
    memset(&ev, 0, sizeof(ev));
    chassis_ctrl_set_twist(&c, &fwd, 5000);
    chassis_ctrl_step(&c, 5000, &r);
    chassis_ctrl_set_twist(&c, &fwd, 5050);
    chassis_ctrl_step(&c, 5050, &r);
    fb.fail_writes = 1;
    memset(&ev, 0, sizeof(ev));
    CHECK(chassis_ctrl_stop_all(&c) > 0, "ctl: 写 0 失败要返回路数");
    CHECK(ev.last == CH_EV_STOP_FAIL, "ctl: 写 0 失败必须上报 → 用物理总开关");

    /* 输入层与设备无关：手柄/键盘/网页都只调 set_twist，喂同一个值就该同一个结果 */
    chassis_ctrl_t a, b2;
    chassis_ctrl_init(&a, &cfg, NULL);
    chassis_ctrl_init(&b2, &cfg, NULL);
    chassis_twist_t diag = { 0.5, -0.25, 0.1 };
    chassis_ctrl_set_twist(&a, &diag, 0);
    chassis_ctrl_set_twist(&b2, &diag, 0);
    chassis_ctrl_step(&a, 0, &r);
    chassis_step_result_t r2;
    chassis_ctrl_step(&b2, 0, &r2);
    CHECK(memcmp(r.target, r2.target, sizeof(r.target)) == 0, "ctl: 同一输入两个实例结果一致");
}

int main(void)
{
    test_kinematics();
    test_limits();
    test_preset();
    test_modbus();
    test_transport();
    test_safety();
    test_controller();

    printf("\n%d checks, %d failed\n", g_checks, g_fail);
    return g_fail == 0 ? 0 : 1;
}
