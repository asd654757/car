#include "chassis_limits.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "chassis_kinematics.h"   /* chassis_js_round */

static int sign_of(double v) { return (v > 0.0) - (v < 0.0); }

void chassis_throttle_init(chassis_throttle_t *st)
{
    memset(st, 0, sizeof(*st));
    for (int i = 0; i < CHASSIS_WHEELS; i++) st->has_last_write[i] = false;
}

int chassis_quantize(double v, int step, bool has_cap, int cap, int *out)
{
    if (!isfinite(v)) return CH_ERR_NOT_FINITE;
    if (step < 1) return CH_ERR_STEP;
    if (has_cap && !(cap > 0)) return CH_ERR_CAP;

    double q = chassis_js_round(step <= 1 ? v : v / (double)step) * (double)step;

    if (!has_cap) { *out = (int)q; return 0; }
    if (fabs(q) <= (double)cap) { *out = (int)q; return 0; }

    /* 四舍五入只许往下不许往上：maxCmdRaw 75 时 round(75/10)*10 = 80，硬顶被量化悄悄抬高。
     * 越界时向下退一整档而不是返回 cap：74 本来就量化成 70，返回 75 会让输入在 74/75 抖动
     * 产生 70↔75 的反复写入 —— 那正是要防的 EEPROM 磨损。 */
    double by_step = (double)sign_of(v) * floor((double)cap / (double)step) * (double)step;
    if (isfinite(by_step) && by_step != 0.0) { *out = (int)by_step; return 0; }
    *out = sign_of(v) * cap;      /* 一档都塞不进 cap，才贴 cap —— 宁可慢也不越限幅 */
    return 0;
}

int chassis_ramp_toward(int current, int next, int max_delta)
{
    int d = next - current;
    if (abs(d) <= max_delta) return next;
    return current + ((d > 0) - (d < 0)) * max_delta;
}

int chassis_plan_writes(chassis_throttle_t *st, const int target[CHASSIS_WHEELS],
                        uint32_t now_ms, const chassis_control_t *ctl,
                        chassis_write_t out_writes[CHASSIS_WHEELS], int *n_writes,
                        int out_deferred[CHASSIS_WHEELS], int *n_deferred)
{
    int nw = 0, nd = 0;

    *n_writes = 0;
    *n_deferred = 0;

    for (int i = 0; i < CHASSIS_WHEELS; i++) {
        int q;
        int rc = chassis_quantize((double)target[i], ctl->quantize_raw, true,
                                  ctl->max_cmd_raw, &q);
        if (rc != CH_OK) return rc;

        /* 规则 1 优先于斜坡：目标就是停 → 立刻写 0，不许被 ramp 拖成 260、220…… */
        if (q == 0) {
            if (st->written[i] == 0) continue;
            out_writes[nw].idx = i;
            out_writes[nw].value = 0;
            out_writes[nw].prev = st->written[i];
            nw++;
            st->written[i] = 0;
            st->last_write_at[i] = now_ms;
            st->has_last_write[i] = true;
            st->counts.writes++;
            st->counts.stops++;
            continue;
        }

        int limited = ctl->ramp_raw_per_tick > 0
            ? chassis_ramp_toward(st->written[i], q, ctl->ramp_raw_per_tick)
            : q;
        if (limited == st->written[i]) continue;      /* 与线上一致 → 一个字节都不发 */

        /* 只有非零写入受限频约束 */
        if (st->has_last_write[i]) {
            /* 必须用无符号差值：毫秒计数跑满约 49.7 天会回绕，有符号差会变成巨大负数，
             * 于是"还没到间隔"永远成立 → 四轮的非零写入被限频永久拦住（归零不受拦，所以看着像"只能停不能动"）。
             * 无符号回绕差只在跨回绕的那一瞬间成立，这与 chassis_controller.c 里的时间门同一口径。 */
            uint32_t age = now_ms - st->last_write_at[i];
            if (age < (uint32_t)ctl->min_write_interval_ms) {
                out_deferred[nd++] = i;
                st->counts.deferred++;
                continue;
            }
        }

        out_writes[nw].idx = i;
        out_writes[nw].value = limited;
        out_writes[nw].prev = st->written[i];
        nw++;
        st->written[i] = limited;
        st->last_write_at[i] = now_ms;
        st->has_last_write[i] = true;
        st->counts.writes++;
    }

    *n_writes = nw;
    *n_deferred = nd;
    return 0;
}

/* 发帧失败 → 把"线上值"退回原值，下一拍自动重试。归零失败尤其要退，否则悄悄停在半速 */
void chassis_rollback_writes(chassis_throttle_t *st, const chassis_write_t *failed, int n)
{
    for (int k = 0; k < n; k++) {
        int i = failed[k].idx;
        if (i < 0 || i >= CHASSIS_WHEELS) continue;
        st->written[i] = failed[k].prev;
        if (st->counts.writes > 0) st->counts.writes--;
        st->counts.failed++;
    }
}

int chassis_plan_stop_all(chassis_throttle_t *st, chassis_write_t out[CHASSIS_WHEELS])
{
    int n = 0;
    for (int i = 0; i < CHASSIS_WHEELS; i++) {
        if (st->written[i] == 0) continue;
        out[n].idx = i;
        out[n].value = 0;
        out[n].prev = st->written[i];
        n++;
        st->written[i] = 0;
    }
    st->counts.stops += n;
    st->counts.writes += n;
    return n;
}
