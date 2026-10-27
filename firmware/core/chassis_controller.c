#include "chassis_controller.h"

#include <string.h>

#define CH_MON_OFF_SPD (CH_MON_SPD - CH_REG_MONITOR)
#define CH_MON_OFF_CMD (CH_MON_CMD - CH_REG_MONITOR)

static void emit(chassis_ctrl_t *c, chassis_event_code_t code, int aux)
{
    if (c->on_event) c->on_event(c->event_ctx, code, aux);
}

static void set_zero(int target[CHASSIS_WHEELS])
{
    for (int i = 0; i < CHASSIS_WHEELS; i++) target[i] = 0;
}

void chassis_ctrl_init(chassis_ctrl_t *c, const chassis_config_t *cfg, const chassis_bus_t *bus)
{
    memset(c, 0, sizeof(*c));
    c->cfg = *cfg;
    c->bus = bus;
    chassis_throttle_init(&c->throttle);
    chassis_safety_init(&c->safety);
    set_zero(c->last_target);
    c->force_zero = true;          /* 默认停：没跑过一拍之前不许有"我还在动"的印象 */
    c->has_defer_log = false;
    c->has_probe = false;          /* 下一拍就发心跳 */
}

void chassis_ctrl_set_twist(chassis_ctrl_t *c, const chassis_twist_t *t, uint32_t now_ms)
{
    c->twist.vx = chassis_clamp_unit(t->vx);
    c->twist.vy = chassis_clamp_unit(t->vy);
    c->twist.wz = chassis_clamp_unit(t->wz);
    chassis_safety_note_input(&c->safety, now_ms);
}

void chassis_ctrl_zero_twist(chassis_ctrl_t *c, uint32_t now_ms)
{
    chassis_twist_t z = { 0.0, 0.0, 0.0 };
    chassis_ctrl_set_twist(c, &z, now_ms);
}

void chassis_ctrl_estop(chassis_ctrl_t *c, chassis_reason_t reason)
{
    chassis_safety_latch_estop(&c->safety, reason);
    emit(c, CH_EV_LATCHED, (int)reason);
}

bool chassis_ctrl_release(chassis_ctrl_t *c)
{
    return chassis_safety_clear_latch(&c->safety);
}

/* 发一轮写入，返回失败路数；失败的那些路退回"线上值"，下一拍自动重发 */
static int send_writes(chassis_ctrl_t *c, const chassis_write_t *writes, int n)
{
    if (n <= 0 || c->bus == NULL || c->bus->write_reg == NULL) return 0;

    chassis_write_t failed[CHASSIS_WHEELS];
    int nf = 0;
    for (int k = 0; k < n; k++) {
        const chassis_wheel_t *w = &c->cfg.wheels[writes[k].idx];
        int rc = c->bus->write_reg(c->bus->ctx, w->channel, w->slave, CH_REG_SPEED,
                                   (int16_t)writes[k].value);
        if (rc != 0) failed[nf++] = writes[k];
    }
    if (nf > 0) chassis_rollback_writes(&c->throttle, failed, nf);
    return nf;
}

static bool probe_all(chassis_ctrl_t *c, uint32_t now_ms)
{
    bool all_ok = true;
    for (int i = 0; i < CHASSIS_WHEELS; i++) {
        const chassis_wheel_t *w = &c->cfg.wheels[i];
        int rc = c->bus->read_block(c->bus->ctx, w->channel, w->slave,
                                   CH_REG_MONITOR, CH_MONITOR_QTY, c->monitor[i]);
        if (rc == 0) {
            c->monitor_valid[i] = true;
        } else {
            all_ok = false;
        }
    }
    chassis_safety_note_comm(&c->safety, now_ms, all_ok);
    return all_ok;
}

chassis_err_t chassis_ctrl_step(chassis_ctrl_t *c, uint32_t now_ms, chassis_step_result_t *res)
{
    chassis_step_result_t r;
    memset(&r, 0, sizeof(r));

    c->tick_count++;

    r.decision = chassis_safety_decide(&c->safety, now_ms, &c->cfg.control);
    c->force_zero = r.decision.force_zero;

    if (r.decision.force_zero) {
        set_zero(r.target);
    } else {
        chassis_wheel_cmd_t cmd;
        chassis_err_t e = chassis_twist_to_raw(&c->twist, &c->cfg, &cmd);
        if (e != CH_OK) {
            set_zero(r.target);
            emit(c, CH_EV_NAN_TARGET, (int)e);
        } else {
            memcpy(r.target, cmd.raw, sizeof(r.target));
            r.scaled = cmd.scaled;
        }
    }
    memcpy(c->last_target, r.target, sizeof(c->last_target));

    chassis_err_t e = chassis_plan_writes(&c->throttle, r.target, now_ms, &c->cfg.control,
                                          r.writes, &r.n_writes, r.deferred, &r.n_deferred);
    if (e != CH_OK) return e;

    if (r.n_writes > 0) c->defer_streak = 0;
    if (r.n_deferred > 0) {
        /* 斜坡期"这一拍没写"是设计内的常态（限频 250ms / 周期 50ms），
         * 所以只有连续 2 秒一个字节都发不出去才算异常，且 5 秒最多提示一次 */
        c->defer_streak++;
        bool due = !c->has_defer_log ||
                   (uint32_t)(now_ms - c->last_defer_log_at) > 5000u;
        if (c->defer_streak >= c->cfg.control.loop_hz * 2 && due) {
            c->last_defer_log_at = now_ms;
            c->has_defer_log = true;
            emit(c, CH_EV_DEFER_STALL, c->defer_streak);
        }
    } else if (r.n_writes == 0) {
        c->defer_streak = 0;
    }

    if (r.n_writes > 0) {
        r.n_failed = send_writes(c, r.writes, r.n_writes);
        chassis_safety_note_comm(&c->safety, now_ms, r.n_failed == 0);
        if (r.n_failed > 0) emit(c, CH_EV_WRITE_FAIL, r.n_failed);
    } else if (c->bus != NULL && c->bus->read_block != NULL) {
        /* 只读心跳：稳态行驶一帧都不发，"发帧失败"这条路永远走不到，
         * 不主动读一次就永远不知道链路已经断了 —— 而这台驱动器断了线不会自己停 */
        bool due = !c->has_probe ||
                   (uint32_t)(now_ms - c->last_probe_at) >= (uint32_t)c->cfg.control.comm_probe_ms;
        if (due) {
            c->last_probe_at = now_ms;
            c->has_probe = true;
            r.probed = true;
            if (!probe_all(c, now_ms)) emit(c, CH_EV_PROBE_FAIL, 0);
        }
    }

    if (res) *res = r;
    return CH_OK;
}

int chassis_ctrl_stop_all(chassis_ctrl_t *c)
{
    chassis_write_t writes[CHASSIS_WHEELS];
    int n = chassis_plan_stop_all(&c->throttle, writes);
    if (n == 0) return 0;
    int nf = send_writes(c, writes, n);
    if (nf > 0) emit(c, CH_EV_STOP_FAIL, nf);
    return nf;
}

bool chassis_ctrl_drive_spd(const chassis_ctrl_t *c, int idx, int16_t *out)
{
    if (idx < 0 || idx >= CHASSIS_WHEELS || !c->monitor_valid[idx]) return false;
    *out = c->monitor[idx][CH_MON_OFF_SPD];
    return true;
}

bool chassis_ctrl_drive_cmd(const chassis_ctrl_t *c, int idx, int16_t *out)
{
    if (idx < 0 || idx >= CHASSIS_WHEELS || !c->monitor_valid[idx]) return false;
    *out = c->monitor[idx][CH_MON_OFF_CMD];
    return true;
}
