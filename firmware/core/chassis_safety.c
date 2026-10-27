#include "chassis_safety.h"

#include <string.h>

void chassis_safety_init(chassis_safety_t *st)
{
    memset(st, 0, sizeof(*st));
    st->state = CH_STATE_IDLE;
    st->reason = CH_REASON_NOT_STARTED;
    st->has_input = false;
}

void chassis_safety_note_input(chassis_safety_t *st, uint32_t now_ms)
{
    st->last_input_at = now_ms;
    st->has_input = true;
}

void chassis_safety_note_comm(chassis_safety_t *st, uint32_t now_ms, bool ok)
{
    if (ok) {
        st->fail_streak = 0;
        st->last_ok_at = now_ms;
    } else {
        st->fail_streak++;
    }
}

void chassis_safety_latch_estop(chassis_safety_t *st, chassis_reason_t reason)
{
    st->state = CH_STATE_ESTOP;
    st->reason = (reason == CH_REASON_NONE) ? CH_REASON_MANUAL_ESTOP : reason;
}

void chassis_safety_latch_fault(chassis_safety_t *st, chassis_reason_t reason)
{
    st->state = CH_STATE_FAULT;
    st->reason = (reason == CH_REASON_NONE) ? CH_REASON_COMM_FAIL : reason;
}

bool chassis_safety_clear_latch(chassis_safety_t *st)
{
    if (st->state == CH_STATE_ESTOP || st->state == CH_STATE_FAULT) {
        st->state = CH_STATE_IDLE;
        st->reason = CH_REASON_CLEARED;      /* 等下一次输入，不自动恢复 */
        return true;
    }
    return false;
}

bool chassis_safety_is_locked(const chassis_safety_t *st)
{
    return st->state == CH_STATE_ESTOP || st->state == CH_STATE_FAULT;
}

chassis_decision_t chassis_safety_decide(chassis_safety_t *st, uint32_t now_ms,
                                         const chassis_control_t *ctl)
{
    chassis_decision_t d;
    d.state = st->state;
    d.reason = st->reason;

    if (st->state == CH_STATE_ESTOP) {
        d.force_zero = true; d.state = st->state; d.reason = st->reason;
        return d;
    }
    if (st->state == CH_STATE_FAULT) {
        d.force_zero = true; d.state = st->state; d.reason = st->reason;
        return d;
    }
    if (ctl->comm_fail_limit > 0 && st->fail_streak >= ctl->comm_fail_limit) {
        chassis_safety_latch_fault(st, CH_REASON_COMM_FAIL);
        d.force_zero = true; d.state = st->state; d.reason = st->reason;
        return d;
    }
    if (!st->has_input) {
        st->state = CH_STATE_IDLE;
        st->reason = CH_REASON_WAIT_INPUT;
        d.force_zero = true; d.state = st->state; d.reason = st->reason;
        return d;
    }

    int64_t age = (int64_t)now_ms - (int64_t)st->last_input_at;
    if (age < 0) age = (int64_t)ctl->input_timeout_ms + 1;   /* 时钟回跳按超时处理，宁停不误 */
    st->input_age_ms = (uint32_t)age;

    if (age > (int64_t)ctl->input_timeout_ms) {
        if (st->state != CH_STATE_IDLE) {
            st->state = CH_STATE_IDLE;
            st->reason = CH_REASON_INPUT_TIMEOUT;
        }
        d.force_zero = true; d.state = st->state; d.reason = st->reason;
        return d;
    }

    st->state = CH_STATE_RUN;
    st->reason = CH_REASON_NORMAL;
    d.force_zero = false; d.state = st->state; d.reason = st->reason;
    return d;
}

const char *chassis_state_name(chassis_state_t s)
{
    switch (s) {
        case CH_STATE_RUN:   return "run";
        case CH_STATE_ESTOP: return "estop";
        case CH_STATE_FAULT: return "fault";
        default:             return "idle";
    }
}

const char *chassis_reason_name(chassis_reason_t r)
{
    switch (r) {
        case CH_REASON_NOT_STARTED:   return "not-started";
        case CH_REASON_WAIT_INPUT:    return "wait-input";
        case CH_REASON_INPUT_TIMEOUT: return "input-timeout";
        case CH_REASON_COMM_FAIL:     return "comm-fail";
        case CH_REASON_MANUAL_ESTOP:  return "manual-estop";
        case CH_REASON_EXTERNAL:      return "external";
        case CH_REASON_CLEARED:       return "cleared";
        case CH_REASON_NORMAL:        return "normal";
        default:                      return "none";
    }
}
