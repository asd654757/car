#ifndef CHASSIS_SAFETY_H
#define CHASSIS_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#include "chassis_config.h"

/* 手册里没有任何"通讯超时自动停车"功能：上位一断，驱动器就保持最后一次速度一直转。
 * 所以停车只能本机自己做，而且必须默认停 —— 拿不到明确许可就不许动。 */

typedef enum {
    CH_STATE_IDLE = 0,
    CH_STATE_RUN,
    CH_STATE_ESTOP,
    CH_STATE_FAULT
} chassis_state_t;

typedef enum {
    CH_REASON_NONE = 0,
    CH_REASON_NOT_STARTED,
    CH_REASON_WAIT_INPUT,
    CH_REASON_INPUT_TIMEOUT,
    CH_REASON_COMM_FAIL,
    CH_REASON_MANUAL_ESTOP,
    CH_REASON_EXTERNAL,
    CH_REASON_CLEARED,
    CH_REASON_NORMAL
} chassis_reason_t;

typedef struct {
    chassis_state_t state;
    chassis_reason_t reason;
    uint32_t last_input_at;
    bool has_input;          /* false = 从没收到过输入，等价 JS 的 lastInputAt = -1 */
    uint32_t last_ok_at;
    int fail_streak;
    uint32_t input_age_ms;   /* 只给日志用 */
} chassis_safety_t;

typedef struct {
    bool force_zero;
    chassis_state_t state;
    chassis_reason_t reason;
} chassis_decision_t;

void chassis_safety_init(chassis_safety_t *st);
void chassis_safety_note_input(chassis_safety_t *st, uint32_t now_ms);
void chassis_safety_note_comm(chassis_safety_t *st, uint32_t now_ms, bool ok);
void chassis_safety_latch_estop(chassis_safety_t *st, chassis_reason_t reason);
void chassis_safety_latch_fault(chassis_safety_t *st, chassis_reason_t reason);

/* 只有显式解除才允许恢复；闭锁不自动解锁。返回是否真的解除了 */
bool chassis_safety_clear_latch(chassis_safety_t *st);

bool chassis_safety_is_locked(const chassis_safety_t *st);

chassis_decision_t chassis_safety_decide(chassis_safety_t *st, uint32_t now_ms,
                                         const chassis_control_t *ctl);

const char *chassis_state_name(chassis_state_t s);
const char *chassis_reason_name(chassis_reason_t r);

#endif /* CHASSIS_SAFETY_H */
