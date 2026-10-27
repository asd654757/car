#ifndef CHASSIS_LIMITS_H
#define CHASSIS_LIMITS_H

#include <stdbool.h>
#include <stdint.h>

#include "chassis_config.h"

/* 一次落线的写入。value 已乐观记进 state.written，发帧失败必须 rollback，
 * 否则节流器以为命令已生效就永远不再重试（这台驱动器断了线不会自己停） */
typedef struct {
    int idx;
    int value;
    int prev;
} chassis_write_t;

typedef struct {
    int writes;
    int deferred;
    int stops;
    int failed;
} chassis_write_counts_t;

typedef struct {
    int written[CHASSIS_WHEELS];              /* 当前认为在线上的值；初值 0，预检会核实 */
    uint32_t last_write_at[CHASSIS_WHEELS];
    bool has_last_write[CHASSIS_WHEELS];      /* false = 从没写过，等价 JS 的 -Infinity */
    chassis_write_counts_t counts;
} chassis_throttle_t;

void chassis_throttle_init(chassis_throttle_t *st);

/* 量化。返回 0 成功，负数 = 参数非法（宁可不写也不许按非法值写进驱动器） */
int chassis_quantize(double v, int step, bool has_cap, int cap, int *out);

/* 斜坡：把 next 相对 current 的变化限制在 max_delta 内 */
int chassis_ramp_toward(int current, int next, int max_delta);

/* 决定这一拍写哪几路。返回 0 成功 */
int chassis_plan_writes(chassis_throttle_t *st, const int target[CHASSIS_WHEELS],
                        uint32_t now_ms, const chassis_control_t *ctl,
                        chassis_write_t out_writes[CHASSIS_WHEELS], int *n_writes,
                        int out_deferred[CHASSIS_WHEELS], int *n_deferred);

void chassis_rollback_writes(chassis_throttle_t *st, const chassis_write_t *failed, int n);

/* 强制四轮归零，绕过量化/斜坡/限频（急停与退出清理走这条）。返回条数 */
int chassis_plan_stop_all(chassis_throttle_t *st, chassis_write_t out[CHASSIS_WHEELS]);

#endif /* CHASSIS_LIMITS_H */
