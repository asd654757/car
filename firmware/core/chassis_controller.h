#ifndef CHASSIS_CONTROLLER_H
#define CHASSIS_CONTROLLER_H

#include "chassis_config.h"
#include "chassis_kinematics.h"
#include "chassis_limits.h"
#include "chassis_safety.h"

/* 输入层（手柄 / 遥控接收机 / 键盘）只负责调 set_twist，不知道有串口；
 * 传输层只负责一帧一帧地收发，不知道有运动学。核心夹在中间，两边都可换。 */

typedef struct chassis_bus {
    void *ctx;
    /* 返回 0 = 成功。非 0 会进通讯失败计数，连续 comm_fail_limit 次即故障闭锁 */
    int (*write_reg)(void *ctx, uint8_t channel, uint8_t slave, uint16_t reg, int16_t value);
    int (*read_block)(void *ctx, uint8_t channel, uint8_t slave, uint16_t reg,
                      uint16_t qty, int16_t *out);
} chassis_bus_t;

typedef enum {
    CH_EV_NONE = 0,
    CH_EV_WRITE_FAIL,       /* aux = 失败路数 */
    CH_EV_NAN_TARGET,       /* 运动学输出非法，本拍强制归零 */
    CH_EV_PROBE_FAIL,       /* 只读心跳失败 */
    CH_EV_DEFER_STALL,      /* aux = 连续多少拍发不出新指令 */
    CH_EV_STOP_FAIL,        /* aux = 写 0 失败路数（最危险：可能停在半速） */
    CH_EV_LATCHED           /* 进入急停/故障闭锁 */
} chassis_event_code_t;

typedef struct {
    int target[CHASSIS_WHEELS];
    chassis_write_t writes[CHASSIS_WHEELS];
    int n_writes;
    int deferred[CHASSIS_WHEELS];
    int n_deferred;
    int n_failed;
    bool scaled;
    bool probed;
    chassis_decision_t decision;
} chassis_step_result_t;

typedef struct {
    chassis_config_t cfg;
    const chassis_bus_t *bus;      /* NULL = 干跑：只算不发，用来在台架前先给人看一遍 */
    void (*on_event)(void *ctx, chassis_event_code_t code, int aux);
    void *event_ctx;

    chassis_throttle_t throttle;
    chassis_safety_t safety;
    chassis_twist_t twist;
    int last_target[CHASSIS_WHEELS];
    bool force_zero;
    int defer_streak;
    uint32_t last_defer_log_at;
    bool has_defer_log;
    uint32_t last_probe_at;
    bool has_probe;

    int16_t monitor[CHASSIS_WHEELS][CH_MONITOR_QTY];   /* 驱动器自报，用来对撞"我们以为发了" */
    bool monitor_valid[CHASSIS_WHEELS];
    uint32_t tick_count;
} chassis_ctrl_t;

void chassis_ctrl_init(chassis_ctrl_t *c, const chassis_config_t *cfg, const chassis_bus_t *bus);

void chassis_ctrl_set_twist(chassis_ctrl_t *c, const chassis_twist_t *t, uint32_t now_ms);
void chassis_ctrl_zero_twist(chassis_ctrl_t *c, uint32_t now_ms);
void chassis_ctrl_estop(chassis_ctrl_t *c, chassis_reason_t reason);
bool chassis_ctrl_release(chassis_ctrl_t *c);

/* 一拍：安全裁决 → 运动学 → 写入节流 → 总线 */
chassis_err_t chassis_ctrl_step(chassis_ctrl_t *c, uint32_t now_ms, chassis_step_result_t *res);

/* 强制四轮归零，绕过节流（退出 / 急停用）。返回写 0 失败的路数，0 = 全落线 */
int chassis_ctrl_stop_all(chassis_ctrl_t *c);

/* 驱动器自报转速（电机侧 rpm）与命令，用于"转没转"的判据 */
bool chassis_ctrl_drive_spd(const chassis_ctrl_t *c, int idx, int16_t *out);
bool chassis_ctrl_drive_cmd(const chassis_ctrl_t *c, int idx, int16_t *out);

#endif /* CHASSIS_CONTROLLER_H */
