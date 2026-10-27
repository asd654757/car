#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_WHEELS 4

/* 轮序固定 FL FR BL BR，与 JS 侧 CONFIG.wheels 下标一致 */
typedef enum { CH_W_FL = 0, CH_W_FR = 1, CH_W_BL = 2, CH_W_BR = 3 } chassis_wheel_idx_t;

/* 核心层的错误码。JS 侧这些情况是 throw，固件里不能靠异常，一律往上返回 */
typedef enum {
    CH_OK = 0,
    CH_ERR_NOT_FINITE = 1,   /* NaN / Inf 进了决策路径 */
    CH_ERR_STEP,             /* 量化档非法 */
    CH_ERR_CAP,              /* 限幅非法 */
    CH_ERR_MPS,              /* 车速不是正有限数 */
    CH_ERR_ACCEL,            /* 加速度不是正有限数 */
    CH_ERR_CONFIG,           /* 机械/控制参数非法（本该被 chassis_validate_config 拦下） */
    CH_ERR_OVER_RATED,       /* 目标超过额定折算上限：改机械才能做的事，软件不绕 */
    CH_ERR_GEARS,            /* 档数非法 */
    CH_ERR_NO_NAMEPLATE,     /* 额定转速没标定，不许据此提速 */
    CH_ERR_BAD_PARAM         /* 除零一类的入参错误 */
} chassis_err_t;

/* Modbus / 驱动器层的量纲，2026-10-09 实机确认 */
#define CH_RAW_PER_RPM        5        /* Pn-3xx 单位 0.2 转/分，电机侧 */
#define CH_REG_SPEED          0x0303   /* Pn-303 内部预设速度 3 = 实机命令源 */
#define CH_REG_MODE           0x0000   /* Pn-000 */
#define CH_REG_CEILING        0x030D   /* Pn-30D 速度上限 */
#define CH_REG_MONITOR        0x0700
#define CH_MONITOR_QTY        24       /* 0x0700~0x0717，跨组读返回异常 0x04 */
#define CH_REG_NAMEPLATE      0x0619   /* Fn-19 起 */
#define CH_NAMEPLATE_QTY      8
#define CH_PN30D_MANUAL_MAX   30000

/* 监控组里真要看的那些，全是电机侧读数 */
#define CH_MON_SPD            0x0706
#define CH_MON_CMD            0x0707
#define CH_MON_TORQUE         0x0709
#define CH_MON_POS            0x0714
#define CH_MON_RATED_SPD      0x061C   /* Fn-1C 额定转速，物理上限 */

typedef struct {
    double gear_ratio;        /* 电机转 / 轮子转，粗测 30:1，误差 ±10%，待量准 */
    double wheel_diameter_m;  /* 口述 0.20 m，未上卡尺 */
    double lx_plus_ly_m;      /* 半宽+半轴距，待实测 */
} chassis_mech_t;

typedef struct {
    int loop_hz;
    int max_cmd_raw;
    int quantize_raw;
    int min_write_interval_ms;
    int input_timeout_ms;
    int ramp_raw_per_tick;
    int comm_fail_limit;
    int comm_probe_ms;
} chassis_control_t;

typedef struct {
    int rated_spd_rpm;        /* Fn-1C 实机读回 3000，四台一致 */
    int rated_volt_v;
    int rated_cur_a;
    int poles;
} chassis_nameplate_t;

typedef struct {
    int max_ceiling_raw;      /* 预检允许的 Pn-30D 上限 */
    int spd_idle_tolerance_rpm;
    int rated_spd_plausible_min;
    int rated_spd_plausible_max;
} chassis_expect_t;

typedef struct {
    char id[4];               /* "FL" 等 */
    uint8_t slave;            /* Modbus 从站号 */
    int8_t sign;              /* +1 = 写正数该轮向前滚 */
    uint8_t channel;          /* 板上第几路 RS485，由 port 层解释，核心不认引脚 */
} chassis_wheel_t;

typedef struct {
    double raw_per_rpm;
    chassis_mech_t mech;
    chassis_control_t control;
    chassis_nameplate_t nameplate;
    chassis_expect_t expect;
    chassis_wheel_t wheels[CHASSIS_WHEELS];
} chassis_config_t;

/* 不带 --mps 时的默认车速档，与 src/config.js 的 DEFAULT_TARGET_MPS 同值。
 * 改这里等于重新做一次安全确认。 */
#define CH_DEFAULT_TARGET_MPS 0.5

void chassis_config_default(chassis_config_t *cfg);

/* 返回错误条数，0 = 通过。把自相矛盾的配置在上线前拦下来 */
int chassis_validate_config(const chassis_config_t *cfg,
                            char errs[][96], int max_errs);

#endif /* CHASSIS_CONFIG_H */
