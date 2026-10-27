#ifndef CHASSIS_PRESET_H
#define CHASSIS_PRESET_H

#include "chassis_config.h"

/* 车速档 → 控制参数：只允许按 m/s 提要求，raw 一律从机械参数派生并被铭牌额定转速硬夹住。
 * 让人手填 raw 就是 10 倍速事故的概率来源。 */

#define CH_DEFAULT_ACCEL_MPS2   0.8
#define CH_SLOW_START_MPS       0.3
#define CH_RAMP_WINDOW_MIN_S    1.0
#define CH_RAMP_WINDOW_MAX_S    4.0

typedef struct {
    double mps;
    double accel_mps2;
    double rated_mps;
    int rated_raw;
    double time_to_full_s;
    int writes_to_full;
    double gate_s;
} chassis_bench_t;

chassis_err_t chassis_rated_raw(const chassis_config_t *cfg, int *out);
chassis_err_t chassis_rated_speed_mps(const chassis_config_t *cfg, double *out);
chassis_err_t chassis_ceiling_for_rated(const chassis_config_t *cfg, int *out);

/* 界面进来该给 1/4 还是 2/4：满杆高于 SLOW_START_MPS 就从最低档起步 */
chassis_err_t chassis_suggested_start_gear(const chassis_config_t *cfg, int gears, int *out);

chassis_err_t chassis_speed_preset(const chassis_config_t *in, double mps, double accel_mps2,
                                   chassis_config_t *out, chassis_bench_t *bench);

const char *chassis_err_message(chassis_err_t e);

#endif /* CHASSIS_PRESET_H */
