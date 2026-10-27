#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "chassis_config.h"

/* M_PI 不是 C99/C11 标准名（POSIX 才有），Keil 的 math.h 不一定给 —— 自己钉一个 */
#ifndef CHASSIS_PI
#define CHASSIS_PI 3.14159265358979323846
#endif

/* 右手系，俯视：x 朝车头，y 朝车身左，ω 逆时针为正。轮速正 = 该轮向前滚 */
typedef struct { double vx, vy, wz; } chassis_twist_t;

typedef struct {
    double values[CHASSIS_WHEELS];
    bool scaled;
    double factor;
} chassis_scaled4_t;

typedef struct {
    int raw[CHASSIS_WHEELS];       /* 带符号、已限幅、已取整，直接写 Pn-303 */
    double wheel_mps[CHASSIS_WHEELS];
    double v_full;                 /* 满杆车速 m/s */
    bool scaled;
} chassis_wheel_cmd_t;

/* 与 JS Math.round 同口径：.5 向 +∞ 进位。C 的 round() 是远离零，
 * 负半档会差一档（quantize 的向下退档依赖这个口径） */
static inline double chassis_js_round(double x) { return floor(x + 0.5); }

static inline double chassis_clamp_unit(double v)
{
    if (!isfinite(v)) return 0.0;
    if (v < -1.0) return -1.0;
    if (v > 1.0) return 1.0;
    return v;
}

void chassis_twist_to_wheel_mps(const chassis_twist_t *t, const chassis_mech_t *m,
                                double out[CHASSIS_WHEELS]);
void chassis_mps_to_wheel_rpm(const double mps[CHASSIS_WHEELS], const chassis_mech_t *m,
                              double out[CHASSIS_WHEELS]);
void chassis_wheel_rpm_to_motor_raw(const double rpm[CHASSIS_WHEELS], const chassis_mech_t *m,
                                    double raw_per_rpm, double out[CHASSIS_WHEELS]);
void chassis_motor_raw_to_wheel_rpm(const double raw[CHASSIS_WHEELS], const chassis_mech_t *m,
                                    double raw_per_rpm, double out[CHASSIS_WHEELS]);
void chassis_apply_signs(const double in[CHASSIS_WHEELS], const chassis_config_t *cfg,
                         double out[CHASSIS_WHEELS]);
chassis_scaled4_t chassis_scale_to_cap(const double in[CHASSIS_WHEELS], double cap);

double chassis_raw_for_speed_mps(double mps, const chassis_config_t *cfg);
double chassis_speed_mps_from_raw(double raw, const chassis_config_t *cfg);
double chassis_full_scale_speed_mps(const chassis_config_t *cfg);

/* 控制环每拍唯一入口：归一化输入 ∈[-1,1] → 四轮 Pn-303 目标值。返回 0 = 正常 */
int chassis_twist_to_raw(const chassis_twist_t *norm, const chassis_config_t *cfg,
                         chassis_wheel_cmd_t *res);

/* 四轮向前滚速度（同单位）→ 底盘 twist，里程计用 */
void chassis_wheel_to_twist(const double w[CHASSIS_WHEELS], const chassis_mech_t *m,
                            chassis_twist_t *out);

#endif /* CHASSIS_KINEMATICS_H */
