#include "chassis_kinematics.h"

void chassis_twist_to_wheel_mps(const chassis_twist_t *t, const chassis_mech_t *m,
                                double out[CHASSIS_WHEELS])
{
    const double k = m->lx_plus_ly_m;
    out[CH_W_FL] = t->vx - t->vy - k * t->wz;
    out[CH_W_FR] = t->vx + t->vy + k * t->wz;
    out[CH_W_BL] = t->vx + t->vy - k * t->wz;
    out[CH_W_BR] = t->vx - t->vy + k * t->wz;
}

void chassis_mps_to_wheel_rpm(const double mps[CHASSIS_WHEELS], const chassis_mech_t *m,
                              double out[CHASSIS_WHEELS])
{
    const double circ = CHASSIS_PI * m->wheel_diameter_m;
    for (int i = 0; i < CHASSIS_WHEELS; i++) out[i] = mps[i] / circ * 60.0;
}

void chassis_wheel_rpm_to_motor_raw(const double rpm[CHASSIS_WHEELS], const chassis_mech_t *m,
                                    double raw_per_rpm, double out[CHASSIS_WHEELS])
{
    for (int i = 0; i < CHASSIS_WHEELS; i++) out[i] = rpm[i] * m->gear_ratio * raw_per_rpm;
}

void chassis_motor_raw_to_wheel_rpm(const double raw[CHASSIS_WHEELS], const chassis_mech_t *m,
                                    double raw_per_rpm, double out[CHASSIS_WHEELS])
{
    for (int i = 0; i < CHASSIS_WHEELS; i++) out[i] = raw[i] / m->gear_ratio / raw_per_rpm;
}

void chassis_apply_signs(const double in[CHASSIS_WHEELS], const chassis_config_t *cfg,
                         double out[CHASSIS_WHEELS])
{
    for (int i = 0; i < CHASSIS_WHEELS; i++) out[i] = in[i] * cfg->wheels[i].sign;
}

/* 等比缩放到 |每路| ≤ cap：保持运动方向不变，比逐路削顶安全 */
chassis_scaled4_t chassis_scale_to_cap(const double in[CHASSIS_WHEELS], double cap)
{
    chassis_scaled4_t r;
    double peak = 0.0;

    for (int i = 0; i < CHASSIS_WHEELS; i++) {
        double a = fabs(in[i]);
        if (a > peak) peak = a;
    }
    r.scaled = false;
    r.factor = 1.0;
    for (int i = 0; i < CHASSIS_WHEELS; i++) r.values[i] = in[i];

    if (peak == 0.0 || peak <= cap) return r;

    r.scaled = true;
    r.factor = cap / peak;
    for (int i = 0; i < CHASSIS_WHEELS; i++) r.values[i] = in[i] * r.factor;
    return r;
}

double chassis_raw_for_speed_mps(double mps, const chassis_config_t *cfg)
{
    const double circ = CHASSIS_PI * cfg->mech.wheel_diameter_m;
    return mps * 60.0 / circ * cfg->mech.gear_ratio * cfg->raw_per_rpm;
}

double chassis_speed_mps_from_raw(double raw, const chassis_config_t *cfg)
{
    const double circ = CHASSIS_PI * cfg->mech.wheel_diameter_m;
    return raw / cfg->raw_per_rpm / cfg->mech.gear_ratio * circ / 60.0;
}

double chassis_full_scale_speed_mps(const chassis_config_t *cfg)
{
    return chassis_speed_mps_from_raw((double)cfg->control.max_cmd_raw, cfg);
}

/* 控制环每拍调用的唯一入口：归一化输入 → 四轮 Pn-303 目标值 */
int chassis_twist_to_raw(const chassis_twist_t *norm, const chassis_config_t *cfg,
                         chassis_wheel_cmd_t *res)
{
    chassis_twist_t mps_twist;
    double mps[CHASSIS_WHEELS], rpm[CHASSIS_WHEELS], unsigned_raw[CHASSIS_WHEELS];
    chassis_scaled4_t sc;

    if (!(cfg->mech.lx_plus_ly_m > 0.0)) return CH_ERR_CONFIG;   /* 除零保护，正常被 validate 拦在前面 */

    const double v_full = chassis_full_scale_speed_mps(cfg);
    mps_twist.vx = chassis_clamp_unit(norm->vx) * v_full;
    mps_twist.vy = chassis_clamp_unit(norm->vy) * v_full;
    mps_twist.wz = chassis_clamp_unit(norm->wz) * v_full / cfg->mech.lx_plus_ly_m;

    chassis_twist_to_wheel_mps(&mps_twist, &cfg->mech, mps);
    chassis_mps_to_wheel_rpm(mps, &cfg->mech, rpm);
    chassis_wheel_rpm_to_motor_raw(rpm, &cfg->mech, cfg->raw_per_rpm, unsigned_raw);

    sc = chassis_scale_to_cap(unsigned_raw, (double)cfg->control.max_cmd_raw);

    double signed_raw[CHASSIS_WHEELS];
    chassis_apply_signs(sc.values, cfg, signed_raw);

    for (int i = 0; i < CHASSIS_WHEELS; i++) {
        double q = chassis_js_round(signed_raw[i]);
        res->raw[i] = isfinite(q) ? (int)q : 0;
        res->wheel_mps[i] = mps[i];
    }
    res->scaled = sc.scaled;
    res->v_full = v_full;

    for (int i = 0; i < CHASSIS_WHEELS; i++) {
        if (!isfinite(res->wheel_mps[i])) return CH_ERR_NOT_FINITE;
    }
    return CH_OK;
}

void chassis_wheel_to_twist(const double w[CHASSIS_WHEELS], const chassis_mech_t *m,
                            chassis_twist_t *out)
{
    const double k = m->lx_plus_ly_m;
    out->vx = (w[CH_W_FL] + w[CH_W_FR] + w[CH_W_BL] + w[CH_W_BR]) / 4.0;
    out->vy = (-w[CH_W_FL] + w[CH_W_FR] + w[CH_W_BL] - w[CH_W_BR]) / 4.0;
    out->wz = (-w[CH_W_FL] + w[CH_W_FR] - w[CH_W_BL] + w[CH_W_BR]) / (4.0 * k);
}
