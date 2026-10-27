#include "chassis_preset.h"

#include <math.h>
#include <string.h>

#include "chassis_kinematics.h"

chassis_err_t chassis_rated_raw(const chassis_config_t *cfg, int *out)
{
    if (!(cfg->nameplate.rated_spd_rpm > 0)) return CH_ERR_NO_NAMEPLATE;
    double r = (double)cfg->nameplate.rated_spd_rpm * cfg->raw_per_rpm;
    if (!isfinite(r) || r <= 0.0) return CH_ERR_NO_NAMEPLATE;
    *out = (int)floor(r);
    return CH_OK;
}

chassis_err_t chassis_rated_speed_mps(const chassis_config_t *cfg, double *out)
{
    int rr;
    chassis_err_t e = chassis_rated_raw(cfg, &rr);
    if (e != CH_OK) return e;
    *out = chassis_speed_mps_from_raw((double)rr, cfg);
    return CH_OK;
}

/* 建议写进 Pn-30D 的值：正好等于额定，一次写到位，之后所有车速档都在软件侧切 */
chassis_err_t chassis_ceiling_for_rated(const chassis_config_t *cfg, int *out)
{
    int rr;
    chassis_err_t e = chassis_rated_raw(cfg, &rr);
    if (e != CH_OK) return e;
    *out = (rr < CH_PN30D_MANUAL_MAX) ? rr : CH_PN30D_MANUAL_MAX;
    return CH_OK;
}

chassis_err_t chassis_suggested_start_gear(const chassis_config_t *cfg, int gears, int *out)
{
    if (gears < 1) return CH_ERR_GEARS;
    if (chassis_full_scale_speed_mps(cfg) > CH_SLOW_START_MPS) {
        *out = 1;
    } else {
        int g = (gears < 2) ? gears : 2;
        *out = (g < 1) ? 1 : g;
    }
    return CH_OK;
}

chassis_err_t chassis_speed_preset(const chassis_config_t *in, double mps, double accel_mps2,
                                  chassis_config_t *out, chassis_bench_t *bench)
{
    if (!isfinite(mps) || mps <= 0.0) return CH_ERR_MPS;
    if (!isfinite(accel_mps2) || accel_mps2 <= 0.0) return CH_ERR_ACCEL;

    double limit;
    chassis_err_t e = chassis_rated_speed_mps(in, &limit);
    if (e != CH_OK) return e;
    /* 想更快只能改机械，软件不给绕 */
    if (mps > limit) return CH_ERR_OVER_RATED;

    int rated_raw_v;
    e = chassis_rated_raw(in, &rated_raw_v);
    if (e != CH_OK) return e;

    chassis_config_t cfg = *in;

    int max_cmd_raw = (int)chassis_js_round(chassis_raw_for_speed_mps(mps, in));
    if (max_cmd_raw < cfg.control.quantize_raw) max_cmd_raw = cfg.control.quantize_raw;

    /* 稳态量化档 = 满量程的 1%，且不小于原档：低速小档行为完全不变，
     * 高速档不会出现"每拍都在写" */
    int q1 = (int)chassis_js_round((double)max_cmd_raw * 0.01 / 5.0) * 5;
    int quantize_raw = (q1 > cfg.control.quantize_raw) ? q1 : cfg.control.quantize_raw;

    /* 斜坡按"多少秒爬到满杆"算，再除以限频间隔得到允许的写入次数：
     * 被限频推迟的那些拍不会累积，真正决定爬升速度的是写入次数 × 间隔 */
    double gate_s = (double)cfg.control.min_write_interval_ms / 1000.0;
    /* 这里与 JS 侧有意不同：preset.js 允许 minWriteIntervalMs=0（算出 writesToFull=Infinity，
     * 斜坡退回原值）。但 0 = 每拍都写 Pn-303，正是要防的 EEPROM 磨损，固件侧宁可拒绝启动。 */
    if (!(gate_s > 0.0)) return CH_ERR_CONFIG;
    double time_to_full = mps / accel_mps2;
    if (time_to_full < CH_RAMP_WINDOW_MIN_S) time_to_full = CH_RAMP_WINDOW_MIN_S;
    if (time_to_full > CH_RAMP_WINDOW_MAX_S) time_to_full = CH_RAMP_WINDOW_MAX_S;
    int writes_to_full = (int)chassis_js_round(time_to_full / gate_s);
    if (writes_to_full < 1) writes_to_full = 1;
    int ramp = (int)ceil((double)max_cmd_raw / (double)writes_to_full);
    if (ramp < cfg.control.ramp_raw_per_tick) ramp = cfg.control.ramp_raw_per_tick;

    int ceiling;
    e = chassis_ceiling_for_rated(in, &ceiling);
    if (e != CH_OK) return e;

    cfg.control.max_cmd_raw = max_cmd_raw;
    cfg.control.quantize_raw = quantize_raw;
    cfg.control.ramp_raw_per_tick = ramp;
    cfg.expect.max_ceiling_raw = ceiling;

    if (out) *out = cfg;
    if (bench) {
        bench->mps = mps;
        bench->accel_mps2 = accel_mps2;
        bench->rated_mps = limit;
        bench->rated_raw = rated_raw_v;
        bench->time_to_full_s = time_to_full;
        bench->writes_to_full = writes_to_full;
        bench->gate_s = gate_s;
    }
    return CH_OK;
}

const char *chassis_err_message(chassis_err_t e)
{
    switch (e) {
        case CH_OK:               return "ok";
        case CH_ERR_NOT_FINITE:   return "目标值非有限数";
        case CH_ERR_STEP:         return "量化档非法（必须 ≥1）";
        case CH_ERR_CAP:          return "cap 非法（要么不传，要么给正的 maxCmdRaw）";
        case CH_ERR_MPS:          return "车速必须是正有限数";
        case CH_ERR_ACCEL:        return "加速度必须是正有限数";
        case CH_ERR_CONFIG:       return "机械/控制参数非法";
        case CH_ERR_OVER_RATED:   return "目标超过额定折算车速上限 → 只能改机械，软件不绕";
        case CH_ERR_GEARS:        return "gears 必须是正整数";
        case CH_ERR_NO_NAMEPLATE: return "铭牌额定转速没标定 → 不许据此提速";
        case CH_ERR_BAD_PARAM:    return "入参非法";
        default:                  return "未知错误";
    }
}
