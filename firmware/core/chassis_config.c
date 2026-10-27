#include "chassis_config.h"

#include <math.h>
#include <string.h>

static void add_err(char errs[][96], int *n, int max_errs, const char *msg)
{
    if (*n < max_errs) {
        strncpy(errs[*n], msg, 95);
        errs[*n][95] = '\0';
    }
    (*n)++;
}

void chassis_config_default(chassis_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));

    cfg->raw_per_rpm = CH_RAW_PER_RPM;

    /* 三个都还是粗测值：它们错 10%，车速就错 10%。量准之前不许拿车速当真值 */
    cfg->mech.gear_ratio = 30.0;
    cfg->mech.wheel_diameter_m = 0.20;
    cfg->mech.lx_plus_ly_m = 0.25;

    cfg->control.loop_hz = 20;               /* 四口并行实测整机周期 35~45 ms */
    cfg->control.max_cmd_raw = 300;          /* 台架档，只在 --mps 0 等价场景用；正常入场档由 preset 派生 */
    cfg->control.quantize_raw = 10;          /* 2 rpm 一档，护 EEPROM（手册故障 08） */
    cfg->control.min_write_interval_ms = 250;
    cfg->control.input_timeout_ms = 400;
    cfg->control.ramp_raw_per_tick = 40;
    cfg->control.comm_fail_limit = 3;
    cfg->control.comm_probe_ms = 1000;

    cfg->nameplate.rated_spd_rpm = 3000;     /* Fn-1C，2026-10-09 四台一致 */
    cfg->nameplate.rated_volt_v = 34;
    cfg->nameplate.rated_cur_a = 11;
    cfg->nameplate.poles = 8;

    cfg->expect.max_ceiling_raw = 15000;     /* = 额定 3000 rpm 对应值，已写入并回读确认 */
    cfg->expect.spd_idle_tolerance_rpm = 2;  /* 静态抖动/测速分辨率，不是车在动 */
    cfg->expect.rated_spd_plausible_min = 50;
    cfg->expect.rated_spd_plausible_max = 6000;

    /* 符号矩阵 = 2026-10-09 实机标定结果：前 + 后 −。
     * 与 Pn-000 方向位分组、滚轮斜向分组都不一致，别用任何规律外推，改线后必重测。 */
    static const char *ids[CHASSIS_WHEELS] = { "FL", "FR", "BL", "BR" };
    static const uint8_t slaves[CHASSIS_WHEELS] = { 1, 4, 2, 3 };
    static const int8_t signs[CHASSIS_WHEELS] = { +1, +1, -1, -1 };
    for (int i = 0; i < CHASSIS_WHEELS; i++) {
        strcpy(cfg->wheels[i].id, ids[i]);
        cfg->wheels[i].slave = slaves[i];
        cfg->wheels[i].sign = signs[i];
        cfg->wheels[i].channel = (uint8_t)i;   /* 由 port 层按接线图落到具体 UART */
    }
}

int chassis_validate_config(const chassis_config_t *cfg, char errs[][96], int max_errs)
{
    int n = 0;
    double rated_raw;

    if (cfg->nameplate.rated_spd_rpm > 0) {
        rated_raw = (double)cfg->nameplate.rated_spd_rpm * cfg->raw_per_rpm;
    } else {
        rated_raw = 0.0;
    }

    for (int i = 0; i < CHASSIS_WHEELS; i++) {
        const chassis_wheel_t *w = &cfg->wheels[i];
        if (w->sign != 1 && w->sign != -1) {
            add_err(errs, &n, max_errs, "轮 sign 必须是 ±1");
        }
        if (w->slave < 1 || w->slave > 247) {
            add_err(errs, &n, max_errs, "轮 slave 站号非法");
        }
        for (int j = i + 1; j < CHASSIS_WHEELS; j++) {
            /* 同一路上站号必须唯一；不同路可以重用站号（每条总线是各自的 Modbus 网络）。
             * 共用一条 485 挂两台是合法接法，所以这里只禁"同路同站号" —— 那种接法是
             * 一发一收两个从站同时应答，总线直接撞车。 */
            if (cfg->wheels[j].slave == w->slave && cfg->wheels[j].channel == w->channel) {
                add_err(errs, &n, max_errs, "同一路 RS485 上站号重复");
            }
        }
    }

    if (cfg->control.max_cmd_raw > cfg->expect.max_ceiling_raw) {
        add_err(errs, &n, max_errs, "maxCmdRaw 超过驱动器上限 Pn-30D");
    }
    /* 物理闸：Pn-30D 手册能填到 30000（=6000 rpm），那是"能填"不是"能跑" */
    if (rated_raw > 0.0) {
        if ((double)cfg->control.max_cmd_raw > rated_raw) {
            add_err(errs, &n, max_errs, "maxCmdRaw 超过额定转速对应 raw → 超额定，拒绝");
        }
        if ((double)cfg->expect.max_ceiling_raw > fmin(rated_raw, (double)CH_PN30D_MANUAL_MAX)) {
            add_err(errs, &n, max_errs, "允许的 Pn-30D 上限越过额定对应 raw");
        }
    } else {
        add_err(errs, &n, max_errs, "铭牌额定转速没标定 → 不许据此提速");
    }

    if (cfg->control.quantize_raw < 1) {
        add_err(errs, &n, max_errs, "quantizeRaw 必须 ≥1");
    }
    if (cfg->control.max_cmd_raw <= 0) {
        add_err(errs, &n, max_errs, "maxCmdRaw 必须是正数");
    }
    if (cfg->control.loop_hz <= 0 || cfg->control.loop_hz > 30) {
        add_err(errs, &n, max_errs, "loopHz 不合理：整机受串口往返限制，20Hz 已到位");
    }
    if (cfg->control.input_timeout_ms <= 0) {
        add_err(errs, &n, max_errs, "inputTimeoutMs 必须为正 —— 看门狗不许被配成 0");
    }
    if (cfg->control.comm_probe_ms <= 0) {
        add_err(errs, &n, max_errs, "commProbeMs 必须为正 —— 不探测就发现不了断线");
    }
    if (cfg->control.comm_fail_limit <= 0) {
        add_err(errs, &n, max_errs, "commFailLimit 必须为正");
    }
    if (cfg->control.ramp_raw_per_tick <= 0) {
        add_err(errs, &n, max_errs, "rampRawPerTick 必须为正");
    }
    if (cfg->control.min_write_interval_ms < 0) {
        add_err(errs, &n, max_errs, "minWriteIntervalMs 不能为负");
    }
    if (!(cfg->mech.gear_ratio > 0.0) || !(cfg->mech.wheel_diameter_m > 0.0) ||
        !(cfg->mech.lx_plus_ly_m > 0.0)) {
        add_err(errs, &n, max_errs, "机械参数必须为正");
    }
    if (!(cfg->raw_per_rpm > 0.0)) {
        add_err(errs, &n, max_errs, "rawPerRpm 必须为正");
    }

    return n;
}
