/* H743 应用环。刻意保持很短：一拍里做的事全在 core 里，这里只管上电、喂拍、把输入递进去。
 *
 * 安全口径（和 E:\底盘\手柄控制操作流程.md 一致）：
 *   • 上电默认不动。预检通过只代表"允许跑"，不代表"开始跑"。
 *   • 使能、轮子离地/清场、每一档提速，都要重新做一次人工确认，代码里不做自动升级。
 *   • 驱动器没有通讯断线停车：这个环一停，轮子会保持着最后速度一直转，
 *     所以硬件看门狗必须在这个循环里喂 —— 见 chassis_app_tick() 里的 TODO。 */

#include "chassis_app_h743.h"

#include "board_pins_h743.h"
#include "chassis_controller.h"
#include "chassis_preset.h"

static chassis_config_t s_src;          /* 铭牌与机械常数，不参与改写 */
static chassis_config_t s_cfg;          /* 按 CH_DEFAULT_TARGET_MPS 折算出来的运行配置 */
static chassis_ctrl_t   s_ctrl;
static uint32_t         s_next_ms;
static uint32_t         s_period_ms;
static bool             s_running;
static const char      *s_reason = "not booted";

static void on_event(void *ctx, chassis_event_code_t code, int aux)
{
    (void)ctx; (void)aux;
    /* TODO: 挂到 LED / 上位机上报。事件里最关键的是 CH_EV_STOP_FAIL 和 CH_EV_LATCHED：
     *       前者表示"想停但 0 没写进去"，后者表示已经闭锁。 */
    switch (code) {
    case CH_EV_STOP_FAIL:   s_reason = "stop write failed"; break;
    case CH_EV_LATCHED:     s_reason = "latched"; break;
    case CH_EV_WRITE_FAIL:  s_reason = "write failed"; break;
    case CH_EV_PROBE_FAIL:  s_reason = "heartbeat failed"; break;
    default: break;
    }
}

int chassis_app_boot(void)
{
    char errs[8][96];

    chassis_config_default(&s_src);
    if (chassis_speed_preset(&s_src, CH_DEFAULT_TARGET_MPS, CH_DEFAULT_ACCEL_MPS2, &s_cfg, NULL) != CH_OK) {
        s_reason = "preset rejected";
        return 1;                       /* 铭牌没标定或超过额定折算 → 干脆不起环 */
    }
    /* 轮位 → 通道的映射由板级给（一条 485 挂两台），核心自己不知道接了几路 */
    chassis_hw_map_channels(&s_cfg);
    if (chassis_validate_config(&s_cfg, errs, 8) != 0) {
        s_reason = "config invalid";
        return 2;                       /* 配置自相矛盾 → 干脆不起环 */
    }

    /* 实机口径 9600 8-N-2 */
    chassis_hw_uart_init(9600u, UART_WORDLENGTH_8B, UART_STOPBITS_2, UART_PARITY_NONE);

    /* 四个通道没凑齐就不起环。留一只能动的路更危险：运动学照样给四轮算指令，
     * 车上只有一只轮子有响应，麦轮车会直接横着拧过去。 */
    for (int i = 0; i < CHASSIS_WHEELS; i++) {
        if (!chassis_hw_channel_ready(s_cfg.wheels[i].channel)) {
            s_reason = "RS485 channel not wired";
            return 3;
        }
    }

    chassis_bus_t bus = chassis_make_bus(chassis_hw_uarts(), CHASSIS_HW_CHANNELS);
    chassis_ctrl_init(&s_ctrl, &s_cfg, &bus);
    s_ctrl.on_event = on_event;         /* init 里 memset 过，回调必须在其后挂 */

    s_period_ms = (uint32_t)(1000u / (uint32_t)s_cfg.control.loop_hz);
    if (s_period_ms == 0u) { s_period_ms = 1u; }
    s_next_ms = HAL_GetTick();
    s_running = true;
    s_reason = "idle";
    return 0;
}

void chassis_app_input(double vx, double vy, double wz)
{
    if (!s_running) { return; }
    chassis_twist_t t = { vx, vy, wz };
    chassis_ctrl_set_twist(&s_ctrl, &t, HAL_GetTick());
}

void chassis_app_stop(void)
{
    if (!s_running) { return; }
    chassis_ctrl_zero_twist(&s_ctrl, HAL_GetTick());
    int failed = chassis_ctrl_stop_all(&s_ctrl);
    if (failed > 0) { s_reason = "stop write failed"; }
}

void chassis_app_tick(void)
{
    if (!s_running) { return; }

    /* TODO: 在这里喂硬件独立看门狗（IWDG）。喂狗必须在这条路径上，
     *       这样一旦这个环卡死，看门狗复位 → 驱动器失去命令但轮子仍在转的问题
     *       至少由"复位后重新闭锁"兜住；复位后 twist 依然是 0。 */
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - s_next_ms) < 0) { return; }
    chassis_ctrl_step(&s_ctrl, now, NULL);
    s_next_ms += s_period_ms;
    /* 落后了就跳过欠的那些拍，不补拍：补拍要拿伪造的未来时间戳去喂决策，
     * 400 ms 的输入看门狗会被自己绊倒，报出一个根本没发生的失联。 */
    if ((int32_t)(now - s_next_ms) >= 0) { s_next_ms = now + s_period_ms; }
}

bool chassis_app_running(void) { return s_running; }
const char *chassis_app_last_reason(void) { return s_reason; }
