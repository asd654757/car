#ifndef CHASSIS_APP_H743_H
#define CHASSIS_APP_H743_H

#include <stdbool.h>
#include <stdint.h>

/* H743 上的应用环：core 的控制器 + port 的 UART 层。
 * 板级 main.c 只负责时钟/SysTick/LED 这些外设，控制逻辑一律不写在 main 里。 */

/* 0 = 环已就绪（注意：就绪 ≠ 会动，twist 从 0 起步，提速必须由输入层显式下达）。
 * 非 0 = 预检没过，轮子一律不动，原因见 chassis_app_last_reason()。 */
int chassis_app_boot(void);

/* 输入层唯一入口：车身速度（m/s, m/s, rad/s）。手柄/接收机/键盘都调这个，核心不认设备。 */
void chassis_app_input(double vx, double vy, double wz);

/* 立即归零并绕过节流（退出、摇杆回中、遥控失联都用它） */
void chassis_app_stop(void);

/* 在主循环里调用；到点才真正跑一拍，其余时间立刻返回 */
void chassis_app_tick(void);

bool chassis_app_running(void);
const char *chassis_app_last_reason(void);

#endif /* CHASSIS_APP_H743_H */
