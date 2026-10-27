#ifndef CHASSIS_TRANSPORT_H
#define CHASSIS_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>

#include "chassis_controller.h"
#include "modbus_rtu.h"

/* 与芯片无关的 Modbus RTU 主站传输层。板级代码只需要实现下面这五个回调，
 * 帧构造/CRC/判帧/超时都由这里管，避免每路串口各写一遍、各错一遍。 */

typedef struct chassis_uart {
    void *ctx;
    /* RS485 方向：true = 发（DE 拉高 / RE 拉低），false = 收 */
    void (*set_direction_tx)(void *ctx, bool tx);
    /* 阻塞发完 n 个字节，返回实际发出数 */
    int (*write_bytes)(void *ctx, const uint8_t *buf, int n);
    /* 收一个字节；返回 1 = 拿到，0 = timeout_ms 内没有，负数 = 线路错误 */
    int (*read_byte)(void *ctx, uint8_t *out, int timeout_ms);
    uint32_t (*now_ms)(void *ctx);
} chassis_uart_t;

/* 一次请求-响应。rsp_cap 至少 256。返回 CH_OK 或错误码，*rsp_len 回填收到的字节数 */
chassis_err_t chassis_modbus_transfer(const chassis_uart_t *u, const uint8_t *req, int req_len,
                                      uint8_t *rsp, int rsp_cap, int timeout_ms, int *rsp_len);

/* 把一路串口挂成核心用的总线：channel 与 chassis_config_t.wheels[].channel 对应 */
chassis_bus_t chassis_make_bus(const chassis_uart_t *uart_by_channel, int n_channels);

/* 9600 8-N-2 下一个字符 = 11 位 ≈ 1.146 ms；静默判帧尾用 3.5 字符 */
#define CHASSIS_BYTE_US_AT_9600  1146
#define CHASSIS_SILENCE_MS       4

#endif /* CHASSIS_TRANSPORT_H */
