#ifndef CHASSIS_MODBUS_RTU_H
#define CHASSIS_MODBUS_RTU_H

#include <stdbool.h>
#include <stdint.h>

#include "chassis_config.h"

/* 与实机验证过的 tools/ws_speed_cmd.ps1 逐字节等价 */

#define CH_MB_FUNC_READ        0x03
#define CH_MB_FUNC_WRITE       0x06
#define CH_MB_REQUEST_LEN      8
#define CH_MB_MAX_QTY          125

#define CH_SLAVE_SPEED         1   /* 站号本身在 config.wheels 里，这里只是文档性常量 */

uint16_t ch_crc16_modbus(const uint8_t *buf, int len);

/* value 允许负数，自动转补码。返回 CH_ERR_CAP = 超出 16 位范围 */
chassis_err_t ch_mb_write_request(uint8_t out[CH_MB_REQUEST_LEN], uint8_t slave,
                                  uint16_t addr, int value);
void ch_mb_read_request(uint8_t out[CH_MB_REQUEST_LEN], uint8_t slave,
                        uint16_t addr, uint16_t qty);

int16_t ch_mb_i16(uint16_t u);
uint16_t ch_mb_u16(int16_t v);

typedef struct {
    bool ok;
    uint8_t exception;      /* 0 = 没有异常码 */
    const char *error;      /* 静态串，供日志 */
    int raw_len;
} ch_mb_result_t;

/* 读响应 → regs_out[i] = start_addr+i 的有符号值。qty 个，调用方保证数组够长 */
ch_mb_result_t ch_mb_parse_read_response(const uint8_t *buf, int len, uint8_t slave,
                                         uint16_t start_addr, uint16_t qty, int16_t *regs_out);

/* 写响应（驱动器回显请求帧）。⚠ 先认异常帧再判长度：异常帧只有 5 字节，
 * 先判 length<8 会把"驱动器为什么拒绝这次写"压成一句 short-response */
ch_mb_result_t ch_mb_parse_write_response(const uint8_t *buf, int len,
                                          const uint8_t *request);

const char *ch_mb_exception_text(uint8_t code);

/* 把回来的字节原样转成十六进制 —— 现场只有一行文字时，这串就是唯一的证据 */
void ch_mb_frame_note(const uint8_t *buf, int len, char *out, int out_size);

#endif /* CHASSIS_MODBUS_RTU_H */
