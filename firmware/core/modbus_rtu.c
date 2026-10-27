#include "modbus_rtu.h"

#include <string.h>

uint16_t ch_crc16_modbus(const uint8_t *buf, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)(buf[i] & 0xFF);
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

static void put_u16_be(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFF);
}

static void put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static uint16_t get_u16_be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint16_t get_u16_le(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0]) | ((uint16_t)p[1] << 8));
}

chassis_err_t ch_mb_write_request(uint8_t out[CH_MB_REQUEST_LEN], uint8_t slave,
                                  uint16_t addr, int value)
{
    if (value < -32768 || value > 65535) return CH_ERR_CAP;
    out[0] = slave;
    out[1] = CH_MB_FUNC_WRITE;
    put_u16_be(&out[2], addr);
    put_u16_be(&out[4], (uint16_t)(value < 0 ? value + 65536 : value));
    put_u16_le(&out[6], ch_crc16_modbus(out, 6));
    return CH_OK;
}

void ch_mb_read_request(uint8_t out[CH_MB_REQUEST_LEN], uint8_t slave,
                        uint16_t addr, uint16_t qty)
{
    out[0] = slave;
    out[1] = CH_MB_FUNC_READ;
    put_u16_be(&out[2], addr);
    put_u16_be(&out[4], qty);
    put_u16_le(&out[6], ch_crc16_modbus(out, 6));
}

int16_t ch_mb_i16(uint16_t u) { return (int16_t)u; }
uint16_t ch_mb_u16(int16_t v) { return (uint16_t)v; }

static ch_mb_result_t fail(const char *why, int len, uint8_t code)
{
    ch_mb_result_t r;
    r.ok = false;
    r.exception = code;
    r.error = why;
    r.raw_len = len;
    return r;
}

static bool is_exception_frame(const uint8_t *buf, int len)
{
    return len >= 3 && buf[1] >= 0x80;
}

static ch_mb_result_t exception_result(const uint8_t *buf, int len)
{
    ch_mb_result_t r = fail(ch_mb_exception_text(buf[2]), len, buf[2]);
    return r;
}

ch_mb_result_t ch_mb_parse_read_response(const uint8_t *buf, int len, uint8_t slave,
                                         uint16_t start_addr, uint16_t qty, int16_t *regs_out)
{
    if (buf == NULL || len == 0) return fail("no-response", len, 0);
    if (buf[0] != slave) return fail("slave-mismatch", len, 0);
    if (is_exception_frame(buf, len)) return exception_result(buf, len);
    /* 从站号对上但只收了 1~2 字节（线路错误 / 帧中静默）也要先挡住：下面要读 buf[2] */
    if (len < 3) return fail("short-response", len, 0);
    (void)start_addr;   /* regs_out 按位置回填：[0] = start_addr，[1] = start_addr+1 … */

    int byte_count = buf[2];
    if (byte_count != (int)qty * 2) return fail("byte-count", len, 0);
    int need = 3 + byte_count + 2;
    if (len < need) return fail("truncated", len, 0);
    if (get_u16_le(&buf[need - 2]) != ch_crc16_modbus(buf, need - 2)) return fail("crc", len, 0);

    for (int i = 0; i < (int)qty; i++) {
        regs_out[i] = ch_mb_i16(get_u16_be(&buf[3 + i * 2]));
    }
    ch_mb_result_t r;
    r.ok = true;
    r.exception = 0;
    r.error = NULL;
    r.raw_len = need;
    return r;
}

ch_mb_result_t ch_mb_parse_write_response(const uint8_t *buf, int len, const uint8_t *request)
{
    if (buf == NULL || len == 0) return fail("no-response", len, 0);
    if (buf[0] != request[0]) return fail("slave-mismatch", len, 0);
    if (is_exception_frame(buf, len)) return exception_result(buf, len);
    if (len < 8) return fail("short-response", len, 0);
    for (int i = 0; i < 6; i++) {
        if (buf[i] != request[i]) return fail("echo-mismatch", len, 0);
    }
    if (get_u16_le(&buf[6]) != ch_crc16_modbus(buf, 6)) return fail("crc", len, 0);

    ch_mb_result_t r;
    r.ok = true;
    r.exception = 0;
    r.error = NULL;
    r.raw_len = 8;
    return r;
}

const char *ch_mb_exception_text(uint8_t code)
{
    switch (code) {
        case 1: return "功能码非法";
        case 2: return "地址非法（参数号/组别不对）";
        case 3: return "值非法（Out-r：超出该参数设定范围）";
        case 4: return "跨参数组读取/从站失效";
        case 6: return "伺服器运行中不能修改（Sd-On：先关使能再写）";
        default: return "异常";
    }
}

void ch_mb_frame_note(const uint8_t *buf, int len, char *out, int out_size)
{
    static const char *hex = "0123456789ABCDEF";
    int p = 0;
    if (out_size <= 0) return;
    out[0] = '\0';
    if (buf == NULL || len <= 0) {
        const char *none = "无响应（超时，一个字节都没回来）";
        strncpy(out, none, (size_t)(out_size - 1));
        out[out_size - 1] = '\0';
        return;
    }
    for (int i = 0; i < len; i++) {
        if (p + 3 >= out_size) break;
        out[p++] = (i ? ' ' : '[');
        out[p++] = hex[(buf[i] >> 4) & 0xF];
        out[p++] = hex[buf[i] & 0xF];
    }
    if (p + 1 < out_size) out[p++] = ']';
    out[p] = '\0';
}
