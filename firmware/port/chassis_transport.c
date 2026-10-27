#include "chassis_transport.h"

#include <string.h>

#define CHASSIS_REQ_LEN   CH_MB_REQUEST_LEN
#define CHASSIS_TIMEOUT_MS 100   /* 实机四口并行 round-trip 25~46 ms，留到 100 ms 足够宽 */

chassis_err_t chassis_modbus_transfer(const chassis_uart_t *u, const uint8_t *req, int req_len,
                                      uint8_t *rsp, int rsp_cap, int timeout_ms, int *rsp_len)
{
    int len = 0;
    if (rsp_len) *rsp_len = 0;
    if (u == NULL || u->write_bytes == NULL || u->read_byte == NULL || u->set_direction_tx == NULL) {
        return CH_ERR_BAD_PARAM;
    }

    u->set_direction_tx(u->ctx, true);
    int sent = u->write_bytes(u->ctx, req, req_len);
    u->set_direction_tx(u->ctx, false);      /* 立刻转收：这路是半双工，晚一个字节就吃掉回帧头 */
    if (sent != req_len) return CH_ERR_BAD_PARAM;

    /* 一条 485 上挂两台时总线是共享的：上一笔超时后才姗姗来迟的回帧、甚至问别家的答语，
     * 都会在这一笔里冒出来。按从站号认帧，不是我们的就整帧丢掉，等到帧间静默再回头等自己那笔。
     * 不丢的话它会被当成本笔的回执报成 echo-mismatch，连续 comm_fail_limit 次就误锁。 */
    int expected = rsp_cap;      /* 先按容量当上限，认出功能码后收紧 */
    int discard = 0;             /* 1 = 正在丢别人那帧的剩余字节 */
    uint32_t start = u->now_ms ? u->now_ms(u->ctx) : 0u;
    uint32_t deadline = start + (uint32_t)timeout_ms;

    for (;;) {
        if (len > 0 && len >= expected) break;          /* 收满一笔 */
        if (len >= rsp_cap) break;                      /* 缓冲区满（bytecount 不可信） */

        int wait;
        if (discard || len > 0) {
            wait = CHASSIS_SILENCE_MS;                  /* 帧内：静默 3.5 字符就当帧尾 */
        } else {
            if (!u->now_ms) { wait = timeout_ms; }
            else {
                uint32_t left = (uint32_t)(deadline - u->now_ms(u->ctx));   /* 无符号回绕差 */
                wait = (left > (uint32_t)timeout_ms) ? 0 : (int)left;       /* 已过线 → 不再等 */
            }
            if (wait <= 0) break;
        }

        uint8_t b;
        int rc = u->read_byte(u->ctx, &b, wait);
        if (rc == 0) {
            if (discard) { discard = 0; expected = rsp_cap; continue; }   /* 别人那帧说完了 */
            break;                                       /* 静默 = 帧结束 */
        }
        if (rc < 0) break;                               /* 线路错误：把已收到的交给上层判，别当成功 */

        if (discard) continue;                           /* 继续丢，直到出现帧间静默 */
        rsp[len++] = b;

        if (len == 1 && b != req[0]) {                   /* 不是问我们的从站 → 整帧作废 */
            discard = 1;
            len = 0;
            continue;
        }
        if (len == 2) {
            if (rsp[1] >= 0x80) expected = 5;                        /* 异常帧固定 5 字节 */
            else if (rsp[1] == CH_MB_FUNC_WRITE) expected = 8;
            else if (rsp[1] == CH_MB_FUNC_READ) expected = rsp_cap;  /* 长度看 bytecount */
            else expected = 0;                                       /* 认不出的功能码 */
        } else if (len == 3 && rsp[1] == CH_MB_FUNC_READ && rsp[2] <= 250) {
            expected = 3 + rsp[2] + 2;
        }
        if (expected == 0) break;
    }

    if (rsp_len) *rsp_len = len;
    return CH_OK;
}

typedef struct {
    const chassis_uart_t *uarts;
    int n;
    uint8_t rx[256];
} bus_adapter_t;

static bus_adapter_t g_adapter;   /* 整机一条总线，够用；要扩到多条再改成数组 */

static int adapter_write_reg(void *ctx, uint8_t channel, uint8_t slave, uint16_t reg, int16_t value)
{
    bus_adapter_t *a = (bus_adapter_t *)ctx;
    if (channel >= (uint8_t)a->n) return -1;
    const chassis_uart_t *u = &a->uarts[channel];

    uint8_t req[CHASSIS_REQ_LEN];
    if (ch_mb_write_request(req, slave, reg, value) != CH_OK) return -2;

    int len = 0;
    if (chassis_modbus_transfer(u, req, CHASSIS_REQ_LEN, a->rx, (int)sizeof(a->rx),
                                CHASSIS_TIMEOUT_MS, &len) != CH_OK) return -3;

    ch_mb_result_t r = ch_mb_parse_write_response(a->rx, len, req);
    return r.ok ? 0 : -1;
}

static int adapter_read_block(void *ctx, uint8_t channel, uint8_t slave, uint16_t reg,
                              uint16_t qty, int16_t *out)
{
    bus_adapter_t *a = (bus_adapter_t *)ctx;
    if (channel >= (uint8_t)a->n || qty == 0 || qty > CH_MB_MAX_QTY) return -1;
    const chassis_uart_t *u = &a->uarts[channel];

    uint8_t req[CHASSIS_REQ_LEN];
    ch_mb_read_request(req, slave, reg, qty);

    int len = 0;
    if (chassis_modbus_transfer(u, req, CHASSIS_REQ_LEN, a->rx, (int)sizeof(a->rx),
                                CHASSIS_TIMEOUT_MS, &len) != CH_OK) return -3;

    ch_mb_result_t r = ch_mb_parse_read_response(a->rx, len, slave, reg, qty, out);
    return r.ok ? 0 : -1;
}

chassis_bus_t chassis_make_bus(const chassis_uart_t *uart_by_channel, int n_channels)
{
    chassis_bus_t bus;
    memset(&bus, 0, sizeof(bus));
    g_adapter.uarts = uart_by_channel;
    g_adapter.n = n_channels;
    bus.ctx = &g_adapter;
    bus.write_reg = adapter_write_reg;
    bus.read_block = adapter_read_block;
    return bus;
}
