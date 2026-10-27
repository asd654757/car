#ifndef BOARD_PINS_H743_H
#define BOARD_PINS_H743_H

#include "chassis_transport.h"
#include "stm32h7xx_hal.h"

/* 一路 RS485 需要的全部板级信息。引脚只允许出现在下面那张表里：
 * 传输层和安全层都不认识 GPIO，换串口、改线、加外接收发器都只动这张表。 */
typedef struct {
    USART_TypeDef  *inst;         /* NULL = 这一路还没定：不初始化，任何传输必然失败（fail-safe） */
    GPIO_TypeDef   *tx_port;
    uint16_t        tx_pin;
    uint32_t        tx_af;
    GPIO_TypeDef   *rx_port;
    uint16_t        rx_pin;
    uint32_t        rx_af;
    GPIO_TypeDef   *dir_port;     /* NULL = 没有方向脚（TTL 收发器） */
    uint16_t        dir_pin;
    uint32_t        dir_tx_level; /* 发送态是置高还是置低，随收发器 DE/RE 接法而变 */
} chassis_hw_uart_t;

#define CHASSIS_HW_CHANNELS CHASSIS_WHEELS

/* Modbus RTU 实机口径 9600 8-N-2，参数由调用方给，别在这里写死第二份 */
void                chassis_hw_uart_init(uint32_t baud, uint32_t word_length, uint32_t stop_bits,
                                         uint32_t parity);
const chassis_uart_t *chassis_hw_uarts(void);
const chassis_hw_uart_t *chassis_hw_uart_table(int *n);

/* 这一路是不是真的接好了（有 UART 实例、且初始化成功）。
 * 起环之前要逐轮问一遍：三只驱动器的线还没接、只有一只能动的话，
 * 麦轮车不会"慢一点"，它会原地打转 —— 所以宁可四个通道没凑齐就整台车不动。 */
bool chassis_hw_channel_ready(int channel);

/* 轮位 → 第几路 RS485。走线一改这里就要跟着改，核心只认编号、不认引脚。 */
void chassis_hw_map_channels(chassis_config_t *cfg);

#endif /* BOARD_PINS_H743_H */
