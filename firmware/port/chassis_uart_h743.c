/* H743 板级 UART 层：把 board_pins_h743.h 那张表实现成 chassis_transport 要的五个回调。
 *
 * 引脚依据是野火例程包里的 BSP（不是我发明的）：
 *   7-2-多串口应用-RS485(U2)/User/uart/bsp_rs485.h
 *     USART2  TX = PD5 / RX = PD6 / 方向 = PD4，发送态置高
 *   7-1-多串口应用-RS232(U2+U5)/User/uart/bsp_uart5.h
 *     UART5   TX = PB12 / RX = PB13（这一路是 RS232，没有方向脚）
 * 第二路板载 RS485 在例程包里查不到，外接模块接哪几个串口也还没定 —— 所以下面只填了通道 0，
 * 留空的通道不初始化、发不出去，连续三次通讯失败会进 FAULT 闭锁，宁可整台车不动。 */

#include "board_pins_h743.h"

#include "chassis_config.h"

/* 驱动器实测口径：9600 8-N-2（见 E:\底盘\手柄控制操作流程.md） */
#define CHASSIS_TX_TIMEOUT_MS 20u

static chassis_hw_uart_t s_hw[CHASSIS_HW_CHANNELS] = {
    { USART2, GPIOD, GPIO_PIN_5, GPIO_AF7_USART2,
            GPIOD, GPIO_PIN_6, GPIO_AF7_USART2,
            GPIOD, GPIO_PIN_4, GPIO_PIN_SET },
    { NULL, NULL, 0u, 0u, NULL, 0u, 0u, NULL, 0u, 0u },
    { NULL, NULL, 0u, 0u, NULL, 0u, 0u, NULL, 0u, 0u },
    { NULL, NULL, 0u, 0u, NULL, 0u, 0u, NULL, 0u, 0u },
};

static UART_HandleTypeDef s_h[CHASSIS_HW_CHANNELS];
static chassis_uart_t     s_uart[CHASSIS_HW_CHANNELS];
static uint8_t            s_idx[CHASSIS_HW_CHANNELS];

/* 2026-10-10 定下的接法：一条 485 挂两台 —— 前轮一对走通道 0，后轮一对走通道 1。
 * 站号本来就错开着（FL=1 FR=4 / BL=2 BR=3），所以每条总线内部不撞号，
 * 改的只是接线，驱动器参数一个都不用动。
 * 通道 1 现在是空的：板载只有 USART2 那一路 485 有 BSP 依据，第二路要等你给出
 * 原理图上的 UART 与方向脚 —— 我不猜引脚。没凑齐时 chassis_app_boot 直接不起环。 */
static const uint8_t s_wheel_channel[CHASSIS_WHEELS] = { 0u, 0u, 1u, 1u };

void chassis_hw_map_channels(chassis_config_t *cfg)
{
    for (int i = 0; i < CHASSIS_WHEELS; i++) {
        cfg->wheels[i].channel = s_wheel_channel[i];
    }
}

static void enable_gpio_clock(GPIO_TypeDef *port)
{
    if (port == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); }
    else if (port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
    else if (port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
    else if (port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
    else if (port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
    else if (port == GPIOF) { __HAL_RCC_GPIOF_CLK_ENABLE(); }
    else if (port == GPIOG) { __HAL_RCC_GPIOG_CLK_ENABLE(); }
    else if (port == GPIOH) { __HAL_RCC_GPIOH_CLK_ENABLE(); }
}

static void enable_uart_clock(USART_TypeDef *inst)
{
    if (inst == USART1) { __HAL_RCC_USART1_CLK_ENABLE(); }
    else if (inst == USART2) { __HAL_RCC_USART2_CLK_ENABLE(); }
    else if (inst == USART3) { __HAL_RCC_USART3_CLK_ENABLE(); }
    else if (inst == USART6) { __HAL_RCC_USART6_CLK_ENABLE(); }
    else if (inst == UART4) { __HAL_RCC_UART4_CLK_ENABLE(); }
    else if (inst == UART5) { __HAL_RCC_UART5_CLK_ENABLE(); }
    else if (inst == UART7) { __HAL_RCC_UART7_CLK_ENABLE(); }
    else if (inst == UART8) { __HAL_RCC_UART8_CLK_ENABLE(); }
}

static void set_direction(void *ctx, bool tx)
{
    const uint8_t i = *(const uint8_t *)ctx;
    const chassis_hw_uart_t *hw = &s_hw[i];
    /* 认句柄不认表：初始化失败的路也被标成"没接"，这里再去碰寄存器就是空指针 */
    if (s_h[i].Instance == NULL) { return; }
    if (hw->dir_port != NULL) {
        uint32_t level = tx ? (uint32_t)hw->dir_tx_level
                            : (hw->dir_tx_level == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(hw->dir_port, hw->dir_pin, level);
    }
    if (tx) {
        /* 丢掉上一笔超时之后才姗姗来迟的字节。不丢的话它会被当成本笔的回执，
         * 报成 echo/CRC 失败 —— 表现是"明明通了却连续三次失败进闭锁"。 */
        UART_HandleTypeDef *h = &s_h[i];
        __HAL_UART_CLEAR_OREFLAG(h);
        while ((h->Instance->ISR & USART_ISR_RXNE_RXFNE) != 0u) { (void)h->Instance->RDR; }
    }
}

static int write_bytes(void *ctx, const uint8_t *buf, int n)
{
    UART_HandleTypeDef *h = &s_h[*(const uint8_t *)ctx];
    if (h->Instance == NULL) { return 0; }
    /* HAL 的阻塞发送在最后一个字节上等的是 TC（移位寄存器空），不是 TXE，
     * 所以返回之后再切收不会啃掉尾字节 —— 这正是半双联 485 最容易错的地方。 */
    if (HAL_UART_Transmit(h, (uint8_t *)buf, (uint32_t)n, CHASSIS_TX_TIMEOUT_MS) != HAL_OK) { return 0; }
    return n;
}

static int read_byte(void *ctx, uint8_t *out, int timeout_ms)
{
    UART_HandleTypeDef *h = &s_h[*(const uint8_t *)ctx];
    if (h->Instance == NULL) { return 0; }
    HAL_StatusTypeDef rc = HAL_UART_Receive(h, out, 1u, (uint32_t)timeout_ms);
    if (rc == HAL_OK) { return 1; }
    if (rc == HAL_TIMEOUT) { return 0; }
    h->ErrorCode = HAL_UART_ERROR_NONE;   /* 线路错误（噪声/帧错/溢出）报负数，让上层记一次失败 */
    return -1;
}

static uint32_t now_ms(void *ctx)
{
    (void)ctx;
    return HAL_GetTick();
}

void chassis_hw_uart_init(uint32_t baud, uint32_t word_length, uint32_t stop_bits, uint32_t parity)
{
    for (uint8_t i = 0; i < CHASSIS_HW_CHANNELS; i++) {
        const chassis_hw_uart_t *hw = &s_hw[i];
        UART_HandleTypeDef *h = &s_h[i];

        s_idx[i] = i;
        s_uart[i].ctx = &s_idx[i];
        s_uart[i].set_direction_tx = set_direction;
        s_uart[i].write_bytes = write_bytes;
        s_uart[i].read_byte = read_byte;
        s_uart[i].now_ms = now_ms;

        if (hw->inst == NULL) { continue; }

        enable_gpio_clock(hw->tx_port);
        enable_gpio_clock(hw->rx_port);
        enable_gpio_clock(hw->dir_port);
        enable_uart_clock(hw->inst);

        GPIO_InitTypeDef g;
        g.Mode = GPIO_MODE_AF_PP;
        g.Pull = GPIO_PULLUP;
        g.Speed = GPIO_SPEED_FREQ_LOW;
        g.Pin = hw->tx_pin;
        g.Alternate = hw->tx_af;
        HAL_GPIO_Init(hw->tx_port, &g);
        g.Pin = hw->rx_pin;
        g.Alternate = hw->rx_af;
        HAL_GPIO_Init(hw->rx_port, &g);

        if (hw->dir_port != NULL) {
            g.Pin = hw->dir_pin;
            g.Mode = GPIO_MODE_OUTPUT_PP;
            g.Pull = GPIO_NOPULL;
            g.Alternate = 0u;
            HAL_GPIO_Init(hw->dir_port, &g);
            HAL_GPIO_WritePin(hw->dir_port, hw->dir_pin,
                              hw->dir_tx_level == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
        }

        h->Instance = hw->inst;
        h->Init.BaudRate = baud;
        h->Init.WordLength = word_length;
        h->Init.StopBits = stop_bits;
        h->Init.Parity = parity;
        h->Init.Mode = UART_MODE_TX_RX;
        h->Init.HwFlowCtl = UART_HWCONTROL_NONE;
        h->Init.OverSampling = UART_OVERSAMPLING_16;
        h->Init.ClockPrescaler = UART_PRESCALER_DIV1;
        h->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        /* AdvancedInit 是静态零初始化，零值就是 UART_ADVFEATURE_NO_INIT，不用显式写 */
        if (HAL_UART_Init(h) != HAL_OK) { h->Instance = NULL; continue; }   /* 初始化失败 → 这一路当"没接"，别硬发 */

        /* H7 的 UART 复位后 FIFO 是开着的，RXNE 要等到 FIFO 达到门限才置位。
         * 我们按字节收（帧间静默判帧尾），所以必须关 FIFO，否则第一个字节永远"读不到"。 */
        __HAL_UART_DISABLE(h);
        CLEAR_BIT(h->Instance->CR1, USART_CR1_FIFOEN);
        __HAL_UART_ENABLE(h);
    }
}

const chassis_uart_t *chassis_hw_uarts(void) { return s_uart; }
const chassis_hw_uart_t *chassis_hw_uart_table(int *n)
{
    if (n) { *n = CHASSIS_HW_CHANNELS; }
    return s_hw;
}

bool chassis_hw_channel_ready(int channel)
{
    if (channel < 0 || channel >= CHASSIS_HW_CHANNELS) { return false; }
    return s_h[channel].Instance != NULL;
}
