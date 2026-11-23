/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   霍尔编码器驱动实现（四路，TIM1~TIM4 编码器模式）
  ******************************************************************************
  */

#include "encoder.h"
#include "usart.h"      /* huart2，printf 重定向要用 */

/* ==========================================================================
 * 一、编码器表
 * ==========================================================================
 * 字段顺序：定时器, 上次计数, 增量, 累计计数
 * 后三个初值给 0，Init 时会重新对齐。
 */
Encoder_t g_enc[ENC_NUM] = {
    { &htim1, 0, 0, 0 },   /* 0：左前，PA8 / PA9  */
    { &htim5, 0, 0, 0 },   /* 1：左后，PA0 / PA1  */
    { &htim3, 0, 0, 0 },   /* 2：右前，PA6 / PA7  */
    { &htim4, 0, 0, 0 },   /* 3：右后，PB6 / PB7  */
};


/* ==========================================================================
 * 二、printf 重定向
 * ==========================================================================
 * 用 arm-none-eabi-gcc 时，printf 最终会调到 _write()，
 * 而 CubeMX 生成的 syscalls.c 里 _write() 会去调 __io_putchar()。
 */
int __io_putchar(int ch)
{
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart2, &c, 1, 0xFFFF);
    return ch;
}


/* ==========================================================================
 * 三、接口实现
 * ========================================================================== */

void Encoder_InitAll(void)
{
    for (uint8_t i = 0; i < ENC_NUM; i++) {
        Encoder_t *e = &g_enc[i];
        if (e->htim == NULL) continue;

        /* CubeMX 只是"配置"了定时器，必须再"启动"它才会真正计数 */
        HAL_TIM_Encoder_Start(e->htim, TIM_CHANNEL_ALL);

        __HAL_TIM_SET_COUNTER(e->htim, 0);
        e->last  = 0;
        e->delta = 0;
        e->total = 0;
    }
}

void Encoder_UpdateAll(void)
{
    for (uint8_t i = 0; i < ENC_NUM; i++) {
        Encoder_t *e = &g_enc[i];
        if (e->htim == NULL) continue;

        /* 把 0~65535 的 CNT 解释成 -32768~32767，正反都能表示 */
        int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(e->htim);

        /* 用 int16_t 做减法，天然处理 65535 -> 0 的回绕：
           上次 65530、这次 3，now-last = -65527，截断成 int16_t 正好是 +9 */
        /* 符号约定：正占空比 = 正转 = 正计数。
           本车 A/B 相序接反，硬件上正转会让 CNT 变小，所以在源头取一次反号。
           以后换电机方向/对调 A、B 线，只改这一行。 */
        e->delta  = -(int16_t)(now - e->last);
        e->last   = now;
        e->total += e->delta;
    }
}

void Encoder_Reset(uint8_t id)
{
    if (id >= ENC_NUM) return;
    g_enc[id].total = 0;
}

void Encoder_ResetAll(void)
{
    for (uint8_t i = 0; i < ENC_NUM; i++) {
        g_enc[i].total = 0;
    }
}

int16_t Encoder_GetDelta(uint8_t id)
{
    if (id >= ENC_NUM) return 0;
    return g_enc[id].delta;
}

int32_t Encoder_GetTotal(uint8_t id)
{
    if (id >= ENC_NUM) return 0;
    return g_enc[id].total;
}

float Encoder_GetDistance_mm(uint8_t id)
{
    if (id >= ENC_NUM) return 0.0f;
    return (float)g_enc[id].total * ENC_MM_PER_COUNT;
}

float Encoder_GetSpeed_mm_s(uint8_t id, uint32_t dt_ms)
{
    if (id >= ENC_NUM || dt_ms == 0U) return 0.0f;

    float dist_mm = (float)g_enc[id].delta * ENC_MM_PER_COUNT;
    return dist_mm * 1000.0f / (float)dt_ms;
}
