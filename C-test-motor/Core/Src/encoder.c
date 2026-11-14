/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   霍尔编码器驱动实现（TIM5 编码器模式）
  ******************************************************************************
  */

#include "encoder.h"
#include "tim.h"        /* htim5 */
#include "usart.h"      /* huart2，printf 重定向要用 */

/* ==========================================================================
 * 一、内部状态（static：只在本文件可见，外部改不到）
 * ========================================================================== */

/** 上一次读到的计数值（注意是 int16_t，不是 uint16_t） */
static int16_t s_last_cnt = 0;

/** 这一段时间内的增量（可能为负，表示反转） */
static int16_t s_delta_cnt = 0;

/** 累计计数（int32_t，够跑很久：±21 亿格） */
static int32_t s_total_cnt = 0;


/* ==========================================================================
 * 二、printf 重定向
 * ==========================================================================
 * 用 arm-none-eabi-gcc 时，printf 最终会调到 _write()，
 * 而 STM32CubeMX 生成的 syscalls.c 里 _write() 会去调 __io_putchar()。
 * 所以我们只实现 __io_putchar()，printf 就能从 USART2 发出去。
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

/**
 * @brief 启动编码器接口并清零
 */
void Encoder_Init(void)
{
  /* 打开 TIM5 的编码器接口，两个通道都用 */
  /* （这一步不能少：CubeMX 只是"配置"了 TIM5，没有"启动"它） */
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  /* 计数器从 0 开始 */
  __HAL_TIM_SET_COUNTER(&htim5, 0);

  s_last_cnt  = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
  s_delta_cnt = 0;
  s_total_cnt = 0;
}

/**
 * @brief 读一次计数器，算出增量并累加
 */
void Encoder_Update(void)//强制转化成int16_t有利于判断正转还是反转
{
  /* now 是 int16_t：把 0 ~ 65535 的 CNT 解释成 -32768 ~ 32767
     这样正转反转都能正确表示 */
  int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);

  /* 差值用 int16_t 做减法，天然处理 65535 -> 0 的回绕。
     例：上次 65530，这次 3，now - last = 3 - 65530 = -65527
         截断成 int16_t 后正好是 +9（跨过了 65535->0）*/
  s_delta_cnt  = (int16_t)(now - s_last_cnt);
  s_last_cnt   = now;
  s_total_cnt += s_delta_cnt;
}

void Encoder_Reset(void)//里程表归零
{
  s_total_cnt = 0;
}

int16_t Encoder_GetDelta(void)//这10ms走了多少，瞬时速度
{
  return s_delta_cnt;
}

int32_t Encoder_GetTotal(void)//路程总和，循迹可能有用
{
  return s_total_cnt;
}

float Encoder_GetDistance_mm(void)
{
  return (float)s_total_cnt * ENC_MM_PER_COUNT;
}

float Encoder_GetSpeed_mm_s(uint32_t dt_ms)//PID反馈量
{
  if (dt_ms == 0U)//除0保护
  {
    return 0.0f;
  }

  /* 这一段走了多少毫米 = 计数 × 每计数多少毫米 */
  float dist_mm = (float)s_delta_cnt * ENC_MM_PER_COUNT;

  /* 毫米 / 秒 = 毫米 / (毫秒 / 1000) */
  return dist_mm * 1000.0f / (float)dt_ms;
}
