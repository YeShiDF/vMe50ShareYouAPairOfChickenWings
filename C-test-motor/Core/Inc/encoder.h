/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   霍尔编码器驱动（TIM5 编码器模式，A 相 = PA0 / B 相 = PA1）
  ******************************************************************************
  * 这个模块只干三件事：
  *   1. 把 TIM5 的编码器接口打开
  *   2. 周期性读回 CNT，算出"这一小段时间转了多少格"
  *   3. 把"格"换算成"毫米"和"毫米每秒"
  *
  * 上层（电机控制、PID、里程统计）只调用这几个函数，不直接碰寄存器。
  ******************************************************************************
  */

#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ==========================================================================
 * 一、机械参数 —— 这几个数必须和你车上的真实硬件一致
 * ========================================================================== */

/** 编码器线数：电机转一圈，A 相（或 B 相）输出多少个完整方波周期
 *  MG310 常见是 11 线或 13 线，看技术手册确认。 */
#define ENC_LINES_PER_MOTOR_REV     13.0f//(f是为了控制单精度，减少单片机运算负担)

/** 倍频数：由 CubeMX 里的 Encoder Mode 决定
 *  TI1 and TI2 -> x4（本工程用的是这个）
 *  TI1 或 TI2  -> x2
 *  这两个值必须和 CubeMX 里选的一致，否则速度会差一倍。 */
#define ENC_MULTIPLIER              4.0f

/** 减速比：电机转多少圈，输出轴（轮子）才转 1 圈
 *  MG310 常见 1:30 / 1:20 / 1:50，查技术手册确认。 */
#define ENC_GEAR_RATIO              20.409f

/** 轮子直径（毫米）。52mm 麦轮 = 52.0f */
#define WHEEL_DIAMETER_MM           52.0f


/* ==========================================================================
 * 二、派生量 —— 由上面四个数算出来，不用改
 * ========================================================================== */

/** 电机转一圈，计数器变化多少（= 线数 × 倍频） */
#define ENC_COUNTS_PER_MOTOR_REV    (ENC_LINES_PER_MOTOR_REV * ENC_MULTIPLIER)//宏的整体要加括号

/** 轮子转一圈，计数器变化多少（= 上面那个 × 减速比） */
#define ENC_COUNTS_PER_WHEEL_REV    (ENC_COUNTS_PER_MOTOR_REV * ENC_GEAR_RATIO)

/** 轮子周长（毫米） */
#define ENC_WHEEL_CIRCUMFERENCE     (3.14159265f * WHEEL_DIAMETER_MM)

/** 一个计数 = 多少毫米（位置分辨率） */
#define ENC_MM_PER_COUNT            (ENC_WHEEL_CIRCUMFERENCE / ENC_COUNTS_PER_WHEEL_REV)


/* ==========================================================================
 * 三、对外接口
 * ========================================================================== */

/**
 * @brief  初始化编码器：启动 TIM5 的编码器接口，把计数器清零
 * @note   在 main() 的 /* USER CODE BEGIN 2 *\/ 里调用一次
 */
void Encoder_Init(void);

/**
 * @brief  更新一次计数（必须周期性调用，比如每 10ms 一次）
 * @note   内部处理了 16 位计数器回绕，正转反转都不会算错
 */
void Encoder_Update(void);

/**
 * @brief  把累计位移清零（重新算里程时用）
 */
void Encoder_Reset(void);

/**
 * @brief  上一次 Update 到现在，转了多少格（带符号，正转正数）
 */
int16_t Encoder_GetDelta(void);

/**
 * @brief  从 Init 到现在，累计转了多少格（带符号）
 */
int32_t Encoder_GetTotal(void);

/**
 * @brief  累计走了多少毫米（带符号，正转正数）
 */
float Encoder_GetDistance_mm(void);

/**
 * @brief  根据两次 Update 的时间间隔，算出当前轮速（mm/s，带符号）
 * @param  dt_ms 两次 Update 之间隔了多少毫秒（比如 10）
 * @note   速度 = 这一段的计数 × 每计数多少毫米 ÷ 时间
 */
float Encoder_GetSpeed_mm_s(uint32_t dt_ms);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
