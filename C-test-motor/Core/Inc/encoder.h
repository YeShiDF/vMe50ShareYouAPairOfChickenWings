/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   霍尔编码器驱动（四路，TIM1~TIM4 编码器模式）
  ******************************************************************************
  * 每路编码器 = 一个定时器的 CH1/CH2（TI1/TI2），只能接 CH1/CH2，别接 CH3/CH4。
  * 本板按原理图：
  *   0 : TIM1  PA8 / PA9
  *   1 : TIM5  PA0 / PA1
  *   2 : TIM3  PA6 / PA7
  *   3 : TIM4  PB6 / PB7
  *
  * 这个模块只干三件事：
  *   1. 把四路定时器的编码器接口打开
  *   2. 周期性读回 CNT，算出"这一小段时间转了多少格"
  *   3. 把"格"换算成"毫米"和"毫米每秒"
  *
  * 规则：整个工程里只有本文件允许出现 __HAL_TIM_GET_COUNTER，
  *       其它模块一律通过下面的接口拿数，避免两边各自维护"上一次计数"。
  ******************************************************************************
  */

#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "tim.h"        /* TIM_HandleTypeDef 与 htim1~htim4 */

/* ==========================================================================
 * 一、机械参数 —— 这几个数必须和你车上的真实硬件一致
 * ========================================================================== */

/** 编码器线数：电机转一圈，A 相（或 B 相）输出多少个完整方波周期 */
#define ENC_LINES_PER_MOTOR_REV     13.0f

/** 倍频数：由 CubeMX 里的 Encoder Mode 决定（TI1 and TI2 = x4） */
#define ENC_MULTIPLIER              4.0f

/** 减速比：电机转多少圈，输出轴（轮子）才转 1 圈 */
#define ENC_GEAR_RATIO              20.409f

/** 轮子直径（毫米）。52mm 麦轮 = 52.0f */
#define WHEEL_DIAMETER_MM           52.0f

/** 编码器路数，必须和 g_enc[] 表长、以及实际接线数一致 */
#define ENC_NUM                     4

/** 表示"这一路没有编码器" */
#define ENC_NONE                    0xFFu


/* ==========================================================================
 * 二、派生量 —— 由上面几个数算出来，不用改
 * ========================================================================== */

/** 电机转一圈，计数器变化多少（= 线数 × 倍频） */
#define ENC_COUNTS_PER_MOTOR_REV    (ENC_LINES_PER_MOTOR_REV * ENC_MULTIPLIER)

/** 轮子转一圈，计数器变化多少（= 上面那个 × 减速比） */
#define ENC_COUNTS_PER_WHEEL_REV    (ENC_COUNTS_PER_MOTOR_REV * ENC_GEAR_RATIO)

/** 轮子周长（毫米） */
#define ENC_WHEEL_CIRCUMFERENCE     (3.14159265f * WHEEL_DIAMETER_MM)

/** 一个计数 = 多少毫米（位置分辨率） */
#define ENC_MM_PER_COUNT            (ENC_WHEEL_CIRCUMFERENCE / ENC_COUNTS_PER_WHEEL_REV)


/* ==========================================================================
 * 三、每一路编码器的运行时状态
 * ==========================================================================
 * 表驱动：加一路只改 ENC_NUM 和下面这张表，接口全部按 id 走。
 */
typedef struct {
    TIM_HandleTypeDef *htim;    /* 这一路用哪个定时器 */
    int16_t            last;    /* 上一次读到的 CNT（int16_t，用于处理回绕） */
    int16_t            delta;   /* 上一拍到这一拍转了多少格 */
    int32_t            total;   /* 从 Init 到现在累计多少格 */
} Encoder_t;

extern Encoder_t g_enc[ENC_NUM];


/* ==========================================================================
 * 四、对外接口 —— 上层一律按 id（0~3）调用
 * ========================================================================== */

/** 启动全部编码器接口并清零。main() 里调用一次 */
void Encoder_InitAll(void);

/** 更新全部计数，必须周期性调用（本工程每 10 ms 一次） */
void Encoder_UpdateAll(void);

/** 把某一（或全部）路的累计计数清零 */
void Encoder_Reset(uint8_t id);
void Encoder_ResetAll(void);

/** 上一拍到这一拍转了多少格（带符号，正转为正）；id 非法返回 0 */
int16_t Encoder_GetDelta(uint8_t id);

/** 累计多少格 */
int32_t Encoder_GetTotal(uint8_t id);

/** 累计走了多少毫米 */
float Encoder_GetDistance_mm(uint8_t id);

/** 轮速 mm/s；dt_ms 是两次 Update 的间隔 */
float Encoder_GetSpeed_mm_s(uint8_t id, uint32_t dt_ms);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
