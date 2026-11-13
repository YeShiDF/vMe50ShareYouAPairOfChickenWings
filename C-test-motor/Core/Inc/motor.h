#ifndef C_TEST_MOTOR_MOTOR_H
#define C_TEST_MOTOR_MOTOR_H

#include <stdint.h>
#include "main.h"

/* ---------------- 引脚分配 ----------------
 * 每一路电机 = 2 根方向线 + 1 路 PWM
 * PWM 必须来自定时器通道，方向上电前先拉低
 * ------------------------------------------ */

/* M1：已接线（PB15 / PB13 / PA8-TIM1_CH1） */
#define MTR1_IN1_PORT   GPIOB
#define MTR1_IN1_PIN    GPIO_PIN_15
#define MTR1_IN2_PORT   GPIOB
#define MTR1_IN2_PIN    GPIO_PIN_13

/* STBY：一片 TB6612 共用一个，高电平工作 */
#define STBY_PORT       GPIOB
#define STBY_PIN        GPIO_PIN_14

/* ---------------- 参数 ---------------- */

/* 占空比量程：内部统一用千分比，对外不暴露 CCR */
#define MOTOR_DUTY_MAX  1000

/* 电机数量，与 g_motor[] 的表长一致 */
#define MOTOR_NUM       4

/* 默认斜坡：每次 Motor_Update() 最多变化多少千分比
 * 按 100 Hz 调用时，60 表示满量程约 167 ms 走完 */
#define MOTOR_SLEW_DEF  60

/* ---------------- 编码器参数 ----------------
 * 规格书：减速后编码器线数 265.2 线/输出轴圈
 * TIM5 当前是 TIM_ENCODERMODE_TI2（2 倍频）-> 265.2 × 2 = 530.4
 * 若在 CubeMX 里改成 TI12（4 倍频），这里必须改成 1060.8，否则转速差一倍
 * ------------------------------------------ */
#define MOTOR_ENC_CPR   530.4f

/* 测速采样周期，必须与实际调用间隔严格一致 */
#define MOTOR_ENC_DT    0.010f

/* ---------------- 数据结构 ---------------- */

typedef struct {
    GPIO_TypeDef      *in1_port;
    uint16_t           in1_pin;
    GPIO_TypeDef      *in2_port;
    uint16_t           in2_pin;

    TIM_HandleTypeDef *htim;      /* 填 NULL 表示这一路还没接线，全部接口自动跳过 */
    uint32_t           channel;

    int16_t            duty;      /* 当前实际输出，-1000 ~ +1000 */
    int16_t            target;    /* 斜坡目标值 */
    uint16_t           slew;      /* 每次 Update 的最大步长 */

    TIM_HandleTypeDef *enc_tim;   /* 编码器定时器，NULL 表示没有 */
    int16_t            enc_last;  /* 上一拍计数快照 */
    float              rpm;       /* 输出轴（轮子）转速，滤波后 */
    int32_t            enc_total; /* 累计计数，直线纠偏/里程用 */
} Motor_t;

extern Motor_t g_motor[MOTOR_NUM];

/* ---------------- 接口 ---------------- */

/* 初始化所有已配置电机：开时钟、配方向脚、启动 PWM */
void Motor_InitAll(void);

/* 使能/失能整片驱动（STBY）。0 = 待机，输出全关 */
void Motor_Enable(uint8_t on);

/* 直接给定占空比，立即生效（调试用） */
void Motor_SetDuty(Motor_t *m, int16_t duty);

/* 设置斜坡目标，由 Motor_Update() 逐步逼近（正式控制用） */
void Motor_SetTarget(Motor_t *m, int16_t target);

/* 按固定周期调用，把 duty 朝 target 推进一步 */
void Motor_Update(Motor_t *m);

/* 短路制动：IN1=IN2=1，电机两端被短接，停得快 */
void Motor_Brake(Motor_t *m);

/* 滑行：IN1=IN2=0，输出高阻，靠惯性慢慢停 */
void Motor_Coast(Motor_t *m);

/* 按编号操作，供上层运动学层使用 */
void Motor_SetSpeed(uint8_t id, int16_t duty);
int16_t Motor_GetSpeed(uint8_t id);

/* 测速：按固定周期（MOTOR_ENC_DT）调用，返回输出轴 rpm，带符号 */
float Motor_UpdateRpm(Motor_t *m);
float Motor_GetRpm(uint8_t id);

/* 里程：累计计数直接换算，用于直线纠偏 */
int32_t Motor_GetCount(Motor_t *m);

#endif // C_TEST_MOTOR_MOTOR_H
