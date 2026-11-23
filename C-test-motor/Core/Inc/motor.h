#ifndef C_TEST_MOTOR_MOTOR_H
#define C_TEST_MOTOR_MOTOR_H

#include <stdint.h>
#include "main.h"
#include "pid.h"          /* 速度环用通用 PID 模块，电机层不自己实现算法 */

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
 * TIM2 已改为 TIM_ENCODERMODE_TI12（4 倍频）-> 265.2 × 4 = 1060.8
 * 注意：改倍频后这个数必须同步改，否则转速差一倍
 * ------------------------------------------ */
#define MOTOR_ENC_CPR   1060.8f

/* 测速采样周期，必须与实际调用间隔严格一致 */
#define MOTOR_ENC_DT    0.010f

/* 编码器符号：符号统一已经在 encoder.c 的源头做过一次（正转=正计数），
 * 这里保持 +1，避免二次反号。
 * 如果以后把 A/B 线对调、或换电机方向，只需要改 encoder.c 那一处。 */
#define MOTOR_ENC_SIGN  (1.0f)

/* ---------------- 速度环参数 ----------------
 * kp：误差 1 rpm 时给多少千分比。整机开环增益约 0.78 rpm/千分比，
 *     所以 kp 超过 1.28 就会过冲，取 0.8 留足余量。
 * ki：积分项（单位：千分比/秒）。用来消掉纯 P 留下的静差。
 * 先用这组默认值，调的时候一次只动一个。 */
#define MOTOR_KP_DEF    0.8f
#define MOTOR_KI_DEF    4.0f

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

    TIM_HandleTypeDef *enc_tim;   /* 编码器定时器，NULL 表示没有；计数由 encoder.c 统一维护 */
    float              rpm;       /* 输出轴（轮子）转速，滤波后 */
    int32_t            enc_total; /* 累计计数，直线纠偏/里程用 */

    /* ---- 速度环 ---- 追加在末尾，不影响前面字段的偏移 ---- */
    float              rpm_target;/* 目标转速，正数 = 正转 */
    PID_t              speed_pid; /* 这台电机自己的 PID 实例（算法在 pid.c） */
} Motor_t;

extern Motor_t g_motor[MOTOR_NUM];

/* ---------------- 接口 ---------------- */

/* 初始化所有已配置电机：开时钟、配方向脚、启动 PWM */
void Motor_InitAll(void);

/* 使能/失能整片驱动（STBY）。0 = 待机，输出全关 */
void Motor_Enable(uint8_t on);


//中层，斜坡
/* 设置斜坡目标，由 Motor_Update() 逐步逼近（正式控制用） */
void Motor_SetTarget(Motor_t *m, int16_t target);

/* 按固定周期调用，把 duty 朝 target 推进一步 */
void Motor_Update(Motor_t *m);


//底层
/* 直接给定占空比，立即生效（调试用） */
void Motor_SetDuty(Motor_t *m, int16_t duty);

/* 短路制动：IN1=IN2=1，电机两端被短接，停得快 */
void Motor_Brake(Motor_t *m);

/* 滑行：IN1=IN2=0，输出高阻，靠惯性慢慢停 */
void Motor_Coast(Motor_t *m);


//上层
/* ---------------- 上层接口：全部按编号 id ----------------
 * 应用代码（底盘、循迹、比赛逻辑）只调这一组，不碰 Motor_t 指针。
 * id 越界或该路未配置编码器时，函数内部直接返回，不会崩。 */
void    Motor_SetSpeed(uint8_t id, int16_t duty);
int16_t Motor_GetSpeed(uint8_t id);
float   Motor_GetRpm(uint8_t id);
int32_t Motor_GetCount(uint8_t id);

/* ---------------- 以下为内部接口：对单个电机实例操作 ----------------
 * 表驱动遍历时用（for (i...) Motor_UpdateRpm(&g_motor[i])），
 * 上层一般不需要直接调。 */
float Motor_UpdateRpm(Motor_t *m);

/* ---------------- 速度环（PID）接口 ----------------
 * 用法：
 *   1. 上层给目标转速   Motor_SetRpm(0, 200.0f);
 *   2. 每 10 ms 调一次  Motor_SpeedLoop(&g_motor[0], MOTOR_ENC_DT);
 * 注意第 2 步必须紧跟在 Motor_UpdateRpm() 之后，否则用的是上一拍的旧转速。 */

/* 设置目标转速（rpm，正数正转）。会同时清掉积分，避免换目标时带着旧积分冲过头。
 * 用法：Motor_SetRpm(0, 300.0f);   —— 第 0 路追 300 rpm */
void Motor_SetRpm(uint8_t id, float rpm);

/* 内部用：直接对实例操作（表驱动时可以循环传 &g_motor[i]） */
void Motor_SetRpmPtr(Motor_t *m, float rpm);

/* 速度环一步：读 rpm、算 PID、写占空比。dt_s 是调用间隔（秒） */
void Motor_SpeedLoop(Motor_t *m, float dt_s);

#endif // C_TEST_MOTOR_MOTOR_H
