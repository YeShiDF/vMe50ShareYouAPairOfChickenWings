#include "motor.h"
#include "tim.h"
#include "encoder.h"   /* Encoder_GetDelta()：计数的唯一来源 */

/* ---------------- 电机表 ----------------
 * 加一路电机只要在这里填一行，其余逻辑不用改。
 * htim 填 NULL 的条目会被所有接口自动跳过。
 * 字段顺序：in1口, in1脚, in2口, in2脚, 定时器, 通道,
 *           duty, target, slew, 编码器定时器, rpm, 累计计数
 * ---------------------------------------- */
Motor_t g_motor[MOTOR_NUM] = {
    /* M1：PB15 / PB13 / PA8(TIM1_CH1)，编码器 PA0/PA1(TIM2) */
    { MTR1_IN1_PORT, MTR1_IN1_PIN,
      MTR1_IN2_PORT, MTR1_IN2_PIN,
      &htim1, TIM_CHANNEL_1,
      0, 0, MOTOR_SLEW_DEF,
      &htim2, 0.0f, 0 },

    /* M2 / M3 / M4：接线后照上面的格式补，例如
     * { GPIOB, GPIO_PIN_12, GPIOC, GPIO_PIN_0, &htim1, TIM_CHANNEL_2,
     *   0, 0, MOTOR_SLEW_DEF, NULL, 0.0f }, */
    { 0 },
    { 0 },
    { 0 },
};

//初始化
/* 按端口打开对应 GPIO 时钟，CubeMX 的 MX_GPIO_Init 里没有使能 GPIOB */
static void Motor_Clock(GPIO_TypeDef *port)
{
    if      (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
}

//初始化端口引脚，等效配置MX
/* 方向脚统一配成推挽输出，初始低电平（高阻/滑行态） */
static void Motor_PinInit(Motor_t *m)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    Motor_Clock(m->in1_port);
    gpio.Pin = m->in1_pin;
    HAL_GPIO_Init(m->in1_port, &gpio);

    Motor_Clock(m->in2_port);
    gpio.Pin = m->in2_pin;
    HAL_GPIO_Init(m->in2_port, &gpio);

    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
}

void Motor_InitAll(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* STBY 先配好并拉低，保证初始化期间电机绝对不会动 */
    Motor_Clock(STBY_PORT);
    gpio.Pin   = STBY_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(STBY_PORT, &gpio);
    HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);

    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        Motor_t *m = &g_motor[i];
        if (m->htim == NULL) continue;//跳过没接线的

        Motor_PinInit(m);
        m->duty   = 0;
        m->target = 0;

        HAL_TIM_PWM_Start(m->htim, m->channel);

        /* 编码器不在这里启动：由 main.c 里的 Encoder_Init() 统一负责。
           两边都启动/清零同一个 TIM2，会让对方的"上一拍计数"失效。 */
        m->rpm = 0.0f;
        m->rpm_target = 0.0f;         /* 速度环：目标转速清零 */

        /* 速度环：给这台电机装一个 PID 实例。
         * 参数来自 motor.h 的默认宏，输出范围就是占空比范围 ±1000 */
        PID_Init(&m->speed_pid, MOTOR_KP_DEF, MOTOR_KI_DEF, 0.0f,
                 -(float)MOTOR_DUTY_MAX, (float)MOTOR_DUTY_MAX);
    }
}

void Motor_Enable(uint8_t on)//作用：打开或者关闭总使能
{
    HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (!on) {
        for (uint8_t i = 0; i < MOTOR_NUM; i++) {
            if (g_motor[i].htim != NULL) Motor_Coast(&g_motor[i]);
        }
    }
}

void Motor_SetDuty(Motor_t *m, int16_t duty)//限幅，正转反转滑行，改变占空比
{
    if (m == NULL || m->htim == NULL) return;

    if (duty >  MOTOR_DUTY_MAX) duty =  MOTOR_DUTY_MAX;
    if (duty < -MOTOR_DUTY_MAX) duty = -MOTOR_DUTY_MAX;
    m->duty = duty;

    /* 正转 IN1=H / IN2=L；反转相反；0 表示两脚都低 -> 高阻滑行 */
    if (duty > 0) {
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
    } else if (duty < 0) {
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
    }

    /* 千分比换算成 CCR，自动适配当前 ARR，换定时器或改频率都不用动这里 */
    uint32_t span = __HAL_TIM_GET_AUTORELOAD(m->htim) + 1u;
    uint32_t mag  = (uint32_t)(duty >= 0 ? duty : -duty);
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, mag * span / MOTOR_DUTY_MAX);
    //CCR = |duty| × (ARR+1) / 1000
}

void Motor_SetTarget(Motor_t *m, int16_t target)//记下目标值并限幅
{
    if (m == NULL || m->htim == NULL) return;

    if (target >  MOTOR_DUTY_MAX) target =  MOTOR_DUTY_MAX;
    if (target < -MOTOR_DUTY_MAX) target = -MOTOR_DUTY_MAX;
    m->target = target;
}

void Motor_Update(Motor_t *m)//（slew限制）
{
    if (m == NULL || m->htim == NULL) return;

    int16_t slew = (int16_t)m->slew;
    if (slew <= 0) slew = MOTOR_SLEW_DEF;

    int16_t diff = (int16_t)(m->target - m->duty);
    if (diff >  slew) diff =  slew;
    if (diff < -slew) diff = -slew;
    if (diff == 0) return;

    int16_t next = (int16_t)(m->duty + diff);

    /* 换向必须经过 0：先停一拍让电流衰减，避免直接反接的电流冲击 */
    if ((m->duty > 0 && next < 0) || (m->duty < 0 && next > 0)) {
        Motor_SetDuty(m, 0);
        return;
    }

    Motor_SetDuty(m, next);
}

void Motor_Brake(Motor_t *m)
{
    if (m == NULL || m->htim == NULL) return;

    __HAL_TIM_SET_COMPARE(m->htim, m->channel, 0);
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);

    m->duty   = 0;
    m->target = 0;
}

void Motor_Coast(Motor_t *m)
{
    if (m == NULL || m->htim == NULL) return;

    __HAL_TIM_SET_COMPARE(m->htim, m->channel, 0);
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);

    m->duty   = 0;
    m->target = 0;
}

void Motor_SetSpeed(uint8_t id, int16_t duty)
{
    if (id >= MOTOR_NUM) return;
    Motor_SetDuty(&g_motor[id], duty);
}

int16_t Motor_GetSpeed(uint8_t id)
{
    if (id >= MOTOR_NUM) return 0;
    return g_motor[id].duty;
}

/* 测速：按固定周期调用（周期必须等于 MOTOR_ENC_DT），不阻塞
 *
 *   rpm = Δcnt × 60 / (CPR × Δt)
 *       = Δcnt × 60 / (1060.8 × 0.01)
 *       = Δcnt × 5.656
 */
float Motor_UpdateRpm(Motor_t *m)
{
    if (m == NULL || m->enc_tim == NULL) return 0.0f;

    /* 计数只从 encoder.c 取：读 CNT、处理回绕都归它管，
       这里拿到的就是"上一拍到现在转了多少格" */
    int16_t d = Encoder_GetDelta();
    m->enc_total += d;

    /* 符号统一：本车正占空比 → 负计数，乘 MOTOR_ENC_SIGN 翻正，
     * 这样「目标 200 rpm、实测 200 rpm」时误差真的是 0 */
    float raw = MOTOR_ENC_SIGN * (float)d * 60.0f / (MOTOR_ENC_CPR * MOTOR_ENC_DT);
    m->rpm += (raw - m->rpm) * 0.4f;              /* 一阶低通，≈25 ms 滞后 */
    return m->rpm;
}

float Motor_GetRpm(uint8_t id)
{
    if (id >= MOTOR_NUM) return 0.0f;
    return g_motor[id].rpm;
}

int32_t Motor_GetCount(Motor_t *m)
{
    if (m == NULL || m->enc_tim == NULL) return 0;
    return m->enc_total;
}

/* ============================================================
 *                        速度环（PID）
 * ------------------------------------------------------------
 * 为什么是「位置式」而不是「增量式」：
 *   位置式直接算 final = P + I，逻辑直观、好调；
 *   增量式算 Δduty，抗饱和更自然但公式绕。先学位置式。
 *
 * 为什么一开始没有 D：
 *   转速是编码器差出来的，本身带量化噪声，D 项对噪声放大 10 倍；
 *   而电机有惯性，本身就是一阶系统，P + I 已经够用。
 *   等以后做位置环（走直线）时再加 D，那是另一回事。
 * ============================================================ */

void Motor_SetRpm(Motor_t *m, float rpm)
{
    if (m == NULL) return;
    m->rpm_target = rpm;

    /* 换目标时清掉 PID 状态：否则从 300 rpm 掉到 0 时，
     * 积分里还存着「该给 300 多千分比」的记忆，会顶住刹车不放 */
    PID_Reset(&m->speed_pid);
}

void Motor_SpeedLoop(Motor_t *m, float dt_s)
{
    if (m == NULL || m->enc_tim == NULL) return;

    /* 这一层不再有任何控制算法，只做三件事：
     *   把「目标、实测、时间」交给 PID，把算出来的量写给电机。
     * 算法相关的抗饱和、微分滤波、限幅全部在 pid.c 里。 */
    float out = PID_Update(&m->speed_pid, m->rpm_target, m->rpm, dt_s);

    /* 直接写占空比。不走 Motor_Update() 的斜坡，
     * 因为斜坡会额外引入 167 ms 的滞后，PID 会因此振荡 */
    Motor_SetDuty(m, (int16_t)out);
}