#include "cmsis_os2.h"
#include "motor.h"
#include "encoder.h"        /* Encoder_Update() */
#include "freertos_os2.h"

/* 采样周期：10 个 tick。
 * configTICK_RATE_HZ = 1000 时正好 10 ms，与 motor.h 的 MOTOR_ENC_DT 一致。
 * 用 pdMS_TO_TICKS 而不是写死 10，这样以后改 tick 频率也不会算错转速。 */
#define MTR_SPD_PERIOD_TICKS   pdMS_TO_TICKS(10)

/* 注意：函数名必须和 CubeMX 里 MtrUpdtSpdTask 的配置完全一致。
 * freertos.c 以 extern 方式声明并调用它，签名必须是 void (void *)。 */
void StartTaskMtrGetSpd(void *argument)
{
    (void)argument;

    /* 进循环前先空转一拍：Encoder_Update() 会把当前 CNT 记成基准，
     * 这一次的 delta 不参与计算，避免第一拍算出大得离谱的转速。 */
    Encoder_Update();

    uint32_t wake = osKernelGetTickCount();

    for (;;) {
        /* ---- 先读编码器，再算转速 ---- */
        Encoder_Update();

        for (uint8_t i = 0; i < MOTOR_NUM; i++) {
            if (g_motor[i].enc_tim != NULL) {
                Motor_UpdateRpm(&g_motor[i]);
            }
        }

        /* ---- 绝对周期对齐：累加固定步长，误差不随轮次累积 ---- */
        wake += MTR_SPD_PERIOD_TICKS;

        /* 万一某一轮超时严重，让节拍重新对齐，避免连着追赶好几拍 */
        if ((int32_t)(osKernelGetTickCount() - wake) >= (int32_t)MTR_SPD_PERIOD_TICKS) {
            wake = osKernelGetTickCount();
        }

        osDelayUntil(wake);
    }
}

void StartMtrTestTask(void *argument)
{
    (void)argument;

    /* Motor_InitAll() 已经在 main.c 的 osKernelStart() 之前调用过了。
       这里不要再调：它会重新把 duty/target 清零、重启 PWM，
       打断测速任务已经攒下的节奏。
       只需要把驱动芯片打开就行。 */
    Motor_Enable(1);

    /* ---------- 开环测试 ----------
     * 直接给占空比，不经过 Motor_SetTarget()/Motor_Update() 那一层斜坡。
     * 目的：排除 PID 和斜率限制，单纯验证「驱动板 → 电机 → 编码器」这条链路。
     *
     * 节奏：停 1s → 20% 转 2s → 停 1s → 40% 转 2s → 停 1s，然后循环。
     * 判断标准：40% 时的 rpm 应该大约是 20% 时的 2 倍（空载、开环近似线性）。
     * 任务函数绝对不能 return，否则会进 prvTaskExitError 关中断死循环。 */
    /* ---------- 线性度验证（20% → 30% → 40% 阶梯） ----------
     * 每档转 4 秒，间隔停 1 秒。
     * 空载开环下，转速应该和占空比近似成正比：
     *   20% ≈ 150 rpm   30% ≈ 225 rpm   40% ≈ 300 rpm
     * 如果 40% 明显低于 300（甚至中途停转），说明电源限流了。        */
    for (;;) {
        Motor_SetDuty(&g_motor[0], 0);      /* 停 1 秒，作为参考零点 */
        osDelay(1000);

        Motor_SetDuty(&g_motor[0], 200);    /* 20% */
        osDelay(4000);

        Motor_SetDuty(&g_motor[0], 300);    /* 30% */
        osDelay(4000);

        Motor_SetDuty(&g_motor[0], 400);    /* 40% */
        osDelay(4000);
    }
}
