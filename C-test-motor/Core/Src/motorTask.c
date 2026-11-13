#include "cmsis_os2.h"
#include "motor.h"
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

    /* 进入循环前重新对齐计数快照。
     * Motor_InitAll() 里记的 enc_last 到现在可能已经过了几百毫秒，
     * 不重新对齐的话第一拍会算出一个大得离谱的差值。 */
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        if (g_motor[i].enc_tim != NULL) {
            g_motor[i].enc_last = (int16_t)__HAL_TIM_GET_COUNTER(g_motor[i].enc_tim);
            g_motor[i].rpm      = 0.0f;
        }
    }

    uint32_t wake = osKernelGetTickCount();

    for (;;) {
        /* ---- 采样：每路有编码器的电机更新一次转速 ---- */
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
       还会重置 enc_last，让测速任务丢一拍。
       只需要把驱动芯片打开就行。 */
    Motor_Enable(1);

    Motor_SetTarget(&g_motor[0], 1000);

    /* 任务函数绝对不能返回。一旦 return，FreeRTOS 会进入
       prvTaskExitError：关中断 + 死循环，整个系统直接停摆。 */
    for (;;) {
        Motor_Update(&g_motor[0]);   /* 每 10 ms 朝 target 推进一步 */
        osDelay(10);
    }
}
