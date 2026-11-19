#ifndef C_TEST_MOTOR_PID_H
#define C_TEST_MOTOR_PID_H

/* ============================================================================
 * 通用 PID 模块
 * ----------------------------------------------------------------------------
 * 这一层只做数学，不知道电机、不知道 HAL、不知道 FreeRTOS。
 * 好处：
 *   1. 换个被控对象（舵机、云台、气泵）可以直接复用；
 *   2. 出问题能单独验证——给它一串 target/meas 就能看出它算得对不对；
 *   3. 想换算法（增量式、模糊、前馈）只改这一个文件，上层不受影响。
 *
 * 用法：
 *     PID_t pid;
 *     PID_Init(&pid, 0.8f, 4.0f, 0.0f, -1000.0f, 1000.0f);
 *     ...
 *     float out = PID_Update(&pid, target, measured, 0.01f);
 * ========================================================================== */

typedef struct {
    /* ---- 三个参数：调的是这三个 ---- */
    float kp;            /* 比例：误差 × kp */
    float ki;            /* 积分：误差 × ki × dt 累加 */
    float kd;            /* 微分：测量值的变化率 × kd */

    /* ---- 内部状态：外部不要直接改 ---- */
    float integral;      /* 积分累计量（已经乘过 ki 和 dt，单位同输出） */
    float prev_meas;     /* 上一拍的测量值（微分作用在测量值上） */
    float d_state;       /* 微分项的低通滤波状态 */
    float d_alpha;       /* 微分低通系数：0~1，越小越平滑、越迟钝 */
    int   first;         /* 首拍标志：第一次调用不产生微分尖峰 */

    /* ---- 输出限幅：被控对象的物理边界 ---- */
    float out_min;
    float out_max;
} PID_t;

/* 初始化：给定参数与输出范围，并清零内部状态 */
void PID_Init(PID_t *p, float kp, float ki, float kd,
              float out_min, float out_max);

/* 在线改参数（调参时不重启程序用） */
void PID_SetGains(PID_t *p, float kp, float ki, float kd);

/* 清零积分和微分状态；换目标/重新起步时调用，避免带着旧记忆 */
void PID_Reset(PID_t *p);

/* 走一步：返回本拍应该输出的量。
 * target : 目标值
 * meas   : 本拍实测值
 * dt_s   : 与上一拍的时间间隔（秒），必须真实，不能写死 */
float PID_Update(PID_t *p, float target, float meas, float dt_s);

#endif // C_TEST_MOTOR_PID_H
