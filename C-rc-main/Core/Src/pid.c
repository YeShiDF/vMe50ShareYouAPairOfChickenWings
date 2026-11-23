#include "pid.h"

/* ============================================================================
 * 位置式 PID 实现
 * ------------------------------------------------------------
 *   out = kp·e  +  ki·∫e·dt  +  kd·de/dt
 *
 * 三个关键工程处理，初学最容易漏：
 *   1. 积分抗饱和（anti-windup）：输出被限幅时，积分不许继续涨
 *   2. 微分作用在「测量值」而不是「误差」：目标突变时不会打出一个无穷大的尖峰
 *   3. 首拍不做微分：否则 prev_meas 是 0，第一拍微分等于整个初值 × kd，直接爆
 * ========================================================================== */

void PID_Init(PID_t *p, float kp, float ki, float kd,
              float out_min, float out_max)
{
    if (p == 0) return;

    p->kp = kp;
    p->ki = ki;
    p->kd = kd;

    p->out_min = out_min;
    p->out_max = out_max;

    p->integral = 0.0f;
    p->prev_meas = 0.0f;
    p->d_state = 0.0f;
    p->d_alpha = 0.25f;   /* 微分低通：约 4 拍平滑，抑制编码器量化噪声 */
    p->first = 1;
}

void PID_SetGains(PID_t *p, float kp, float ki, float kd)
{
    if (p == 0) return;
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
}

void PID_Reset(PID_t *p)
{
    if (p == 0) return;
    p->integral = 0.0f;
    p->d_state = 0.0f;
    p->prev_meas = 0.0f;
    p->first = 1;
}

float PID_Update(PID_t *p, float target, float meas, float dt_s)
{
    if (p == 0 || dt_s <= 0.0f) return 0.0f;

    /* ---- 1. 比例项 ---- */
    float err = target - meas;
    float p_term = p->kp * err;

    /* ---- 2. 积分项 ---- */
    p->integral += p->ki * err * dt_s;

    /* ---- 3. 微分项（作用在测量值上，加低通）---- */
    float d_term = 0.0f;
    if (p->kd != 0.0f) {
        if (p->first) {
            /* 首拍只记基准，不算微分，避免开环那一刻的假尖峰 */
            p->first = 0;
        } else {
            /* 误差的导数 = -(测量值的导数)，所以这里取负号 */
            float d_raw = -(meas - p->prev_meas) / dt_s;
            p->d_state += (d_raw - p->d_state) * p->d_alpha;  /* 一阶低通 */
            d_term = p->kd * p->d_state;
        }
    }

    /* ---- 4. 合成并限幅 ---- */
    float out = p_term + p->integral + d_term;

    if (out > p->out_max) out = p->out_max;
    if (out < p->out_min) out = p->out_min;

    /* ---- 5. 抗饱和：把积分修正成「不超出当前输出」的水平 ----
     * 好处是输出一旦饱和，积分就不会继续累积；
     * 反向时能立刻退出饱和，不会拖好几秒。 */
    p->integral = out - p_term - d_term;

    /* ---- 6. 保存本拍测量值，供下一拍算微分 ---- */
    p->prev_meas = meas;

    return out;
}
