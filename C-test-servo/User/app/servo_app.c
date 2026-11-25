/**
  ******************************************************************************
  * @file    servo_app.c
  * @brief   应用层实现：角度 -> PWM -> 同步帧，中间带限速。
  ******************************************************************************
  */
#include "servo_app.h"

/* 上电默认参数，改这里就行，不用动逻辑 */
static servo_unit_t s_servo[SERVO_NUM] =
{
  { .id = 1U, .range_deg = 270U, .offset_deg = 0, .speed_dps = 300U, .cur_deg = 135, .tgt_deg = 135 },
  { .id = 2U, .range_deg = 270U, .offset_deg = 0, .speed_dps = 300U, .cur_deg = 135, .tgt_deg = 135 },
  { .id = 3U, .range_deg = 270U, .offset_deg = 0, .speed_dps = 300U, .cur_deg = 135, .tgt_deg = 135 },
  { .id = 4U, .range_deg = 270U, .offset_deg = 0, .speed_dps = 300U, .cur_deg = 135, .tgt_deg = 135 }
};

/* 最近一次总线操作的结果，调试时能看到"到底哪一步失败" */
static bus_servo_status_t s_last_status = BUS_SERVO_OK;

/* ============================ 内部函数 ============================ */

/**
  * @brief  角度 -> PWM 脉宽。
  * @note   行程 0~range_deg 线性映射到 500~2500。
  */
static uint16_t deg_to_pwm(const servo_unit_t *u, int16_t deg)
{
  int32_t d    = (int32_t)deg + (int32_t)u->offset_deg;
  int32_t span = (int32_t)u->range_deg;
  int32_t pwm;

  if (span <= 0)
  {
    return BUS_SERVO_PWM_MIN;
  }

  /* 先夹到机械行程内，防止算出发散的值把舵机顶到限位 */
  if (d < 0)
  {
    d = 0;
  }
  if (d > span)
  {
    d = span;
  }

  /* 加 span/2 是为了四舍五入，避免整数除法总是往下取整 */
  pwm = (int32_t)BUS_SERVO_PWM_MIN + (((d * 2000) + (span / 2)) / span);

  if (pwm < (int32_t)BUS_SERVO_PWM_MIN)
  {
    pwm = (int32_t)BUS_SERVO_PWM_MIN;
  }
  if (pwm > (int32_t)BUS_SERVO_PWM_MAX)
  {
    pwm = (int32_t)BUS_SERVO_PWM_MAX;
  }

  return (uint16_t)pwm;
}

/**
  * @brief  每个周期能走多少度。
  */
static int16_t step_per_tick(const servo_unit_t *u)
{
  int16_t step = (int16_t)(((uint32_t)u->speed_dps + 50U) / 100U);

  if (step < 1)
  {
    step = 1;      /* 再慢也得走，否则永远到不了 */
  }
  return step;
}

/* ============================ 对外接口 ============================ */

void ServoApp_Init(UART_HandleTypeDef *huart)
{
  uint8_t  ids[SERVO_NUM];
  uint16_t pwms[SERVO_NUM];
  uint8_t  i;

  BusServo_Init(huart);

  /* 上电先摆到行程中点，让机械结构有个确定的基准 */
  for (i = 0U; i < SERVO_NUM; i++)
  {
    s_servo[i].cur_deg = (int16_t)(s_servo[i].range_deg / 2U);
    s_servo[i].tgt_deg = s_servo[i].cur_deg;

    ids[i]  = s_servo[i].id;
    pwms[i] = deg_to_pwm(&s_servo[i], s_servo[i].cur_deg);
  }

  s_last_status = BusServo_SetPwmSync(ids, pwms, SERVO_NUM, 800U);
}

void ServoApp_Task(void)
{
  uint8_t  ids[SERVO_NUM];
  uint16_t pwms[SERVO_NUM];
  uint8_t  count = 0U;
  uint8_t  i;

  for (i = 0U; i < SERVO_NUM; i++)
  {
    int16_t step;
    uint8_t moved = 0U;

    if (s_servo[i].cur_deg == s_servo[i].tgt_deg)
    {
      continue;                    /* 已经到位，这个周期不用管它 */
    }

    step = step_per_tick(&s_servo[i]);

    if (s_servo[i].cur_deg < s_servo[i].tgt_deg)
    {
      s_servo[i].cur_deg += step;
      if (s_servo[i].cur_deg > s_servo[i].tgt_deg)
      {
        s_servo[i].cur_deg = s_servo[i].tgt_deg;
      }
      moved = 1U;
    }
    else
    {
      s_servo[i].cur_deg -= step;
      if (s_servo[i].cur_deg < s_servo[i].tgt_deg)
      {
        s_servo[i].cur_deg = s_servo[i].tgt_deg;
      }
      moved = 1U;
    }

    if (moved != 0U)
    {
      ids[count]  = s_servo[i].id;
      pwms[count] = deg_to_pwm(&s_servo[i], s_servo[i].cur_deg);
      count++;
    }
  }

  if (count == 0U)
  {
    return;                          /* 都不动就不占总线带宽 */
  }

  /* 注意：这里是一帧发完，不是分四次发。
     分四次发四颗舵机会先后启动，机械臂会扭一下 */
  s_last_status = BusServo_SetPwmSync(ids, pwms, count, SERVO_MOVE_TIME_MS);
}

void ServoApp_SetAngle(uint8_t idx, int16_t deg)
{
  if (idx >= SERVO_NUM)
  {
    return;
  }

  if (deg < 0)
  {
    deg = 0;
  }
  if (deg > (int16_t)s_servo[idx].range_deg)
  {
    deg = (int16_t)s_servo[idx].range_deg;
  }

  s_servo[idx].tgt_deg = deg;
}

void ServoApp_SetAll(const int16_t *deg)
{
  uint8_t i;

  if (deg == NULL)
  {
    return;
  }

  for (i = 0U; i < SERVO_NUM; i++)
  {
    ServoApp_SetAngle(i, deg[i]);
  }
}

int16_t ServoApp_GetAngle(uint8_t idx)
{
  if (idx >= SERVO_NUM)
  {
    return 0;
  }
  return s_servo[idx].cur_deg;
}

uint8_t ServoApp_ScanPresent(void)//检测是否离线
{
  uint8_t  mask = 0U;
  uint8_t  i;

  for (i = 0U; i < SERVO_NUM; i++)
  {
    if (BusServo_Ping(s_servo[i].id) == BUS_SERVO_OK)
    {
      mask |= (uint8_t)(1U << i);
    }
  }

  return mask;
}

bus_servo_status_t ServoApp_LastStatus(void)//获取上一次舵机状态
{
  return s_last_status;
}
