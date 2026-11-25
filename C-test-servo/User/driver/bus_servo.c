/**
  ******************************************************************************
  * @file    bus_servo.c
  * @brief   驱动层实现：把"ID/PWM/时间"翻译成 ASCII 帧，把回复解析成数字。
  ******************************************************************************
  */
#include "bus_servo.h"
#include "bsp_uart.h"
#include <stdio.h>
#include <string.h>

/* 发一条命令最长等多久 */
#define CMD_TIMEOUT_MS      (20U)
/* 等舵机回复最长等多久（舵机处理需要几毫秒） */
#define REPLY_TIMEOUT_MS    (30U)

/* ============================ 内部工具 ============================ */

/**
  * @brief  发一条命令，并把半双工回显丢掉。
  */
static bus_servo_status_t send_cmd(const char *cmd)
{
  uint16_t len;

  if (cmd == NULL)
  {
    return BUS_SERVO_ERR_PARAM;
  }

  len = (uint16_t)strlen(cmd);
  if (len == 0U)
  {
    return BUS_SERVO_ERR_PARAM;
  }

  if (BspUart_Send((const uint8_t *)cmd, len, CMD_TIMEOUT_MS) != BSP_UART_OK)
  {
    return BUS_SERVO_ERR_SEND;
  }

  /* 半双工：自己发出去的字节会被自己收到（回显），这里先清掉，
     否则后面读到的"第一条回复"其实是自己发的命令 */
  BspUart_DropRx();

  return BUS_SERVO_OK;
}

/**
  * @brief  一直收到帧尾 '!' 为止。
  * @note   帧头之前收到的杂字节会被丢弃。
  */
static bus_servo_status_t read_reply(char *buf, uint8_t max_len, uint32_t timeout_ms)
{
  uint8_t  ch;
  uint8_t  n = 0U;
  uint32_t start = BspUart_Millis();

  while ((BspUart_Millis() - start) < timeout_ms)
  {
    if (BspUart_Recv(&ch, 1U, 1U) != BSP_UART_OK)
    {
      continue;
    }
    //用循环的方式一个字节一个字节接受字符

    if ((n == 0U) && (ch != '#'))
    {
      continue;                      /* 还没看到帧头，丢掉 */
    }

    if (n >= (uint8_t)(max_len - 1U))
    {
      return BUS_SERVO_ERR_FORMAT;   /* 太长了，肯定不对 */
    }

    buf[n] = (char)ch;
    n++;

    if (ch == '!')                   /* 收到帧尾，结束 */
    {
      buf[n] = '\0';
      return BUS_SERVO_OK;
    }
  }

  return BUS_SERVO_ERR_TIMEOUT;
}

/**
  * @brief  把连续 digits 个 ASCII 数字转成整数。失败返回 -1。
  */
static int32_t parse_digits(const char *s, uint8_t digits)
{
  int32_t value = 0;
  uint8_t i;

  for (i = 0U; i < digits; i++)
  {
    if ((s[i] < '0') || (s[i] > '9'))
    {
      return -1;
    }
    value = value * 10 + (s[i] - '0');
  }
  return value;
}

/**
  * @brief  解析 "28.1" 这种带一位小数的值，结果放大 10 倍（281）。
  * @param  end 输出：解析结束的位置
  */
static int32_t parse_tenths(const char *s, const char **end)
{
  int32_t value = 0;
  int32_t tenth = 0;
  uint8_t has_dot = 0U;

  while ((*s >= '0') && (*s <= '9'))
  {
    value = value * 10 + (*s - '0');
    s++;
  }

  if (*s == '.')
  {
    s++;
    has_dot = 1U;
    if ((*s >= '0') && (*s <= '9'))
    {
      tenth = (*s - '0');
      s++;
    }
  }

  *end = s;
  return (value * 10) + (has_dot ? tenth : 0);
}

/* ============================ 对外接口 ============================ */

void BusServo_Init(UART_HandleTypeDef *huart)
{
  BspUart_Init(huart);
}

bus_servo_status_t BusServo_SetPwm(uint8_t id, uint16_t pwm, uint16_t time_ms)
{
  char cmd[24];
  int  n;

  if (id > BUS_SERVO_ID_MAX)
  {
    return BUS_SERVO_ERR_PARAM;
  }
  if ((pwm < BUS_SERVO_PWM_MIN) || (pwm > BUS_SERVO_PWM_MAX))
  {
    return BUS_SERVO_ERR_PARAM;
  }
  if (time_ms > BUS_SERVO_TIME_MAX)
  {
    return BUS_SERVO_ERR_PARAM;
  }

  /* 例：id=1, pwm=1500, time=1000 -> "#001P1500T1000!" */
  n = snprintf(cmd, sizeof(cmd), "#%03uP%04uT%04u!", id, pwm, time_ms);
  if ((n <= 0) || (n >= (int)sizeof(cmd)))
  {
    return BUS_SERVO_ERR_FORMAT;
  }

  return send_cmd(cmd);
}

bus_servo_status_t BusServo_SetPwmSync(const uint8_t *ids,
                                       const uint16_t *pwms,
                                       uint8_t count,
                                       uint16_t time_ms)
{
  char    cmd[128];
  int     len;
  int     n;
  uint8_t i;

  if ((ids == NULL) || (pwms == NULL) || (count == 0U))
  {
    return BUS_SERVO_ERR_PARAM;
  }
  if (time_ms > BUS_SERVO_TIME_MAX)
  {
    return BUS_SERVO_ERR_PARAM;
  }

  /* 多舵机指令头部固定为 {G0000 */
  n = snprintf(cmd, sizeof(cmd), "{G0000");
  if (n <= 0)
  {
    return BUS_SERVO_ERR_FORMAT;
  }
  len = n;

  for (i = 0U; i < count; i++)
  {
    if (ids[i] > BUS_SERVO_ID_MAX)
    {
      return BUS_SERVO_ERR_PARAM;
    }
    if ((pwms[i] < BUS_SERVO_PWM_MIN) || (pwms[i] > BUS_SERVO_PWM_MAX))
    {
      return BUS_SERVO_ERR_PARAM;
    }

    n = snprintf(cmd + len, sizeof(cmd) - (size_t)len,
                 "#%03uP%04uT%04u!", ids[i], pwms[i], time_ms);
    if ((n <= 0) || ((size_t)n >= (sizeof(cmd) - (size_t)len)))
    {
      return BUS_SERVO_ERR_FORMAT;
    }
    len += n;
  }

  if ((size_t)len + 2U > sizeof(cmd))
  {
    return BUS_SERVO_ERR_FORMAT;
  }

  cmd[len] = '}';                      /* 整条指令以 } 收尾 */
  len++;
  cmd[len] = '\0';

  return send_cmd(cmd);
}

bus_servo_status_t BusServo_Release(uint8_t id)
{
  char cmd[16];

  if (id > BUS_SERVO_ID_MAX)
  {
    return BUS_SERVO_ERR_PARAM;
  }

  snprintf(cmd, sizeof(cmd), "#%03uPULK!", id);
  return send_cmd(cmd);
}

bus_servo_status_t BusServo_Restore(uint8_t id)
{
  char cmd[16];

  if (id > BUS_SERVO_ID_MAX)
  {
    return BUS_SERVO_ERR_PARAM;
  }

  snprintf(cmd, sizeof(cmd), "#%03uPULR!", id);
  return send_cmd(cmd);
}

bus_servo_status_t BusServo_ReadPwm(uint8_t id, uint16_t *pwm)
{
  char  cmd[16];
  char  rsp[BUS_SERVO_FRAME_MAX];
  bus_servo_status_t st;
  int32_t value;

  if ((id > BUS_SERVO_ID_MAX) || (pwm == NULL))
  {
    return BUS_SERVO_ERR_PARAM;
  }

  snprintf(cmd, sizeof(cmd), "#%03uPRAD!", id);

  st = send_cmd(cmd);
  if (st != BUS_SERVO_OK)
  {
    return st;
  }

  st = read_reply(rsp, (uint8_t)sizeof(rsp), REPLY_TIMEOUT_MS);
  if (st != BUS_SERVO_OK)
  {
    return st;
  }

  /* 期望格式：#000P1500!  长度至少 10，第 5 个字符是 'P' */
  if (strlen(rsp) < 10U)
  {
    return BUS_SERVO_ERR_FORMAT;
  }
  if ((rsp[4] != 'P') || (rsp[9] != '!'))
  {
    return BUS_SERVO_ERR_FORMAT;
  }

  value = parse_digits(&rsp[5], 4U);
  if ((value < (int32_t)BUS_SERVO_PWM_MIN) || (value > (int32_t)BUS_SERVO_PWM_MAX))
  {
    return BUS_SERVO_ERR_FORMAT;
  }

  *pwm = (uint16_t)value;
  return BUS_SERVO_OK;
}

bus_servo_status_t BusServo_ReadTempVolt(uint8_t id, int16_t *temp_x10, int16_t *volt_x10)
{
  char  cmd[16];
  char  rsp[BUS_SERVO_FRAME_MAX];
  bus_servo_status_t st;
  const char *p;
  const char *q;
  int32_t tmp;
  int32_t vol;

  if ((id > BUS_SERVO_ID_MAX) || (temp_x10 == NULL) || (volt_x10 == NULL))
  {
    return BUS_SERVO_ERR_PARAM;
  }

  snprintf(cmd, sizeof(cmd), "#%03uPRTV!", id);

  st = send_cmd(cmd);
  if (st != BUS_SERVO_OK)
  {
    return st;
  }

  st = read_reply(rsp, (uint8_t)sizeof(rsp), REPLY_TIMEOUT_MS);
  if (st != BUS_SERVO_OK)
  {
    return st;
  }

  /* 期望格式：#000T28.1V7.4! */
  p = strchr(rsp, 'T');
  if (p == NULL)
  {
    return BUS_SERVO_ERR_FORMAT;
  }

  tmp = parse_tenths(p + 1, &q);
  if ((q == NULL) || (*q != 'V'))
  {
    return BUS_SERVO_ERR_FORMAT;
  }

  vol = parse_tenths(q + 1, &q);

  *temp_x10 = (int16_t)tmp;
  *volt_x10 = (int16_t)vol;
  return BUS_SERVO_OK;
}

bus_servo_status_t BusServo_Ping(uint8_t id)
{
  char  cmd[16];
  char  rsp[BUS_SERVO_FRAME_MAX];
  bus_servo_status_t st;

  if (id > BUS_SERVO_ID_MAX)
  {
    return BUS_SERVO_ERR_PARAM;
  }

  snprintf(cmd, sizeof(cmd), "#%03uPID!", id);

  st = send_cmd(cmd);
  if (st != BUS_SERVO_OK)
  {
    return st;
  }

  /* 存在则回 #000P!，不存在则一直没回复 -> 超时 */
  return read_reply(rsp, (uint8_t)sizeof(rsp), REPLY_TIMEOUT_MS);
}

bus_servo_status_t BusServo_SetMode(uint8_t id, uint8_t mode)
{
  char  cmd[16];
  char  rsp[BUS_SERVO_FRAME_MAX];
  bus_servo_status_t st;

  if ((id > BUS_SERVO_ID_MAX) || (mode < 1U) || (mode > 8U))
  {
    return BUS_SERVO_ERR_PARAM;
  }

  snprintf(cmd, sizeof(cmd), "#%03uPMOD%u!", id, mode);

  st = send_cmd(cmd);
  if (st != BUS_SERVO_OK)
  {
    return st;
  }

  return read_reply(rsp, (uint8_t)sizeof(rsp), REPLY_TIMEOUT_MS);
}

bus_servo_status_t BusServo_SetId(uint8_t old_id, uint8_t new_id)
{
  char  cmd[16];
  char  rsp[BUS_SERVO_FRAME_MAX];
  bus_servo_status_t st;

  if ((old_id > BUS_SERVO_ID_MAX) || (new_id > BUS_SERVO_ID_MAX))
  {
    return BUS_SERVO_ERR_PARAM;
  }

  snprintf(cmd, sizeof(cmd), "#%03uPID%03u!", old_id, new_id);

  st = send_cmd(cmd);
  if (st != BUS_SERVO_OK)
  {
    return st;
  }

  return read_reply(rsp, (uint8_t)sizeof(rsp), REPLY_TIMEOUT_MS);
}
