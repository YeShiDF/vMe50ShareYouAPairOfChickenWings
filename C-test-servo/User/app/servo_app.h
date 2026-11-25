/**
  ******************************************************************************
  * @file    servo_app.h
  * @brief   应用层：用"角度"和"第几号舵机"说话。
  *          只依赖 driver 层，不碰 UART，也不碰 HAL。
  ******************************************************************************
  */
#ifndef __SERVO_APP_H
#define __SERVO_APP_H

#include "bus_servo.h"
#include <stdint.h>

/* 车上装几颗舵机 */
#define SERVO_NUM            (4U)

/* 每 10ms 调用一次 ServoApp_Task，这个常量要和调用周期一致 */
#define SERVO_TASK_PERIOD_MS (10U)

/* 一次同步动作给舵机的执行时间，比周期略长一点，动作才连贯 */
#define SERVO_MOVE_TIME_MS   (15U)

typedef struct
{
  uint8_t  id;          /* 舵机 ID，1~4 */
  uint16_t range_deg;   /* 机械行程：270 或 180 */
  int16_t  offset_deg;  /* 安装偏差补偿：正数表示实际角度比算出来的大 */
  uint16_t speed_dps;   /* 限速：度/秒 */
  int16_t  cur_deg;     /* 当前角度（软件推算，不是回读值） */
  int16_t  tgt_deg;     /* 目标角度 */
} servo_unit_t;

/**
  * @brief  初始化：绑定串口，把四颗舵机摆到行程中点。
  * @param  huart 舵机总线串口（本工程为 &huart3）
  */
void ServoApp_Init(UART_HandleTypeDef *huart);

/**
  * @brief  周期任务：必须每 10ms 调用一次（放在 SysTick 回调或主循环里）。
  */
void ServoApp_Task(void);

/**
  * @brief  设置某一颗的目标角度（度）。
  * @param  idx 0 ~ SERVO_NUM-1
  */
void ServoApp_SetAngle(uint8_t idx, int16_t deg);

/**
  * @brief  一次性设置全部目标角度，数组长度 SERVO_NUM。
  */
void ServoApp_SetAll(const int16_t *deg);

/**
  * @brief  读回软件推算的当前角度。
  */
int16_t ServoApp_GetAngle(uint8_t idx);

/**
  * @brief  探测哪几颗舵机在线。
  * @retval 位掩码，bit0=第 0 颗在线，以此类推
  */
uint8_t ServoApp_ScanPresent(void);

/**
  * @brief  取最近一次总线操作的状态，调试用。
  */
bus_servo_status_t ServoApp_LastStatus(void);

#endif /* __SERVO_APP_H */
