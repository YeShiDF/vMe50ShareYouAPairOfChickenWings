/**
  ******************************************************************************
  * @file    bus_servo.h
  * @brief   驱动层：众灵 ZP 系列总线舵机的 ASCII 协议。
  *          只依赖 bsp_uart，不认识"角度"，只认识 ID / PWM / 时间。
  ******************************************************************************
  */
#ifndef __BUS_SERVO_H
#define __BUS_SERVO_H

#include "main.h"     /* 提供 UART_HandleTypeDef 类型 */
#include <stdint.h>

/* ---------- 协议常量（来自《总线舵机通信协议》2022-04-19 版） ---------- */
#define BUS_SERVO_ID_MAX        (254U)   /* 0~254 可设置为 ID */
#define BUS_SERVO_ID_BROADCAST  (255U)   /* 255 = 广播，所有舵机都执行 */
#define BUS_SERVO_PWM_MIN       (500U)   /* PWM 值下限 */
#define BUS_SERVO_PWM_MAX       (2500U)  /* PWM 值上限 */
#define BUS_SERVO_TIME_MAX      (9999U)  /* 运动时间上限，单位 ms */
#define BUS_SERVO_FRAME_MAX     (64U)    /* 单条回复的最大长度 */

/* ---------- 返回状态 ---------- */
typedef enum
{
  BUS_SERVO_OK = 0,          /* 成功 */
  BUS_SERVO_ERR_PARAM = -1,  /* 入参越界 */
  BUS_SERVO_ERR_SEND = -2,   /* 串口发送失败 */
  BUS_SERVO_ERR_TIMEOUT = -3,/* 等回复超时 */
  BUS_SERVO_ERR_FORMAT = -4  /* 回复帧格式不对 */
} bus_servo_status_t;

/* ---------- 初始化 ---------- */
/* 传入舵机所在串口（本工程为 &huart3） */
void BusServo_Init(UART_HandleTypeDef *huart);

/* ---------- 控制 ---------- */

/* 单颗：转到 pwm，用时 time_ms 毫秒 */
bus_servo_status_t BusServo_SetPwm(uint8_t id, uint16_t pwm, uint16_t time_ms);

/* 多颗同步：一帧发完，所有舵机同一时刻开始动 */
bus_servo_status_t BusServo_SetPwmSync(const uint8_t *ids,
                                       const uint16_t *pwms,
                                       uint8_t count,
                                       uint16_t time_ms);

/* 释放扭力（可以用手扳动） */
bus_servo_status_t BusServo_Release(uint8_t id);

/* 恢复扭力 */
bus_servo_status_t BusServo_Restore(uint8_t id);

/* ---------- 回读 ---------- */

/* 读当前 PWM 值，回复 #000P1500! */
bus_servo_status_t BusServo_ReadPwm(uint8_t id, uint16_t *pwm);

/* 读温度/电压，回复 #000T28.1V7.4!
   返回值放大 10 倍存成整数：28.1℃ -> 281 */
bus_servo_status_t BusServo_ReadTempVolt(uint8_t id, int16_t *temp_x10, int16_t *volt_x10);

/* 探测某 ID 是否存在（回复 #000P! 表示在） */
bus_servo_status_t BusServo_Ping(uint8_t id);

/* ---------- 配置（烧写用，正常跑车时不调用） ---------- */

/* 工作模式 1~8，默认 1 = 270° 顺时针 */
bus_servo_status_t BusServo_SetMode(uint8_t id, uint8_t mode);

/* 改 ID，成功后旧 ID 失效 */
bus_servo_status_t BusServo_SetId(uint8_t old_id, uint8_t new_id);

#endif /* __BUS_SERVO_H */
