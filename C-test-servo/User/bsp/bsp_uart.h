/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @brief   BSP 层：串口收发的最小封装。
  *          本文件"不知道舵机存在"，只认识 UART_HandleTypeDef。
  ******************************************************************************
  */
#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"
#include <stdint.h>

/* 返回值约定：0 成功，非 0 失败 */
#define BSP_UART_OK      (0)
#define BSP_UART_ERR     (-1)

/**
  * @brief  绑定一条串口，并在内部清空残留接收数据。
  * @param  huart_bus 舵机所在的串口句柄（本工程为 &huart3）
  */
void BspUart_Init(UART_HandleTypeDef *huart_bus);

/**
  * @brief  发送一段字节（阻塞）。
  * @param  data       数据首地址
  * @param  len        字节数
  * @param  timeout_ms 最长等待时间
  * @retval BSP_UART_OK / BSP_UART_ERR
  */
int BspUart_Send(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

/**
  * @brief  接收固定字节数（阻塞）。
  * @param  data       接收缓冲区
  * @param  len        期望字节数
  * @param  timeout_ms 最长等待时间
  * @retval BSP_UART_OK / BSP_UART_ERR
  */
int BspUart_Recv(uint8_t *data, uint16_t len, uint32_t timeout_ms);

/**
  * @brief  丢弃接收寄存器里的残留数据（含半双工的回显和溢出错误）。
  */
void BspUart_DropRx(void);

/**
  * @brief  取毫秒计时，用于超时判断。
  */
uint32_t BspUart_Millis(void);

#endif /* __BSP_UART_H */
