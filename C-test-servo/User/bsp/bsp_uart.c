/**
  ******************************************************************************
  * @file    bsp_uart.c
  * @brief   BSP 层实现：只做"把字节送出去 / 把字节收进来"。
  ******************************************************************************
  */
#include "bsp_uart.h"

/* 本模块只服务一条总线，用静态指针保存，避免到处传参 */
static UART_HandleTypeDef *s_bus = NULL;

void BspUart_Init(UART_HandleTypeDef *huart_bus)
{
  s_bus = huart_bus;
  BspUart_DropRx();
}

int BspUart_Send(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
  if ((s_bus == NULL) || (data == NULL) || (len == 0U))
  {
    return BSP_UART_ERR;
  }

  /* HAL 的发送接口参数不是 const，这里强制转换是安全的：
     它只读不写这块内存 */
  if (HAL_UART_Transmit(s_bus, (uint8_t *)data, len, timeout_ms) != HAL_OK)
  {
    return BSP_UART_ERR;
  }
  return BSP_UART_OK;
}

int BspUart_Recv(uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
  if ((s_bus == NULL) || (data == NULL) || (len == 0U))
  {
    return BSP_UART_ERR;
  }

  if (HAL_UART_Receive(s_bus, data, len, timeout_ms) != HAL_OK)
  {
    return BSP_UART_ERR;
  }
  return BSP_UART_OK;
}

void BspUart_DropRx(void)
{
  volatile uint32_t tmp;

  if (s_bus == NULL)
  {
    return;
  }

  /* 1) 把接收寄存器里已经到达的字节读空（读 DR 会清 RXNE 标志） */
  while (__HAL_UART_GET_FLAG(s_bus, UART_FLAG_RXNE) != RESET)
  {
    tmp = s_bus->Instance->DR;
  }

  /* 2) 清溢出错误：STM32F1 的清法是"先读 SR 再读 DR" */
  tmp = s_bus->Instance->SR;
  tmp = s_bus->Instance->DR;
  (void)tmp;
}

uint32_t BspUart_Millis(void)
{
  /* HAL_GetTick 由 SysTick 中断每 1ms 累加 */
  return HAL_GetTick();
}
