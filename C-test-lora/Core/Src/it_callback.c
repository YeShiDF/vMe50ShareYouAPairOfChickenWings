/*
 *此处用于存放中断回调函数
 *
 *
 *
 */
#include "main.h"
#include "lora.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == lora_local.huart) {
        osMessageQueuePut(cmdQueueHandle,&rxBit,0,0);
        if (HAL_UART_Receive_DMA(lora_local.huart,&rxBit,1)!=HAL_OK)
            errReport(rx_error);
    }
}