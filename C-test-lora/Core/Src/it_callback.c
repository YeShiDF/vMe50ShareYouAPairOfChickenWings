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
        lora_receive();
    }
}