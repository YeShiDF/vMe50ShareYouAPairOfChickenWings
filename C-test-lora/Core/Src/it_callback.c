/*
 *此处用于存放中断回调函数
 *
 *
 *
 */
#include "main.h"
#include "lora.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == lora.huart) {
        osMessageQueuePut(cmdQueueHandle,&rxBit,0,0);
        HAL_UART_Receive_DMA(lora.huart,&rxBit,1);
    }
    if (huart == remote.huart) {
        //此处补上遥控器的主函数队列代码
    }
}