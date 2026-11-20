#include <stdlib.h>

#include "cmsis_os2.h"
#include "lora.h"
#include "freertos_os2.h"
#include "task.h"
#include "usart.h"

extern lora_t lora;

void StartTaskMainCmd() {//主函数示例
    uint8_t cmd = 0x00;
    for (;;) {
        cmd = 0x00;

        osMessageQueueGet(cmdQueueHandle,&cmd,0,osWaitForever);
        //代码块
    }
}


