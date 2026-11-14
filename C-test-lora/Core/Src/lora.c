#include "lora.h"

#include <stdio.h>
#include <string.h>

#include "usart.h"

lora_t lora[2] = {{0x0000,0x01,GPIOA,GPIOA,2,3,&huart2},
{0x00ff,0x01,GPIOA,GPIOA,9,10,&huart1} };

typedef enum {
    normal,
    error,
} lora_state_t;

char AT[] ={'+','+','+'};
char readBuf[64] = {0};

lora_state_t lora_init() {
    for (int i = 0; i < 2; i++) {
        uint16_t size;
        HAL_UART_Transmit(lora[i].huart,AT,3,1000);
        HAL_UARTEx_ReceiveToIdle(lora[i].huart,readBuf,64,&size,100);
        HAL_Delay(100);
        if (size != 0) {
            if (readBuf!="Entry AT") return error;
        }

        char message[64] = {0};
        sprintf(message,"AT+MAC%d\r\n",lora[i].mac);
        HAL_UART_Transmit(lora[i].huart,message,strlen(message),1000);
        sprintf(message,"AT+CHANNEL%d\r\n",lora[i].channel);
        HAL_UART_Transmit(lora[i].huart,message,strlen(message),1000);

        HAL_UART_Transmit(lora[i].huart,AT,3,1000);
        HAL_UARTEx_ReceiveToIdle(lora[i].huart,readBuf,64,&size,100);
        HAL_Delay(100);
        if (size != 0) {
            if (readBuf != "Exit AT\nPower On") return error;
        }

        return normal;
    }
}

void lora_send(lora_t *lora,uint8_t* data,uint8_t size) {
    if (&lora != NULL) {
        HAL_UART_Transmit_DMA(lora->huart,data,size);
    }
}
