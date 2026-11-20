#include "lora.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os2.h"
#include "usart.h"
#include "main.h"
unsigned char ctrlMac[3] = {0x00,0x00,0x10};
lora_t lora = {0x9527,0x10,GPIOA,GPIOA,2,3,&huart2,
    &hdma_usart2_rx,&hdma_usart2_tx};
lora_t remote = {0x0721,0x10,GPIOA,GPIOA,9,10,&huart1,
    NULL,NULL};
uint8_t rxBit = 0x00;

/* 定义在 loraTask.c 里，是数组，不是单个字节 */
extern uint8_t rxData[];

void dbg_send(const char *s) {
    HAL_UART_Transmit(&huart1, (const uint8_t *) s, (uint16_t) strlen(s), 200);
}

lora_state_t lora_init_car() {
    delay(500);

    char msg[64] = {0};
    uint16_t size = 0;
    HAL_UART_Transmit(lora.huart,"+++\r\n",5,100);
    HAL_UARTEx_ReceiveToIdle(lora.huart,msg,64,&size,1000);

    delay(50);
    if (strstr(msg,"Entry") == NULL) {
        HAL_UART_Transmit(lora.huart,"+++\r\n",5,100);
    }
    delay(50);
    memset(msg,0,sizeof(msg));
    HAL_UART_Transmit(lora.huart,"AT+LEVEL4\r\n",11,100);
    HAL_UARTEx_ReceiveToIdle(lora.huart,msg,64,&size,1000);
    dbg_send(msg);

    delay(50);
    memset(msg,0,sizeof(msg));
    char b[20] = {0};
    sprintf(b,"AT+MAC%02x%c%02x%s",(uint8_t)(lora.mac>>8),',',(uint8_t)lora.mac,"\r\n");
    HAL_UART_Transmit(lora.huart,b,strlen(b),100);
    HAL_UARTEx_ReceiveToIdle(lora.huart,msg,64,&size,1000);
    dbg_send(msg);

    delay(50);
    memset(msg,0,sizeof(msg));
    memset(b,0,sizeof(b));
    sprintf(b,"AT+CHANNEL%02x%s",lora.channel,"\r\n");
    HAL_UART_Transmit(lora.huart,b,strlen(b),100);
    HAL_UARTEx_ReceiveToIdle(lora.huart,msg,64,&size,1000);
    dbg_send(msg);

    delay(50);
    memset(msg,0,sizeof(msg));
    HAL_UART_Transmit(lora.huart,"AT+MODE1\r\n",10,100);
    HAL_UARTEx_ReceiveToIdle(lora.huart,msg,64,&size,1000);
    dbg_send(msg);


    delay(50);
    memset(msg,0,sizeof(msg));
    HAL_UART_Transmit(lora.huart,"AT+RESET\r\n",10,100);
    HAL_UARTEx_ReceiveToIdle(lora.huart,msg,64,&size,1000);
    dbg_send(msg);

    delay(100);
    HAL_UART_Receive_DMA(lora.huart,&rxBit,1);

    return normal;
//发送数据格式：地址2位+信道1位+数据

}
extern osMessageQueueId_t uartSendQueueHandle;

lora_state_t lora_send(uint8_t* data,uint8_t size,loraTarget_t target) {
    if (data == NULL || size == 0) return config_null;
    if (target == ctrl){
        osDelay(50);
        HAL_UART_Transmit(lora.huart,(uint8_t[]){(uint8_t)(remote.mac>>8),(uint8_t)remote.mac,
            remote.channel},3,100);
        HAL_UART_Transmit(lora.huart,data,size,100);
        return normal;
        // dbg_send(data);
    }
    if (target == car) {
        delay(100);
        HAL_UART_Transmit(remote.huart,(uint8_t[]){(uint8_t)(lora.mac>>8),(uint8_t)lora.mac,
            lora.channel},3,100);
        HAL_UART_Transmit(remote.huart,data,size,100);
        return normal;
    }
    return config_null;
}


// buffer arr


//数据格式：包头（0xAA） + 包总长 + 数据位 + 校验和(建议加上编码)
