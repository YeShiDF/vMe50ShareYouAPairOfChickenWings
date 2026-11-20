#include "lora.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os2.h"
#include "usart.h"
#include "main.h"


#if IFCTRL
lora_t lora_local = {CAR_MAC,LORA_CHANNEL,GPIOA,GPIOA,2,3,&huart2,
    &hdma_usart2_rx,&hdma_usart2_tx};//车载lora配置修改处
lora_t lora_client = {CTRL_MAC,LORA_CHANNEL};

void dbg_send(const char *s) {
    HAL_UART_Transmit(&huart1, (const uint8_t *) s, (uint16_t) strlen(s), 200);
}//仅限车载调试且开启uart1时使用
#else
lora_t lora_local = {CTRL_MAC,LORA_CHANNEL,GPIOA,GPIOA,9,10,
    &huart1,NULL,NULL};//遥控lora配置修改处(如有需要自行配置dma)
lora_t lora_client = {CAR_MAC,LORA_CHANNEL};

#define dbg_send(s)

#endif
uint8_t rxBit = 0x00;




lora_state_t lora_init() {
    delay(500);

    char msg[64] = {0};
    uint16_t size = 0;
    HAL_UART_Transmit(lora_local.huart,"+++\r\n",5,100);
    HAL_UARTEx_ReceiveToIdle(lora_local.huart,msg,64,&size,1000);

    delay(50);
    if (strstr(msg,"Entry") == NULL) {
        HAL_UART_Transmit(lora_local.huart,"+++\r\n",5,100);
    }
    delay(50);
    memset(msg,0,sizeof(msg));
    HAL_UART_Transmit(lora_local.huart,"AT+LEVEL4\r\n",11,100);
    HAL_UARTEx_ReceiveToIdle(lora_local.huart,msg,64,&size,1000);
    dbg_send(msg);

    delay(50);
    memset(msg,0,sizeof(msg));
    char b[20] = {0};
    sprintf(b,"AT+MAC%02x%c%02x%s",(uint8_t)(lora_local.mac>>8),',',(uint8_t)lora_local.mac,"\r\n");
    HAL_UART_Transmit(lora_local.huart,b,strlen(b),100);
    HAL_UARTEx_ReceiveToIdle(lora_local.huart,msg,64,&size,1000);
    dbg_send(msg);

    delay(50);
    memset(msg,0,sizeof(msg));
    memset(b,0,sizeof(b));
    sprintf(b,"AT+CHANNEL%02x%s",lora_local.channel,"\r\n");
    HAL_UART_Transmit(lora_local.huart,b,strlen(b),100);
    HAL_UARTEx_ReceiveToIdle(lora_local.huart,msg,64,&size,1000);
    dbg_send(msg);

    delay(50);
    memset(msg,0,sizeof(msg));
    HAL_UART_Transmit(lora_local.huart,"AT+MODE1\r\n",10,100);
    HAL_UARTEx_ReceiveToIdle(lora_local.huart,msg,64,&size,1000);
    dbg_send(msg);


    delay(50);
    memset(msg,0,sizeof(msg));
    HAL_UART_Transmit(lora_local.huart,"AT+RESET\r\n",10,100);
    HAL_UARTEx_ReceiveToIdle(lora_local.huart,msg,64,&size,1000);
    dbg_send(msg);

    delay(100);
    HAL_UART_Receive_DMA(lora_local.huart,&rxBit,1);

    return normal;
//发送数据格式：地址2位+信道1位+数据

}


lora_state_t lora_send(uint8_t* data,uint8_t size) {
    if (data == NULL || size == 0) return config_null;

    osDelay(50);
    HAL_UART_Transmit(lora_local.huart,(uint8_t[]){(uint8_t)(lora_client.mac>>8),
        (uint8_t)lora_client.mac,lora_client.channel},3,100);
    HAL_UART_Transmit(lora_local.huart,data,size,100);
    return normal;
    // dbg_send(data);


    return config_null;
}


// buffer arr


//数据格式：包头（0xAA） + 包总长 + 数据位 + 校验和(建议加上编码)
