
/*
 *需要开启usart2和其中断
 *
 *
 *
 */
#ifndef C_TEST_LORA_LORA_H
#define C_TEST_LORA_LORA_H

#define LORA_RX 0
#define LORA_TX 1

#define MAX_BUFFER_SIZE 256
#define MIN_COMMAND_LENGTH 4
#define MAX_SEND_DATA_LENGTH 230
#define CAR_MAC 0x9527
#define CTRL_MAC 0x0721
#define LORA_CHANNEL 0x10
#define LORA_MODE 1
#define LORA_LEVEL 4

#define IFCTRL 0//0-car,1-ctrl
#define LORA_AUX_PIN GPIO_PIN_8//aux引脚
#define LORA_AUX_PORT GPIOA

#include <stdint.h>

#include "cmsis_os2.h"
#include "main.h"


extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern osSemaphoreId_t loraRxSemHandle;

extern uint8_t rxBit;
extern osMessageQueueId_t cmdQueueHandle;

typedef struct {
    uint16_t mac;
    uint8_t channel;

    GPIO_TypeDef *rx_port;
    GPIO_TypeDef *tx_port;
    uint16_t rx_pin;
    uint16_t tx_pin;
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef *hdma_rx;
    DMA_HandleTypeDef *hdma_tx;

}lora_t;

typedef enum {
    normal,
    error,
    config_null,
    send_time_out,
    send_error,
} lora_state_t;

typedef enum {
    car,
    ctrl,
} loraTarget_t;
extern lora_t remote,lora;

lora_state_t lora_init_car(void);
lora_state_t lora_send(uint8_t* data,uint8_t size,loraTarget_t target);
void dbg_send(const char *s);

#endif //C_TEST_LORA_LORA_H
