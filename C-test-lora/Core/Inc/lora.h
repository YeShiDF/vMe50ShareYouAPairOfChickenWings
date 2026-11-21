
/*
 *
 *提示：至少需要打开uart的中断/DMA中一项
 *
 *
 */
#ifndef C_TEST_LORA_LORA_H
#define C_TEST_LORA_LORA_H

#define CAR_MAC 0x9527
#define CTRL_MAC 0x0721
#define LORA_CHANNEL 0x10
#define LORA_MODE 1
#define LORA_LEVEL 4

#define IFCTRL 0//0-car,1-ctrl
#define LORA_AUX_PIN GPIO_PIN_8//aux引脚，暂时没用
#define LORA_AUX_PORT GPIOA

#define LORA_DEBUG 0

#include <stdint.h>

#include "cmsis_os2.h"
#include "main.h"

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
    rx_error,
} lora_state_t;

extern lora_t lora_client,lora_local;

lora_state_t lora_init(void);
lora_state_t lora_receive(void);
lora_state_t lora_send(uint8_t* data,uint8_t size);
void dbg_send(const char *s);
void errReport(lora_state_t err);


#endif //C_TEST_LORA_LORA_H
