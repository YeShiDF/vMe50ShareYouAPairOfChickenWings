

#ifndef C_TEST_LORA_LORA_H
#define C_TEST_LORA_LORA_H

#define LORA_RX 0
#define LORA_TX 1
#include <stdint.h>

#include "stm32f103xb.h"
#include "stm32f1xx_hal_uart.h"


typedef struct {
    uint16_t mac;
    uint8_t channel;

    GPIO_TypeDef *rx_port;
    GPIO_TypeDef *tx_port;
    uint16_t rx_pin;
    uint16_t tx_pin;
    UART_HandleTypeDef *huart;
}lora_t;

#endif //C_TEST_LORA_LORA_H
