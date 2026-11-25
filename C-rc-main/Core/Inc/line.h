
#ifndef C_RC_MAIN_LINE_H
#define C_RC_MAIN_LINE_H

#include "main.h"
#include "chassis.h"

#ifndef delay
#define delay(x) HAL_Delay(x)
#endif

#ifndef read
#define read(port,pin) HAL_GPIO_ReadPin(port,pin)
#endif

#ifndef write
#define write(port,pin,value) HAL_GPIO_WritePin(port,pin,value)
#endif

#define LINE_LL_PORT
#define LINE_LL_PIN
#define LINE_LM_PORT
#define LINE_LM_PIN
#define LINE_RM_PORT
#define LINE_RM_PIN
#define LINE_RR_PORT
#define LINE_RR_PIN
#define LINE_M_PORT GPIOA
#define LINE_M_PIN GPIO_PIN_0

typedef enum {
    white,
    black,
} line_stat_t;

typedef enum {
    LL,
    LM,
    M,
    RM,
    RR,
} line_num_t;

typedef struct  {
    line_stat_t stat;
    line_num_t num;
    GPIO_TypeDef *port;
    uint16_t pin;
} line_t;


void lineSimpleFollow();

#endif //C_RC_MAIN_LINE_H
