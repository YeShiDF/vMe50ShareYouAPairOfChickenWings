
#ifndef C_RC_MAIN_LINE_H
#define C_RC_MAIN_LINE_H

#include "main.h"

#ifndef delay
#define delay(x) HAL_Delay(x)
#endif

#ifndef read
#define read(port,pin) HAL_GPIO_ReadPin(port,pin)
#endif

#ifndef write
#define write(port,pin,value) HAL_GPIO_WritePin(port,pin,value)
#endif


#endif //C_RC_MAIN_LINE_H
