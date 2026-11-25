
#ifndef F103C8TX_CHASSIS_H
#define F103C8TX_CHASSIS_H

#include "motor.h"

typedef enum {
    leftFront,
    rightFront,
    leftBack,
    rightBack,
} motorLocation_t;

typedef enum {
    clockwise = 1,
    antiClockwise = -1,
} circleDirection_t;//顺时针为正转

void chassisInit();
int chassisSetSpd(int perc);
int chassisAddSpd(int perc);
int chassisSetCircle(int perc);
int chassisAddCircle(int perc);

#endif //F103C8TX_CHASSIS_H
