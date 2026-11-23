#include "chassis.h"

int16_t mtrSpd[4] ={0};

void chassisInit() {
    Motor_InitAll();
}

void chassisCircle(float percentage) {//设正转为顺时针
    float rpm = percentage * MAX_MOTOR_RPM;

    for (uint8_t i = 0; i < 4; i++) {

    }
}

void chassisAddCircle(float percentage) {
    //增量旋转
}

void chassisSetSpd(float percentage) {
    //百分比设置速度
}

void chassisAddSpeed(float percentage) {
    //百分比增量速度
}


