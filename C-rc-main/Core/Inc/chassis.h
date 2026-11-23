

#ifndef C_RC_MAIN_CHASSIS_H
#define C_RC_MAIN_CHASSIS_H

#include "motor.h"

#define MAX_MOTOR_RPM 200

typedef enum {
    motorLF,//左前
    motorRF,
    motorLB,
    motorRB,//右后
}motor_type_t;

/*麦轮布局：
 *左前右后，右前左后平行，出力方向围成菱形
 *      /\
 *     /  \
 *    /    \
 *    \    /
 *     \  /
 *      \/
 *
 *
 */

void chassisSetSpeed(float percentage);
void chassisAddSpeed(float percentage);
void chassisCircle(float percentage);
void chassisAddCircle(float percentage);

#endif //C_RC_MAIN_CHASSIS_H
