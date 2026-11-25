#include "chassis.h"

#include <stdlib.h>
static void chassisUpdate();
int rotationalSpeed,horizontalSpeed;
void chassisInit() {
    for (int i = 0; i < 4; i++) {
        Motor_SetRpm(i,0);
    }
    rotationalSpeed = 0;
    horizontalSpeed = 0;
    chassisUpdate();
}

static void chassisUpdate() {
    for (int i = 0; i < 4; i++) {
        int spd = horizontalSpeed + ((i%2)? -rotationalSpeed : rotationalSpeed);
        if (abs(spd) > 100) spd = (spd>0)?100:-100;
        Motor_SetRpm(i,spd*10);
    }
}

int chassisSetSpd(int perc) {
    if (abs(perc) > 100) perc = (perc>0)?100:-100;
    horizontalSpeed = perc;
    chassisUpdate();
    return horizontalSpeed;
}

int chassisAddSpd(int perc) {
    if (abs(perc) > 100) perc = (perc>0)?100:-100;
    horizontalSpeed += perc;
    if (abs(horizontalSpeed) > 100) horizontalSpeed = (horizontalSpeed>0)?100:-100;
    chassisUpdate();
    return horizontalSpeed;
}

int chassisSetCircle(int perc) {
    if (abs(perc) > 100) perc = (perc>0)?100:-100;
    rotationalSpeed = perc;
    chassisUpdate();
    return rotationalSpeed;
}

int chassisAddCircle(int perc) {
    if (abs(perc) > 100) perc = (perc>0)?100:-100;
    rotationalSpeed += perc;
    if (abs(rotationalSpeed) > 100) rotationalSpeed = (rotationalSpeed>0)?100:-100;
    chassisUpdate();
    return rotationalSpeed;
}