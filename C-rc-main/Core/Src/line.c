#include "line.h"
#include "gpio.h"

extern line_t line[5];
int err = 0;

void lineUpdate(){}

void lineInit(){}

static  int readErr() {
    int error = 0;
    for (int i = 0; i < 5; i++) {
        line[i].stat = read(line[i].port, line[i].pin);
    }
    err = 3*!line[LL].stat + !line[LM].stat - !line[RM].stat - 3*!line[RR].stat;
    return err;
}

void lineSimpleFollow() {
    while (1) {
        err = readErr();
        delay(20);

        if (err == 0) {
            if (line[M].stat == white) {
                delay(100);
                if (readErr() == 0) {
                    //丢线处理
                    continue;
                }
            }
        }
        chassisSetSpd( 50);
        chassisSetCircle(err*20);
    }
}
