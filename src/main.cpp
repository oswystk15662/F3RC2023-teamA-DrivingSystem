
#include <mbed.h>
#include "driveMotor.hpp"
#include "parameters.hpp"


Timer timer;

int main(){
    timer.start();
    //モーターの用意
    //DriveMotor motor0(D9, D8, D12, D11, 0.0005f, 0.00002f, 0.00004f);
    DriveMotor motor0(D9, D8, D12, D11, 1.3f, 0.06f, 0, 0.00003f, 0.000001f, 0);
    motor0.rotateTo(10000, false);
    //motor0.rotatePermanent(2000, false);

    while(motor0.moving){
        //printf("SPEED:%d TSPEED:%d ENCODER:%d\n", int(motor0.delta_before*SPEED_ADJUSTMENT_FREQUENCY), int(motor0.targetDelta*SPEED_ADJUSTMENT_FREQUENCY), int(motor0.encoder.getAmount()));
        printf("SPEED:%d TSPEED:%d POS:%d\n", int(motor0._s1), int(motor0._s2),  int(motor0.encoder.getAmount()));
        //printf("PWM:%d\n", int(motor0.pwm*100));
    }
}

//0.002 0.0003 0.00004