
#include <mbed.h>
#include "driveBase.hpp"
#include "driveMotor.hpp"
#include "parameters.hpp"

int main(){
    //モーターの用意
    DriveMotor motor0(PA_0, PA_1, PA_2, PA_3, MOTOR_0_KP, MOTOR_0_KI, MOTOR_0_KD);
    DriveMotor motor1(PA_4, PA_5, PA_6, PA_7, MOTOR_0_KP, MOTOR_0_KI, MOTOR_0_KD);
    DriveMotor motor2(PA_8, PA_9, PA_10, PA_11, MOTOR_0_KP, MOTOR_0_KI, MOTOR_0_KD);
    DriveMotor motor3(PA_12, PA_13, PA_14, PA_15, MOTOR_0_KP, MOTOR_0_KI, MOTOR_0_KD);

    //足回り
    DriveBase driveBase(&motor0, &motor1, &motor2, &motor3);

    //現在の座標を設定
    driveBase.localization.setPosition(0,0,0);

    //目的地に移動
    driveBase.goTo(1000, 500, PI/2);
}
