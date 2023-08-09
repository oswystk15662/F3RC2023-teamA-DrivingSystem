
#include <mbed.h>
#include "driveMotor.hpp"
#include "driveBase.hpp"
#include "parameters.hpp"


//エンコーダーのピン，モーターのピン，PIDゲイン
DriveMotor motor0(A0, A1, D2, D3, MOTOR_0_KP_1, MOTOR_0_KI_1, MOTOR_0_KD_1, MOTOR_0_KP_2, MOTOR_0_KI_2, MOTOR_0_KD_2);
DriveMotor motor1(A2, A3, D4, D5, MOTOR_1_KP_1, MOTOR_1_KI_1, MOTOR_1_KD_1, MOTOR_1_KP_2, MOTOR_1_KI_2, MOTOR_1_KD_2);
DriveMotor motor2(A4, A5, D6, D7, MOTOR_2_KP_1, MOTOR_2_KI_1, MOTOR_2_KD_1, MOTOR_2_KP_2, MOTOR_2_KI_2, MOTOR_2_KD_2);
DriveMotor motor3(D0, D1, D8, D9, MOTOR_3_KP_1, MOTOR_3_KI_1, MOTOR_3_KD_1, MOTOR_3_KP_2, MOTOR_3_KI_2, MOTOR_3_KD_2);



//足回り全体
DriveBase driveBase(&motor0, &motor1, &motor2, &motor3, DRIVEBASE_KP, DRIVEBASE_KI, DRIVEBASE_KD, DRIVEBASE_ROTATE_KP, DRIVEBASE_ROTATE_KI, DRIVEBASE_ROTATE_KD);

int main(){
    
    driveBase.runNoEncoder(0.3, 0.3, 0, 0, 4);
    driveBase.runNoEncoder(0, 0, 0, 0.2, 2);
    driveBase.runNoEncoder(0.3, 0.3, 0, 0, 4);

    while(1){
        
    }
}

