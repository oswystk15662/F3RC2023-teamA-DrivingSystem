#include <mbed.h>
#include <math.h>
#include <chrono>
#include "driveBase.hpp"
#include "parameters.hpp"

float radiansMod(float x, float y){
    //x (mod y) を -y/2 ~ y/2の範囲で出力
    //主に2つの方角のなす角度を計算するのに使用
    return fmod((fmod(x,y) + y/2),y) + y/2;
}

//初期化
DriveBase::DriveBase(DriveMotor* motor_0, DriveMotor* motor_1, DriveMotor* motor_2, DriveMotor* motor_3, float kp_1, float ki_1, float kd_1, float kp_2, float ki_2, float kd_2): localization(motor_0, motor_1, motor_2, motor_3), pidController(SPEED_ADJUSTMENT_FREQUENCY, kp_1, ki_1, kd_1), pidRotateController(SPEED_ADJUSTMENT_FREQUENCY, kp_2, ki_2, kd_2) {
    motors[0] = motor_0;
    motors[1] = motor_1;
    motors[2] = motor_2;
    motors[3] = motor_3;

    moving = false;
}

void DriveBase::resetPID(){
    for (int i=0;i<4;i++){
        motors[i]->pidController.reset();
    }
}

/*
//ある方向に向かって進む
void DriveBase::goTowardRelative(float deltaX, float deltaY, float deltaD){
    float rotated_x = cos(localization.direction)*deltaX + sin(localization.direction)*deltaY;
    float rotated_y = -sin(localization.direction)*deltaX + cos(localization.direction)*deltaY;

    float delta[4]; //モーターの速度
    delta[0] = SQRT2/2 * (- rotated_x + rotated_y) + TRED_RADIUS * deltaD;
    delta[1] = SQRT2/2 * (- rotated_x - rotated_y) + TRED_RADIUS * deltaD;
    delta[2] = SQRT2/2 * (+ rotated_x - rotated_y) + TRED_RADIUS * deltaD;
    delta[3] = SQRT2/2 * (+ rotated_x + rotated_y) + TRED_RADIUS * deltaD;
    
    for (int i=0;i<4;i++){
        motors[i]->rotateTowardRelative(delta[i]);
    }
}
*/


//速度を指定して移動
void DriveBase::go(float targetSpeedX, float targetSpeedY, float targetSpeedD){
    float targetSpeedR = sqrtf(targetSpeedX*targetSpeedX + targetSpeedY*targetSpeedY);

    //速度を制限する
    if(targetSpeedR > MAX_SPEED){
        targetSpeedX = MAX_SPEED*(targetSpeedX/targetSpeedR);
        targetSpeedY = MAX_SPEED*(targetSpeedY/targetSpeedR);
        pidController.reset();
    }

    if(targetSpeedD > MAX_ROTATE_SPEED){
        targetSpeedD = MAX_ROTATE_SPEED;
        pidRotateController.reset();
    }else if(targetSpeedD < -MAX_ROTATE_SPEED){
        targetSpeedD = -MAX_ROTATE_SPEED;
        pidRotateController.reset();
    }

    float targetAccX = (targetSpeedX - localization.speedX) * SPEED_ADJUSTMENT_FREQUENCY;
    float targetAccY = (targetSpeedY - localization.speedY) * SPEED_ADJUSTMENT_FREQUENCY;
    float targetAccR = sqrtf(targetAccX*targetAccX + targetAccY*targetAccY);
    float targetAccD = (targetSpeedD - localization.rotateSpeed) * SPEED_ADJUSTMENT_FREQUENCY;

    //加速度を制限する
    if(targetAccR > MAX_ACCELERATION){
        targetAccX = MAX_ACCELERATION*(targetAccX/targetAccR);
        targetAccY = MAX_ACCELERATION*(targetAccY/targetAccR);
        targetSpeedX = localization.speedX + targetAccX/SPEED_ADJUSTMENT_FREQUENCY;
        targetSpeedY = localization.speedY + targetAccY/SPEED_ADJUSTMENT_FREQUENCY;
    }

    if(targetAccD > MAX_ROTATE_ACCELERATION){
        targetSpeedD = localization.rotateSpeed + MAX_ROTATE_ACCELERATION / SPEED_ADJUSTMENT_FREQUENCY;
    }else if(targetAccD < -MAX_ROTATE_ACCELERATION){
        targetSpeedD = localization.rotateSpeed - MAX_ROTATE_ACCELERATION / SPEED_ADJUSTMENT_FREQUENCY;
    }

    //X, Yに回転行列をかける
    float vx = cos(localization.direction)*targetSpeedX + sin(localization.direction)*targetSpeedY;
    float vy = -sin(localization.direction)*targetSpeedX + cos(localization.direction)*targetSpeedY;

    //各モーターの速度
    float speeds[4]; //モーターの速度
    speeds[0] = SQRT2/2 * (- vx + vy) + TRED_RADIUS * targetSpeedD;
    speeds[1] = SQRT2/2 * (- vx - vy) + TRED_RADIUS * targetSpeedD;
    speeds[2] = SQRT2/2 * (+ vx - vy) + TRED_RADIUS * targetSpeedD;
    speeds[3] = SQRT2/2 * (+ vx + vy) + TRED_RADIUS * targetSpeedD;
    
    for (int i=0;i<4;i++){
        motors[i]->rotate(speeds[i]);
    }

}

//現在決められている目標に向かって進む
//PID制御で目標値に向かうが，速度・加速度に制限を設けることで台形制御を実現する
void DriveBase::goTowardTargetAccDcc(){
    float differenceX = target_X-localization.posX;
    float differenceY = target_Y-localization.posY;
    float differenceR = sqrtf(differenceX*differenceX + differenceY*differenceY);
    float differenceD = radiansMod(target_D-localization.direction);

    float targetSpeedR = pidController.calculate(differenceR);

    float targetSpeedX, targetSpeedY;
    if(differenceR == 0){
        //0除算の回避
        targetSpeedX = 0;
        targetSpeedY = 0;
    }else{
        targetSpeedX = targetSpeedR*(differenceX/differenceR);
        targetSpeedY = targetSpeedR*(differenceY/differenceR);
    }

    float targetSpeedD = pidController.calculate(differenceD);

    go(targetSpeedX, targetSpeedY, targetSpeedD);

    if (differenceR < MOVEMENT_THRESHOLD && abs(radiansMod(target_D - localization.direction)) < MOVEMENT_THRESHOLD_RAD){
        stopMovement();
    }
}

//モーターの停止
void DriveBase::stopMovement(){
    movementTicker.detach();
    moving = false;
    for(int i=0;i<4;i++){
        motors[i]->stop();
    }
}


//目標位置に向かって直線移動する
void DriveBase::goTo(float X, float Y, float D, bool idle){
    if(moving){
        printf("warning: a motion requested while the robot is moving.");
        movementTicker.detach();
    }
    moving = true;

    //目標位置の設定
    target_X = X;
    target_Y = Y;
    target_D = D;

    //PID制御器の初期化
    resetPID();

    //割り込みの設定
    movementTicker.attach([this] {goTowardTargetAccDcc();}, std::chrono::milliseconds(1000)/SPEED_ADJUSTMENT_FREQUENCY);

    //idle=trueなら移動が終わるまで待機
    if(idle){
        while(moving) {}
    }
}

//超信地旋回
void DriveBase::rotateTo(float D, bool idle){
    goTo(localization.posX, localization.posY, D, idle);
}

//平行移動
void DriveBase::goParallelTo(float X, float Y, bool idle){
    goTo(X, Y, localization.direction, idle);
}

