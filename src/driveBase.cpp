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
DriveBase::DriveBase(DriveMotor* motor_0, DriveMotor* motor_1, DriveMotor* motor_2, DriveMotor* motor_3): localization(motor_0, motor_1, motor_2, motor_3){
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


//現在決められている目標に向かって進む
//PID制御で目標値に向かうが，速度・加速度に制限を設けることで台形制御を実現する
void DriveBase::goTowardTargetAccDcc(){
    //速度を制限する
    
    float delta_X = target_X - localization.posX;
    float delta_Y = target_Y - localization.posY;
    float delta_D = target_D - localization.direction;
    float delta_R = sqrt(delta_X*delta_X + delta_Y*delta_Y);

    if(delta_R > MAX_DELTA_R){
        delta_X = (delta_X / delta_R) * MAX_DELTA_R;
        delta_Y = (delta_Y / delta_R) * MAX_DELTA_R;
        target_X = localization.posX + delta_X;
        target_Y = localization.posY + delta_Y;
    }

    if(delta_D > MAX_DELTA_D){
        delta_D = MAX_DELTA_D;
        target_D = localization.direction + MAX_DELTA_D;
    }else if(delta_D < -MAX_DELTA_D){
        delta_D = -MAX_DELTA_D;
        target_D = localization.direction + MAX_DELTA_D;
    }

    //加速度を制限する

    float delta_delta_X = delta_X - delta_X_before;
    float delta_delta_Y = delta_Y - delta_Y_before;
    float delta_delta_D = delta_D - delta_D_before;
    float delta_delta_R = sqrt(delta_delta_X*delta_delta_X + delta_delta_Y*delta_delta_Y);

    if(delta_delta_R > MAX_DELTA_DELTA_R){
        delta_delta_X = (delta_X / delta_R) * MAX_DELTA_DELTA_R;
        delta_delta_Y = (delta_Y / delta_R) * MAX_DELTA_DELTA_R;
        delta_X = delta_X_before + delta_delta_X;
        delta_Y = delta_Y_before + delta_delta_Y;
        target_X = localization.posX + delta_X;
        target_Y = localization.posY + delta_Y;
    }

    if(delta_delta_D > MAX_DELTA_DELTA_D){
        delta_delta_D = MAX_DELTA_DELTA_D;
        delta_D = delta_D_before + delta_delta_D;
        target_D = localization.direction + delta_D;
    }else if(delta_D < -MAX_DELTA_DELTA_D){
        delta_delta_D = -MAX_DELTA_DELTA_D;
        delta_D = delta_D_before + delta_delta_D;
        target_D = localization.direction + delta_D;
    }

    //更新
    delta_X_before = delta_X;
    delta_Y_before = delta_Y;
    delta_D_before = delta_D;

    //モーターを動かす
    goTowardRelative(delta_X, delta_Y, delta_D);

    if (pow(target_X-localization.posX,2) + pow(target_Y-localization.posY,2) < (MOVEMENT_THRESHOLD*MOVEMENT_THRESHOLD) && abs(radiansMod(target_D - localization.direction)) < MOVEMENT_THRESHOLD_RAD){
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

