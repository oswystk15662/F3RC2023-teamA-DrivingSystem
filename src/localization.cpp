#include <mbed.h>
#include <math.h>
#include "parameters.hpp"
#include "localization.hpp"
#include "driveBase.hpp"

Localization::Localization(DriveMotor* motor_0, DriveMotor* motor_1, DriveMotor* motor_2, DriveMotor* motor_3){
    this->driveMotors[0] = motor_0;
    this->driveMotors[1] = motor_1;
    this->driveMotors[2] = motor_2;
    this->driveMotors[3] = motor_3;

    ticker.attach([this] {encoderLocalization();}, std::chrono::milliseconds(1000)/ENCODER_LOCALIZATION_FREQUENCY);
}


void Localization::encoderLocalization(){
    //エンコーダーによる位置，速度の推定
    int incrementDelta[4] = {0,0,0,0}; //前回からのエンコーダーのインクリメント数の変化

    for (int i=0;i<4;i++){
        incrementDelta[i] = driveMotors[i]->encoder.IncrementedNum - incrementedNumBefore[i];
    }

    //回転成分
    float dd = (incrementDelta[0] + incrementDelta[1] + incrementDelta[2] + incrementDelta[3])/4;

    //回転成分を取り除く
    float a = SQRT2*(driveMotors[0]->encoder.IncrementedNum-dd); //-dx+dy
    float b = SQRT2*(driveMotors[1]->encoder.IncrementedNum-dd); //-dx-dy
    float c = SQRT2*(driveMotors[2]->encoder.IncrementedNum-dd); //+dx-dy
    float d = SQRT2*(driveMotors[3]->encoder.IncrementedNum-dd); //+dx+dy

    //位置の変化．ただしロボットに対して相対的な向き
    float dx = (-a-b+c+d)/4; // x成分の変化量/MMPP
    float dy = (+a-b-c+d)/4; // y成分の変化量/MMPP

    //推定値の書き込み
    float dX = MMPP * (cos(direction)*dx-sin(direction)*dy);
    posX += dX;
    speedX = dX*ENCODER_LOCALIZATION_FREQUENCY;

    float dY = MMPP * (sin(direction)*dx+cos(direction)*dy);
    posY += dY;
    speedX = dY*ENCODER_LOCALIZATION_FREQUENCY;

    float dD = RADPP * dd;
    direction += dD;
    rotateSpeed = dD*ENCODER_LOCALIZATION_FREQUENCY;

    for (int i=0;i<4;i++){
        incrementedNumBefore[i] = driveMotors[i]->encoder.IncrementedNum;
    }

}

void Localization::setPosition(float X, float Y, float D){
    //距離センサなどの情報から位置を設定
    posX = X;
    posY = Y;
    direction = D;
}

