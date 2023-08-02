#pragma once 

#include "encoder.hpp"
#include "driveMotor.hpp"

//自己位置推

class Localization{
    public:
        DriveMotor* driveMotors[4]; //モーター

        //推定された位置
        float posX = 0.0f; //位置X[mm]
        float posY = 0.0f; //位置Y[mm]
        float direction = 0.0f; //方角D[rad]

        //推定された速度
        float rotateSpeed = 0.0f;
        float speedX = 0.0f;
        float speedY = 0.0f;

        void setPosition(float X, float Y, float D); //外部から位置を強制的に設定する．
        void encoderLocalization(); //エンコーダーによる自己位置推定

        Localization(DriveMotor* motor_0, DriveMotor* motor_1, DriveMotor* motor_2, DriveMotor* motor_3);

    private:
        int incrementedNumBefore[4] = {0,0,0,0};

        Ticker ticker;
};