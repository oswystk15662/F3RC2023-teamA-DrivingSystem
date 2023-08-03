#pragma once

#include <mbed.h>
#include "driveMotor.hpp"
#include "encoder.hpp"
#include "parameters.hpp"
#include "localization.hpp"
#include "PIDcontroller.hpp"


float radiansMod(float x, float y=2*PI);

class DriveBase{
    public:
        DriveMotor* motors[4];
        Localization localization;

        PIDController pidController;
        PIDController pidRotateController;
        

        //直線移動
        void goTo(float X, float Y, float D, bool idle=true);
        void rotateTo(float D, bool idle=true);
        void goParallelTo(float X, float Y, bool idle=true);

        //曲線移動 これから作る
        

        //移動の停止
        void stopMovement();

        DriveBase(DriveMotor* motor_0, DriveMotor* motor_1, DriveMotor* motor_2, DriveMotor* motor_3, float kp_1, float ki_1, float kd_1, float kp_2, float ki_2, float kd_2);

    private:
        void go(float targetSpeedX, float targetSpeedY, float targetSpeedD);
        void goTowardTargetAccDcc();
        void resetPID();

        bool moving = false;

        Ticker movementTicker;

        //目標位置
        float target_X = 0.0f;
        float target_Y = 0.0f;
        float target_D = 0.0f;

        //前回の移動
        float delta_X_before = 0.0f;
        float delta_Y_before = 0.0f;
        float delta_D_before = 0.0f;
};



