#pragma once

#include <mbed.h>
#include "encoder.hpp"
#include "PIDcontroller.hpp"

class DriveMotor{
    public:
        Encoder encoder;
        PwmOut pwmOut; //モーター
        DigitalOut dirOut;
        PIDController pidController;
       
        void rotateToward(float target); //target[mm] に向かって動く
        void rotateTowardRelative(float distance); //自分に対してdistance[mm] の方向に動く

        void rotateTo(float target, bool idle=true);
    
        void setPWM(float speed); //PWMの直接書き込み

        void stop();
        DriveMotor(PinName encoder_pin_a, PinName encoder_pin_b, PinName pwm_pin, PinName dir_pin, float kp, float ki, float kd);

    private:
    void rotateTowardTargetAccDcc(); //target[mm] に向かって動く
        float current_target = 0.0f;
        float delta_before = 0.0f;
        bool moving = false;
        Ticker movementTicker;
        
};

