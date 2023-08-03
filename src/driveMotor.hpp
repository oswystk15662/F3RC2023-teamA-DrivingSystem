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
        PIDController pidSpeedController;
       
        void rotateToward(float target); //target[mm] に向かって動く
        void rotateTowardRelative(float distance); //自分に対してdistance[mm] の方向に動く

        void rotateTo(float target, bool idle=true);
    
        void setPWM(float signed_pwm); //PWMの直接書き込み

        void rotate(float targetSpeed); //速度を指定

        void rotatePermanent(float speed, bool idle=true);

        void stop();
        DriveMotor(PinName encoder_pin_a, PinName encoder_pin_b, PinName pwm_pin, PinName dir_pin, float kp_1, float ki_1, float kd_1, float kp_2, float ki_2, float kd_2);

        float target = 0.0f;
        float target_speed = 0.0f;
        bool moving = false;


        float delta_before = 0.0f;

        float targetDelta = 0.0f;

        float lastEncoderAmount = 0.0f;

        float pwm = 0.0f;

        float _s1 = 0;
        float _s2 = 0;

    private:
    void rotateTowardTargetAccDcc(); //target[mm] に向かって動く
        Ticker movementTicker;
        
};

