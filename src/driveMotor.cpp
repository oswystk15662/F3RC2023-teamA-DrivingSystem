#include <mbed.h>
#include <math.h>
#include "encoder.hpp"
#include "driveMotor.hpp"
#include "parameters.hpp"

//初期化
DriveMotor::DriveMotor(PinName encoder_pin_a, PinName encoder_pin_b, PinName pwm_pin, PinName dir_pin, float kp, float ki, float kd): encoder(encoder_pin_a, encoder_pin_b), pwmOut(pwm_pin), dirOut(dir_pin), pidController(SPEED_ADJUSTMENT_FREQUENCY,kp,ki,kd) {
    pidController.reset();
}

//PWMの書き込み
void DriveMotor::setPWM(float speed){
    if(speed > 0){
        pwmOut.write(speed);
        dirOut.write(1);
    }else{
        pwmOut.write(-speed);
        dirOut.write(0);
    }
}

//目標の位置に向かって回転
void DriveMotor::rotateToward(float target){
    float u = pidController.calculate(target - encoder.getAmount());
    setPWM(u);
}

//目標の方向に向かって回転
void DriveMotor::rotateTowardRelative(float distance){
    rotateToward(encoder.getAmount() + distance);
}

//停止
void DriveMotor::stop(){
    setPWM(0);
}


//現在決められている目標に向かって進む
//PID制御で目標値に向かうが，速度・加速度に制限を設けることで台形制御を実現する
void DriveMotor::rotateTowardTargetAccDcc(){

    //速度を制限する

    float delta = current_target - encoder.getAmount();
    if(delta > MAX_DELTA_R){
        delta = MAX_DELTA_R;
        current_target = encoder.getAmount() + delta;
    }else if(delta < -MAX_DELTA_R){
        delta = -MAX_DELTA_R;
        current_target = encoder.getAmount() + delta;
    }

    //加速度を制限する

    float delta_delta = delta - delta_before;
    if(delta > MAX_DELTA_DELTA_R){
        delta_delta = MAX_DELTA_DELTA_R;
        delta = delta_before + delta_delta;
        current_target = encoder.getAmount() + delta;
    }else if(delta < -MAX_DELTA_DELTA_R){
        delta_delta = -MAX_DELTA_DELTA_R;
        delta = delta_before + delta_delta;
        current_target = encoder.getAmount() + delta;
    }

    delta_before = delta;
    rotateTowardRelative(delta);

    if (abs(current_target - encoder.getAmount()) < MOVEMENT_THRESHOLD){
        moving = false;
    }
}

//目標まで移動
void DriveMotor::rotateTo(float target, bool idle){
    if(moving){
        printf("warning: a motion requested while the motor is moving.");
        movementTicker.detach();
    }
    moving = true;

    //PID制御器の初期化
    pidController.reset();

    //割り込みの設定
    movementTicker.attach([this] {rotateTowardTargetAccDcc();}, std::chrono::milliseconds(1000)/SPEED_ADJUSTMENT_FREQUENCY);

    //idle=trueなら移動が終わるまで待機
    if(idle){
        while(moving) {}
    }

}