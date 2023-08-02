#pragma once

class PIDController {
private:
    int FREQUENCY; //制御頻度
    float KP; // P制御のゲイン
    float KI; // I制御のゲイン
    float KD; // D制御のゲイン

    float prevError; // 前回のエラー
    float integral; // 積分値

public:
    PIDController(int frequency, float kp, float ki, float kd);
    float calculate(float error);
    void reset();
};
