#pragma once

class Encoder{
    public:
        InterruptIn A;
        DigitalIn B;
        void increment(int sgn);
        float getAmount();
        float getSpeed();
        int IncrementedNum = 0; //エンコーダーのカウント数
        Encoder(PinName pinA, PinName pinB);
};
