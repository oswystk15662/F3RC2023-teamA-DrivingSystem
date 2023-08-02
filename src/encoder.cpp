#include <mbed.h>
#include <math.h>
#include "parameters.hpp"
#include "encoder.hpp"

Encoder::Encoder(PinName pinA, PinName pinB): A(pinA), B(pinB) {
    A.rise([this]() {increment(1);}); //インクリメント
    //A.fall([this]() {increment(-1);});
}

void Encoder::increment(int sgn){
    //エンコーダーのインクリメント，デクリメント
    if (B.read() == 0){
        IncrementedNum += sgn;
    }else{
        IncrementedNum -= sgn;
    }
}

float Encoder::getAmount(){
    return MMPP * IncrementedNum;
}
