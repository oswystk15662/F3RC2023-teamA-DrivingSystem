
# F3RC2023-teamA-DrivingSystem

F3RCの足回り周辺のプログラムです

## 機能

今のところ
1. PID制御器
2. ロータリエンコーダの値の読み取り
3. 駆動用モーターの制御
4. エンコーダー自己位置推定による目標位置への直線移動

をサポートしています．あとまだバグがあるかも．今後は，
1. レーザー自己位置推定，エンコーダー↔︎レーザーの切り替え
2. 曲線に沿った移動

を作ります．

## フローチャート

![DriveSystem](https://user-images.githubusercontent.com/139035878/258173851-2150bdb4-e32b-4fe9-99ad-47ba0cc4f7de.png)

## 使い方

0. 注意
   
    パラメータや繰り返し使う定数はすべてparameters.hppに書かれていて，いろんなところで使っています．
    これはプログラムが完成してから調整するのを簡単にするためです．
    プログラムを読んでて「何だこの定数！？」ってなったら大体ここに書かれてます．

1. PID制御器

    制御頻度とゲインを指定する感じです．
    ```cpp
    #include "PIDController.hpp"
    #include <mbed.h>
    #include <chrono>

    int frequency = 20; //制御頻度
    float target = 0; //目標値

    //PID制御器
    //引数は順に制御頻度，Pゲイン，Iゲイン，Dゲインを指定
    PIDController pid(frequency, 0.01f, 0.01f, 0.0f);

    Ticker ticker; //タイマー割り込み

    //割り込み
    int interrupt(){
        float x = センサー.read(); //センサーの値
        float u = pid.get(target - x); //計算
        モーター.write(u); //出力
    }

    int main(){
        //初期化
        pid.reset();

        //割り込みの設定
        ticker.attach(&interrupt, std::chrono::milliseconds(1000)/frequency);

        while(true) {} 
    }
    ```

2. ロータリエンコーダ
    ```cpp
    #include "encoder.hpp"

    //左からA, Bのピン
    Encoder encoder(A1, A2);
    ```
    でインスタンスを生成します．

    インクリメント数は，メンバ変数`int Encoder::IncrementedNum`

    回転した距離[mm]は，メンバ関数`float Encoder::getAmount(void)`

    から取得できます．

3. 駆動用モーターの制御
    ```cpp
    #include "driveMotor.hpp"

    //引数は左から，エンコーダーのピンA，エンコーダーのピンB，モーターPWMのピン，モーターDIRのピン，Pゲイン，Iゲイン，Dゲイン
    DriveMotor motor(D9, D8, D12, D11, 1.3f, 0.06f, 0, 0.00003f, 0.000001f, 0);
    ```
    詳しいことは後で書きます

4. 足回り
    ```cpp
    #include <mbed.h>
    #include "driveBase.hpp"
    #include "driveMotor.hpp"
    #include "parameters.hpp"

    int main(){
        //モーター
        DriveMotor motor0(PA_0, PA_1, PA_2, PA_3, MOTOR_0_KP_1, MOTOR_0_KI_1, MOTOR_0_KD_1, MOTOR_0_KP_1, MOTOR_1_KI_1, MOTOR_2_KD_1);
        DriveMotor motor1(PA_4, PA_5, PA_6, PA_7, MOTOR_1_KP_1, MOTOR_1_KI_1, MOTOR_1_KD_1, MOTOR_0_KP_1, MOTOR_1_KI_1, MOTOR_2_KD_1);
        DriveMotor motor2(PA_8, PA_9, PA_10, PA_11, MOTOR_2_KP_1, MOTOR_2_KI_1, MOTOR_2_KD_1, MOTOR_0_KP_1, MOTOR_1_KI_1, MOTOR_2_KD_1);
        DriveMotor motor3(PA_12, PA_13, PA_14, PA_15, MOTOR_3_KP_1, MOTOR_3_KI_1, MOTOR_3_KD_1, MOTOR_0_KP_1, MOTOR_1_KI_1, MOTOR_2_KD_1);

        //足回り
        DriveBase driveBase(&motor0, &motor1, &motor2, &motor3, DRIVEBASE_KP, DRIVEBASE_KI, DRIVEBASE_KD, DRIVEBASE_ROTATE_KP, DRIVEBASE_ROTATE_KI, DRIVEBASE_ROTATE_KD);

        //現在の座標を設定
        driveBase.localization.setPosition(0,0,0);

        //目的地に移動
        driveBase.goTo(1000, 500, PI/2);
    }
    ```
    詳しいことは後で書きます

    


