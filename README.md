
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
2. 曲線に沿った移動(余裕があれば)

を作ります．


## 使い方

0. 注意
   
    パラメータや繰り返し使う定数はすべて`parameters.hpp`に書かれていて，いろんなところで使っています．
    これはプログラムが完成してから調整するのを簡単にするためです．
    プログラムを読んでて「何だこの定数！？」ってなったら大体ここに書かれてます．
    定数の中身はコメントを見れば大体わかると思います．

1. PID制御器
   
    PID制御は`PIDController`を使用して計算できます．制御頻度とゲインを指定し，`float PIDController::calculate(float error)`を指定の制御頻度のタイマー割り込み内で実行することで計算ができます．

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

    インクリメント数は，メンバ変数`int Encoder::IncrementedNum`，
    回転した距離[mm]は，メンバ関数`float Encoder::getAmount()`
    から取得できます．また，割り込み入力は`void Encoder::incrementedNum()`で行われています．それ以外の機能はありません．

3. 駆動用モーターの制御
    ```cpp
    #include "driveMotor.hpp"

    //引数は左から，エンコーダーのピンA，エンコーダーのピンB，モーターPWMのピン，モーターDIRのピン，Pゲイン，Iゲイン，Dゲイン
    DriveMotor motor(D9, D8, D12, D11, 1.3f, 0.06f, 0, 0.00003f, 0.000001f, 0);
    ```
    `motor.rotateTo(float x, bool idle=true);`を使って，モータを正確にx [mm]動かすことができます．デフォルトではモータを動かしている間は次の処理に進みませんが，第2引数に`false`を設定すると移動を待たないですぐ次の処理に進むようになります．

4. 足回り・自己位置制御
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

    ### 自己位置推定

    自己位置推定の`Localization`クラスは単体では使えません．`DriveBase`クラスのメンバ変数として`DriveBase::localization`があるので，これを使う感じになります．`DriveBase::localization`が持っている機能は以下の通りです

    #### ロボットの座標の設定

    1. 座標の設定：`void localization.setPosition(float X, float Y, float D);`

    ロボットの座標が何らかの形で確定するときに使ってください．
   
    <br/>

    #### エンコーダから推定されたロボットの位置

    2. X座標[mm]：`float localization.posX`
    3. Y座標[mm]：`float localization.posY`
    4. 方角[rad]：`float localization.direction`
   
    <br/>

    #### エンコーダから推定されたロボットの速度

    5. X方向[mm/s]：`float localization.speedX`  
    6. Y方向[mm/s]：`float localization.speedY`  
    7. 回転[rad/s]：`float localization.rotateSpeed`  

    <br/>

    また，これらの値の更新はタイマー割り込み(200Hz)を用いて行われており，割り込みの中身は`void localization.encoderLocalization();`です．

    ### モーターの制御

    自己位置推定での情報をもとに，ロボットを特定の座標に動かすことができます．**基本的に`driveBase.goTo(座標)`を並べるだけでどんな移動もできます．**(カーブとかはできません)

    #### 目標位置(X,Y,D)に向かって移動(加減速とPID制御を使用)

    8. 回転しながら直線移動する：`void DriveBase::goTo(float X, float Y, float D, bool idle=true);`
    9.  並進移動のみ．回転なし：`void DriveBase::goParallelTo(float X, float Y, bool idle=true)`
    10. 回転のみ．並進移動なし(超信地旋回)：`void DriveBase::rotateTo(float D, bool idle=true)`
   
   <br/>

    最後のbool引数`idle`はいずれも`DriveMotor`のときと同じで，デフォルトの`true`のときはモータを動かしている間は次の処理に進みませんが，`false`を設定すると移動を待たずにすぐに次の処理に進みます．モータのスピードの制御はタイマー割り込み(20Hz)を使用しており，割り込みの中身は`void DriveBase::goTowardTargetAccDcc();`で確認できます．モータが動いてるか否かは，メンバ変数`bool DriveMotor::moving`で確認できます．

    

## 説明

ここからは，プログラムの中身について制御初心者向けの説明をします．以下はこのリポジトリ内のプログラム全体のフローチャートです(かなり省略してますが)

説明しようと思ったんですが眠いので今日は寝ます

![DriveSystem](https://user-images.githubusercontent.com/139035878/258173851-2150bdb4-e32b-4fe9-99ad-47ba0cc4f7de.png)

1. 線形変換
   
   準備として，ロボットの速度ベクトルと足回りの4つのオムニホイールの回転速度の間の変換を考えてみます．
   ロボットの足回りのホイールに0,1,2,3と番号をつけ，それぞれのタイヤの向き(正方向に回転した時に進む方向)の単位ベクトルを，$\bm{n_0},\bm{n_1},\bm{n_2},\bm{n_3}$とします．ここでは，ロボットの向いている方向を基準とするxy座標系において，
   $$
   \bm{n_0} = \frac1{\sqrt{2}} \begin{pmatrix}-1\\1\end{pmatrix}\\
   \bm{n_1} = \frac1{\sqrt{2}} \begin{pmatrix}-1\\-1\end{pmatrix}\\
   \bm{n_2} = \frac1{\sqrt{2}} \begin{pmatrix}1\\-1\end{pmatrix}\\
   \bm{n_3} = \frac1{\sqrt{2}} \begin{pmatrix}1\\1\end{pmatrix}\\
   $$
   であるとします．また，ロボットの半径(4つのホイールの重心からホイールまでの距離と定義する)を$r$とおき，ロボットの重心をxy座標系の原点とすると，4つのホイールの位置ベクトルはそれぞれ$r\bm{n_3},r\bm{n_0},r\bm{n_1},r\bm{n_2}$と書くことができます．
   
   フィールド上にXY座標をとり，ロボットの重心の位置を$(X,Y)$，フィールドに対してロボットが向いている方向を$\theta$とします．回転行列を
   $R(\theta)=\begin{pmatrix}\cos\theta&-\sin\theta\\\sin\theta&\cos\theta\end{pmatrix}$と略記することにします．

   まず，XY座標系でのモーターの位置ベクトル$\bm{N_0},\bm{N_1},\bm{N_2},\bm{N_3}$はそれぞれ次のように書けます．
   $$\bm{N_0}=\begin{pmatrix}X\\Y\end{pmatrix}+rR(\theta)\bm{n_3}$$
   $$\bm{N_1}=\begin{pmatrix}X\\Y\end{pmatrix}+rR(\theta)\bm{n_0}$$
   $$\bm{N_2}=\begin{pmatrix}X\\Y\end{pmatrix}+rR(\theta)\bm{n_1}$$
   $$\bm{N_3}=\begin{pmatrix}X\\Y\end{pmatrix}+rR(\theta)\bm{n_2}$$
   両辺を微分して速度ベクトルを計算すると次のようになります
   $$\dot{\bm{N_0}}=\begin{pmatrix}\dot{X}\\\dot{Y}\end{pmatrix}+r\dot{\theta}R\left(\theta+\frac{\pi}{2}\right)\bm{n_3} = \begin{pmatrix}\dot{X}\\\dot{Y}\end{pmatrix}+r\dot{\theta}R(\theta)\bm{n_0}$$
   $$\dot{\bm{N_1}}=\begin{pmatrix}\dot{X}\\\dot{Y}\end{pmatrix}+r\dot{\theta}R\left(\theta+\frac{\pi}{2}\right)\bm{n_0} = \begin{pmatrix}\dot{X}\\\dot{Y}\end{pmatrix}+r\dot{\theta}R(\theta)\bm{n_1}$$
   $$\dot{\bm{N_2}}=\begin{pmatrix}\dot{X}\\\dot{Y}\end{pmatrix}+r\dot{\theta}R\left(\theta+\frac{\pi}{2}\right)\bm{n_1} = \begin{pmatrix}\dot{X}\\\dot{Y}\end{pmatrix}+r\dot{\theta}R(\theta)\bm{n_2}$$
   $$\dot{\bm{N_3}}=\begin{pmatrix}\dot{X}\\\dot{Y}\end{pmatrix}+r\dot{\theta}R\left(\theta+\frac{\pi}{2}\right)\bm{n_2} = \begin{pmatrix}\dot{X}\\\dot{Y}\end{pmatrix}+r\dot{\theta}R(\theta)\bm{n_3}$$

   ここで，モーターの回転速度$v_0,v_1,v_2,v_3$は，それぞれこれらの速度ベクトルのモーターの向き$R(\theta)\bm{n_0},R(\theta)\bm{n_1},R(\theta)\bm{n_2},R(\theta)\bm{n_3}$への射影となります．よって，内積を使って書くことができ，

   $$v_0=R(\theta)\bm{n_0}\cdot \dot{\bm{N_0}}=\bm{n_0}^TR(\theta)^T\begin{pmatrix}\dot{X}\\\dot{Y}\end{pmatrix}+r\dot{\theta} = \bm{n_0} \cdot R(-\theta)\begin{pmatrix}\dot{X}\\\dot{Y}\end{pmatrix}+r\dot{\theta}$$
   $$v_1=R(\theta)\bm{n_1}\cdot \dot{\bm{N_1}}=\bm{n_1}^TR(\theta)^T\begin{pmatrix}\dot{X}\\\dot{Y}\end{pmatrix}+r\dot{\theta} = \bm{n_1} \cdot R(-\theta)\begin{pmatrix}\dot{X}\\\dot{Y}\end{pmatrix}+r\dot{\theta}$$
   $$v_2=R(\theta)\bm{n_2}\cdot \dot{\bm{N_2}}=\bm{n_2}^TR(\theta)^T\begin{pmatrix}\dot{X}\\\dot{Y}\end{pmatrix}+r\dot{\theta} = \bm{n_2} \cdot R(-\theta)\begin{pmatrix}\dot{X}\\\dot{Y}\end{pmatrix}+r\dot{\theta}$$
   $$v_3=R(\theta)\bm{n_3}\cdot \dot{\bm{N_3}}=\bm{n_3}^TR(\theta)^T\begin{pmatrix}\dot{X}\\\dot{Y}\end{pmatrix}+r\dot{\theta} = \bm{n_3} \cdot R(-\theta)\begin{pmatrix}\dot{X}\\\dot{Y}\end{pmatrix}+r\dot{\theta}$$

   となります．
   
   

2. 自己位置推定

3. PID制御
   
4. 足回り制御