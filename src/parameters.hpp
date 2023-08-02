//パラメーター

#pragma once

//定数
#define PI (3.141592653589793)
#define SQRT2 (1.414213562373095)

//全部まだ仮の値

//固定値
#define ENC_RES_MAX (500) //ロータリエンコーダーの分解能
#define WHEEL_DIAMETER (88.0f)	//オムニホイールの直径	[mm]
#define WHEEL_RADIUS	(WHEEL_DIAMETER/2.0f) //オムニホイールの半径	[mm]
#define TRED_RADIUS (150.0f) //中心からオムニホイールの距離[mm]
#define MMPP 		(WHEEL_DIAMETER*PI)/(ENC_RES_MAX)	//エンコーダ1パルスあたりに進む距離[mm]
#define RADPP 	(MMPP/TRED_RADIUS)	//エンコーダ1パルスあたりの回転角[rad]

//モーターのPIDゲイン
#define MOTOR_0_KP (0.0f)
#define MOTOR_0_KI (0.0f)
#define MOTOR_0_KD (0.0f)

#define MOTOR_1_KP (0.0f)
#define MOTOR_1_KI (0.0f)
#define MOTOR_1_KD (0.0f)

#define MOTOR_2_KP (0.0f)
#define MOTOR_2_KI (0.0f)
#define MOTOR_2_KD (0.0f)

#define MOTOR_3_KP (0.0f)
#define MOTOR_3_KI (0.0f)
#define MOTOR_3_KD (0.0f)

//移動パラメータ
#define MAX_ACCELERATION (5000) //最大加速度  [mm/s^2]
#define MAX_SPEED (100) //最高速度  [mm/s^2]
#define MAX_ROTATE_ACCELERATION (2000) //最大回転速度  [mm/s^2]
#define MAX_ROTATE_SPEED (800) //最大回転加速度  [mm/s^2]

//制御周期など
#define ENCODER_LOCALIZATION_FREQUENCY (200) //エンコーダーによる自己位置推定の頻度[Hz]
#define SPEED_ADJUSTMENT_FREQUENCY (20) //速度調整の頻度[Hz]

//目的地到着を判定する閾値
#define MOVEMENT_THRESHOLD (20) //目的地に到着したとみなす半径[mm]
#define MOVEMENT_THRESHOLD_RAD (0.02f) //目的地に到着したとみなす角度の誤差[rad]

#define MAX_DELTA_R (MAX_SPEED)/(ENCODER_LOCALIZATION_FREQUENCY) 
#define MAX_DELTA_D (MAX_ROTATE_SPEED)/(ENCODER_LOCALIZATION_FREQUENCY)
#define MAX_DELTA_DELTA_R (MAX_ACCELERATION)/(ENCODER_LOCALIZATION_FREQUENCY*ENCODER_LOCALIZATION_FREQUENCY)
#define MAX_DELTA_DELTA_D (MAX_ROTATE_ACCELERATION)/(ENCODER_LOCALIZATION_FREQUENCY*ENCODER_LOCALIZATION_FREQUENCY)

