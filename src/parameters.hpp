//パラメーター

#pragma once

//定数
#define PI (3.141592653589793)
#define SQRT2 (1.414213562373095)

//全部まだ仮の値

//固定値
#define ENC_RES_MAX (500) //ロータリエンコーダーの分解能
#define TRED_RADIUS (150.0f) //中心からオムニホイールの距離[mm]
#define WHEEL_DIAMETER (88.0f)	//オムニホイールの直径	[mm]
#define WHEEL_RADIUS	(WHEEL_DIAMETER/2.0f) //オムニホイールの半径	[mm]
#define MMPP 		(WHEEL_DIAMETER*PI)/(ENC_RES_MAX)	//エンコーダ1パルスあたりに進む距離[mm]
#define RADPP 	(MMPP/TRED_RADIUS)	//エンコーダ1パルスあたりの回転角[rad]

//モーターのPIDゲイン

#define MOTOR_0_KP_1 (1.3f)
#define MOTOR_0_KI_1 (0.06f)
#define MOTOR_0_KD_1 (0.0f)

#define MOTOR_1_KP_1 (1.3f)
#define MOTOR_1_KI_1 (0.06f)
#define MOTOR_1_KD_1 (0.0f)

#define MOTOR_2_KP_1 (1.3f)
#define MOTOR_2_KI_1 (0.06f)
#define MOTOR_2_KD_1 (0.0f)

#define MOTOR_3_KP_1 (1.3f)
#define MOTOR_3_KI_1 (0.06f)
#define MOTOR_3_KD_1 (0.0f)

#define MOTOR_0_KP_2 (0.00003f)
#define MOTOR_0_KI_2 (0.000001f)
#define MOTOR_0_KD_2 (0.0f)

#define MOTOR_1_KP_2 (0.00003f)
#define MOTOR_1_KI_2 (0.000001f)
#define MOTOR_1_KD_2 (0.0f)

#define MOTOR_2_KP_2 (0.00003f)
#define MOTOR_2_KI_2 (0.000001f)
#define MOTOR_2_KD_2 (0.0f)

#define MOTOR_3_KP_2 (0.00003f)
#define MOTOR_3_KI_2 (0.000001f)
#define MOTOR_3_KD_2 (0.0f)

#define DRIVEBASE_KP (1.3f)
#define DRIVEBASE_KI (0.06f)
#define DRIVEBASE_KD (0.0f)

#define DRIVEBASE_ROTATE_KP (DRIVEBASE_KP)
#define DRIVEBASE_ROTATE_KI (DRIVEBASE_KI)
#define DRIVEBASE_ROTATE_KD (DRIVEBASE_KD)

//移動パラメータ
#define MAX_ACCELERATION (20000) //最大加速度 [mm/s^2]
#define MAX_SPEED (4000) //最高速度  [mm/s]
#define MAX_ROTATE_ACCELERATION (2000) //最大回転速度  [mm/s^2]
#define MAX_ROTATE_SPEED (800) //最大回転加速度  [mm/s^2]

//制御周期など
#define ENCODER_LOCALIZATION_FREQUENCY (200) //エンコーダーによる自己位置推定の頻度[Hz]
#define SPEED_ADJUSTMENT_FREQUENCY (20) //速度調整の頻度[Hz]

//目的地到着を判定する閾値
#define MOVEMENT_THRESHOLD (10) //目的地に到着したとみなす半径[mm]
#define MOVEMENT_THRESHOLD_RAD (0.02f) //目的地に到着したとみなす角度の誤差[rad]

#define MAX_DELTA_R (MAX_SPEED)/(SPEED_ADJUSTMENT_FREQUENCY) 
#define MAX_DELTA_D (MAX_ROTATE_SPEED)/(SPEED_ADJUSTMENT_FREQUENCY)

