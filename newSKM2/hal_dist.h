//   ロボット名	： 電研ベーシックDCマウス、サンシャイン
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.28			sato			新規（ファイルのインクルード）
// *************************************************************************/
// 多重コンパイル抑止
#ifndef _HAL_DIST_H
#define _HAL_DIST_H

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O

//**************************************************
// 定義（define）
//**************************************************
/* 距離センサ */
#define		LED_DIST_RF		( PORTA.PODR.BIT.B6 )			// ビットアクセス
#define		LED_DIST_RS		( PORTA.PODR.BIT.B4 )			// ビットアクセス
#define		LED_DIST_LF		( PORTA.PODR.BIT.B1 )			// ビットアクセス
#define		LED_DIST_LS		( PORTA.PODR.BIT.B3 )			// ビットアクセス

//**************************************************
// 列挙体（enum）
//**************************************************
/* 距離センサID  */
typedef enum{
	DIST_SEN_R_FRONT = 0,			// 右前
	DIST_SEN_L_FRONT,			// 左前
	DIST_SEN_R_SIDE,			// 右横
	DIST_SEN_L_SIDE,			// 左横
	DIST_SEN_NUM
}enDIST_SEN_ID;

/*距離センサポーリングタイプ*/
typedef enum{
	DIST_POL_FRONT = 0,	//前壁
	DIST_POL_SIDE,		//横壁
	DISR_POL_MAX
}enDIST_POL;

/*距離センサ動作状態*/
typedef enum{
	DIST_STANDBAY = 0,	//スタンバイ
	DIST_NO_CTRL,		//制御不要
	DIST_NO_WALL,		//壁なし
	DIST_FRONT,		//前壁制御
	DIST_RIGHT,		//右壁制御
	DIST_LEFT,		//左壁制御
	DIST_STATE_MAX
}enDIST_SEN_STATE;

//**************************************************
// 構造体（struct）
//**************************************************


//**************************************************
// グローバル変数
//**************************************************




//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************
/* 距離センサ */
PUBLIC void DIST_Pol_Front( void );
PUBLIC void DIST_Pol_Side( void );
PUBLIC SHORT DIST_getNowVal( enDIST_SEN_ID en_id );
PUBLIC void DIST_getErr( LONG* p_err );

PUBLIC BOOL DIST_isWall_FRONT( void );
PUBLIC BOOL DIST_isWall_R_SIDE( void );
PUBLIC BOOL DIST_isWall_L_SIDE( void );

PUBLIC void Dist_autocalibration (void);
PUBLIC void calibration(void);

#endif
